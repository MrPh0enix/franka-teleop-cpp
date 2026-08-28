#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

// Key listener
#include <fcntl.h>
#include <termios.h>

// UDP socket
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

// Cap'n Proto message
#include <capnp/message.h>
#include <capnp/serialize.h>
#include "messages/robot-state.capnp.h"

// Franka libraries
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include "examples_common.h"

namespace
{

    constexpr std::size_t kJointCount = 7;
    constexpr std::size_t kBufferSize = 2048;
    constexpr std::size_t kRecordQueueCapacity = 65536;
    using Clock = std::chrono::steady_clock;

  struct RemoteRobotData
  {
    std::array<double, kJointCount> pos{};
    std::array<double, kJointCount> vel{};
    std::array<double, kJointCount> ext_trq{};
    std::array<double, kJointCount> trq{};
    std::array<double, kJointCount> trq_der{};
    double time = 0.0;
    double gripper_width = 0.1;
  };

  struct PublishedRobotData
  {
    std::array<double, kJointCount> pos{};
    std::array<double, kJointCount> vel{};
    std::array<double, kJointCount> ext_trq{};
    std::array<double, kJointCount> trq{};
    std::array<double, kJointCount> trq_der{};
    double gripper_width = 0.1;
  };

    struct RecordSample
    {
        double local_time = 0.0;
        double robot_time = 0.0;
        bool follower_connected = false;
        std::array<double, kJointCount> leader_pos{};
        std::array<double, kJointCount> leader_vel{};
        std::array<double, kJointCount> leader_ext_trq{};
        std::array<double, kJointCount> leader_trq{};
        std::array<double, kJointCount> follower_pos{};
        std::array<double, kJointCount> follower_vel{};
        std::array<double, kJointCount> follower_ext_trq{};
        std::array<double, kJointCount> follower_trq{};
        std::array<double, kJointCount> desired_acc{};
        std::array<double, kJointCount> command_trq{};
    };

    std::array<RecordSample, kRecordQueueCapacity> record_queue;
    std::atomic<std::size_t> record_write_index{0};
    std::atomic<std::size_t> record_read_index{0};
    std::atomic<std::uint64_t> dropped_record_samples{0};
    std::atomic<bool> recording{false};

    RemoteRobotData shared_follower_data;
    PublishedRobotData shared_leader_data;

    std::mutex remote_state_mutex;
    std::mutex publish_state_mutex;
    std::atomic<bool> running{true};
    std::atomic<bool> sub_connected{false};
    std::atomic<std::int64_t> last_rx_ns{0};

    enum class GripperCommand
    {
        kNone,
        kOpen,
        kClose
    };
    std::atomic<GripperCommand> gripper_command{GripperCommand::kNone};

    std::int64_t nowNanoseconds()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   Clock::now().time_since_epoch())
            .count();
    }

    double nowSeconds()
    {
        return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    }

    int readOptionalInt(const YAML::Node &config, const char *section, const char *key, int fallback)
    {
        const YAML::Node value = config[section][key];
        return value ? value.as<int>() : fallback;
    }

    double readOptionalDouble(const YAML::Node &config, const char *section, const char *key, double fallback)
    {
        const YAML::Node value = config[section][key];
        return value ? value.as<double>() : fallback;
    }

    void setReceiveTimeout(int socket_fd, int timeout_ms)
    {
        timeval timeout{};
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                       sizeof(timeout)) < 0)
        {
            throw std::runtime_error("Failed to set UDP receive timeout");
        }
    }

    // Recording functions
    bool enqueueRecordSample(const RecordSample &sample)
    {
        const std::size_t write = record_write_index.load(std::memory_order_relaxed);
        const std::size_t next = (write + 1) % kRecordQueueCapacity;
        if (next == record_read_index.load(std::memory_order_acquire))
        {
            dropped_record_samples.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        record_queue[write] = sample;
        record_write_index.store(next, std::memory_order_release);
        return true;
    }

    bool dequeueRecordSample(RecordSample &sample)
    {
        const std::size_t read = record_read_index.load(std::memory_order_relaxed);
        if (read == record_write_index.load(std::memory_order_acquire))
        {
            return false;
        }
        sample = record_queue[read];
        record_read_index.store((read + 1) % kRecordQueueCapacity,
                                std::memory_order_release);
        return true;
    }

    std::string makeRecordingFilename()
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        localtime_r(&time, &tm);
        std::ostringstream stream;
        stream << "teleop_recording_" << std::put_time(&tm, "%Y%m%d_%H%M%S")
               << ".csv";
        return stream.str();
    }

    void writeArrayHeader(std::ofstream &file, const char *prefix)
    {
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            file << ',' << prefix << (i + 1);
        }
    }

    void writeArray(std::ofstream &file,
                    const std::array<double, kJointCount> &values)
    {
        for (double value : values)
        {
            file << ',' << value;
        }
    }

    void recorderThread()
    {
        std::ofstream file;
        bool session_active = false;
        std::uint64_t session_dropped_start = 0;
        auto last_flush = Clock::now();

        while (running.load(std::memory_order_relaxed) ||
               record_read_index.load(std::memory_order_acquire) !=
                   record_write_index.load(std::memory_order_acquire))
        {
            const bool should_record = recording.load(std::memory_order_acquire);

            if (should_record && !session_active)
            {
                const std::string filename = makeRecordingFilename();
                file.open(filename, std::ios::out | std::ios::trunc);
                if (!file)
                {
                    std::cerr << "Could not open recording file: " << filename << '\n';
                    recording.store(false, std::memory_order_release);
                }
                else
                {
                    file << std::setprecision(17);
                    file << "local_time_s,robot_time_s,follower_connected";
                    writeArrayHeader(file, "leader_q");
                    writeArrayHeader(file, "leader_dq");
                    writeArrayHeader(file, "leader_tau_ext");
                    writeArrayHeader(file, "leader_tau");
                    writeArrayHeader(file, "follower_q");
                    writeArrayHeader(file, "follower_dq");
                    writeArrayHeader(file, "follower_tau_ext");
                    writeArrayHeader(file, "follower_tau");
                    writeArrayHeader(file, "desired_ddq");
                    writeArrayHeader(file, "command_tau");
                    file << '\n';
                    session_dropped_start =
                        dropped_record_samples.load(std::memory_order_relaxed);
                    session_active = true;
                    last_flush = Clock::now();
                    std::cout << "Recording started: " << filename << '\n';
                }
            }

            RecordSample sample;
            bool wrote_sample = false;
            while (dequeueRecordSample(sample))
            {
                if (session_active)
                {
                    file << sample.local_time << ',' << sample.robot_time << ','
                         << (sample.follower_connected ? 1 : 0);
                    writeArray(file, sample.leader_pos);
                    writeArray(file, sample.leader_vel);
                    writeArray(file, sample.leader_ext_trq);
                    writeArray(file, sample.leader_trq);
                    writeArray(file, sample.follower_pos);
                    writeArray(file, sample.follower_vel);
                    writeArray(file, sample.follower_ext_trq);
                    writeArray(file, sample.follower_trq);
                    writeArray(file, sample.desired_acc);
                    writeArray(file, sample.command_trq);
                    file << '\n';
                    wrote_sample = true;
                }
            }

            if (!should_record && session_active)
            {
                file.flush();
                file.close();
                const auto dropped = dropped_record_samples.load(std::memory_order_relaxed) -
                                     session_dropped_start;
                std::cout << "Recording stopped. Dropped samples: " << dropped << '\n';
                session_active = false;
            }
            else if (wrote_sample && Clock::now() - last_flush >= std::chrono::seconds(1))
            {
                // Periodic flushing is intentionally done outside the control callback.
                file.flush();
                last_flush = Clock::now();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        if (file.is_open())
        {
            file.flush();
            file.close();
        }
    }

    // Publisher and subscriber functions
    void publisherThread(const YAML::Node &config)
    {
        const std::string ip = config["leader"]["ip"].as<std::string>();
        const int port = config["leader"]["port"].as<int>();
        const int publish_frequency = config["global"]["freq"].as<int>();

        if (publish_frequency <= 0)
        {
            std::cerr << "Publisher frequency must be positive.\n";
            running.store(false);
            return;
        }

        const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0)
        {
            perror("Publisher socket creation failed");
            running.store(false);
            return;
        }

        sockaddr_in send_address{};
        send_address.sin_family = AF_INET;
        send_address.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &send_address.sin_addr) != 1)
        {
            std::cerr << "Invalid leader IP address: " << ip << '\n';
            close(socket_fd);
            running.store(false);
            return;
        }

        const auto period =
            std::chrono::nanoseconds(1'000'000'000LL / publish_frequency);
        auto next_wakeup = Clock::now();

        while (running.load(std::memory_order_relaxed))
        {
            next_wakeup += period;

            PublishedRobotData snapshot;
            {
                std::lock_guard<std::mutex> lock(publish_state_mutex);
                snapshot = shared_leader_data;
            }

            capnp::MallocMessageBuilder message;
            RobotState::Builder state = message.initRoot<RobotState>();

            state.setTime(nowSeconds());
            state.setJoint1Pos(snapshot.pos[0]);
            state.setJoint2Pos(snapshot.pos[1]);
            state.setJoint3Pos(snapshot.pos[2]);
            state.setJoint4Pos(snapshot.pos[3]);
            state.setJoint5Pos(snapshot.pos[4]);
            state.setJoint6Pos(snapshot.pos[5]);
            state.setJoint7Pos(snapshot.pos[6]);
            state.setJoint1Vel(snapshot.vel[0]);
            state.setJoint2Vel(snapshot.vel[1]);
            state.setJoint3Vel(snapshot.vel[2]);
            state.setJoint4Vel(snapshot.vel[3]);
            state.setJoint5Vel(snapshot.vel[4]);
            state.setJoint6Vel(snapshot.vel[5]);
            state.setJoint7Vel(snapshot.vel[6]);
            state.setJoint1Torque(snapshot.trq[0]);
            state.setJoint2Torque(snapshot.trq[1]);
            state.setJoint3Torque(snapshot.trq[2]);
            state.setJoint4Torque(snapshot.trq[3]);
            state.setJoint5Torque(snapshot.trq[4]);
            state.setJoint6Torque(snapshot.trq[5]);
            state.setJoint7Torque(snapshot.trq[6]);
            state.setJoint1ExtTorque(snapshot.ext_trq[0]);
            state.setJoint2ExtTorque(snapshot.ext_trq[1]);
            state.setJoint3ExtTorque(snapshot.ext_trq[2]);
            state.setJoint4ExtTorque(snapshot.ext_trq[3]);
            state.setJoint5ExtTorque(snapshot.ext_trq[4]);
            state.setJoint6ExtTorque(snapshot.ext_trq[5]);
            state.setJoint7ExtTorque(snapshot.ext_trq[6]);
            state.setGripperWidth(snapshot.gripper_width);
            state.setJoint1ExtTorqueDer(snapshot.trq_der[0]);
            state.setJoint2ExtTorqueDer(snapshot.trq_der[1]);
            state.setJoint3ExtTorqueDer(snapshot.trq_der[2]);
            state.setJoint4ExtTorqueDer(snapshot.trq_der[3]);
            state.setJoint5ExtTorqueDer(snapshot.trq_der[4]);
            state.setJoint6ExtTorqueDer(snapshot.trq_der[5]);
            state.setJoint7ExtTorqueDer(snapshot.trq_der[6]);

            kj::VectorOutputStream output;
            capnp::writeMessage(output, message);
            const kj::ArrayPtr<const kj::byte> bytes = output.getArray();

            const ssize_t sent = sendto(socket_fd, bytes.begin(), bytes.size(), 0,
                                        reinterpret_cast<sockaddr *>(&send_address),
                                        sizeof(send_address));
            if (sent < 0 && running.load(std::memory_order_relaxed))
            {
                perror("Failed to publish leader state");
            }

            std::this_thread::sleep_until(next_wakeup);
        }

        close(socket_fd);
    }

    void subscriberThread(const YAML::Node &config)
    {
        const int port = config["follower"]["port"].as<int>();
        const int receive_timeout_ms =
            readOptionalInt(config, "global", "udp_receive_timeout_ms", 100);

        const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0)
        {
            perror("Subscriber socket creation failed");
            running.store(false);
            return;
        }

        try
        {
            setReceiveTimeout(socket_fd, receive_timeout_ms);
        }
        catch (const std::exception &exception)
        {
            std::cerr << exception.what() << '\n';
            close(socket_fd);
            running.store(false);
            return;
        }

        sockaddr_in receive_address{};
        receive_address.sin_family = AF_INET;
        receive_address.sin_port = htons(port);
        receive_address.sin_addr.s_addr = INADDR_ANY;

        if (bind(socket_fd, reinterpret_cast<sockaddr *>(&receive_address),
                 sizeof(receive_address)) < 0)
        {
            perror("Subscriber socket bind failed");
            close(socket_fd);
            running.store(false);
            return;
        }

        std::cout << "Subscriber listening on port: " << port << '\n';
        std::array<char, kBufferSize> buffer{};

        while (running.load(std::memory_order_relaxed))
        {
            const ssize_t received =
                recvfrom(socket_fd, buffer.data(), buffer.size(), 0, nullptr, nullptr);

            if (received < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                {
                    continue;
                }
                if (running.load(std::memory_order_relaxed))
                {
                    perror("Failed to receive follower state");
                }
                break;
            }

            if (received == 0)
            {
                continue;
            }

            try
            {
                std::vector<capnp::word> aligned_buffer(
                    (static_cast<std::size_t>(received) + sizeof(capnp::word) - 1) /
                    sizeof(capnp::word));
                std::memcpy(aligned_buffer.data(), buffer.data(),
                            static_cast<std::size_t>(received));

                const kj::ArrayPtr<const capnp::word> received_data(
                    aligned_buffer.data(), aligned_buffer.size());
                capnp::FlatArrayMessageReader reader(received_data);
                const RobotState::Reader state = reader.getRoot<RobotState>();

                RemoteRobotData snapshot;
                snapshot.time = state.getTime();
                snapshot.pos = {state.getJoint1Pos(), state.getJoint2Pos(),
                                state.getJoint3Pos(), state.getJoint4Pos(),
                                state.getJoint5Pos(), state.getJoint6Pos(),
                                state.getJoint7Pos()};
                snapshot.vel = {state.getJoint1Vel(), state.getJoint2Vel(),
                                state.getJoint3Vel(), state.getJoint4Vel(),
                                state.getJoint5Vel(), state.getJoint6Vel(),
                                state.getJoint7Vel()};
                snapshot.trq = {state.getJoint1Torque(), state.getJoint2Torque(),
                                state.getJoint3Torque(), state.getJoint4Torque(),
                                state.getJoint5Torque(), state.getJoint6Torque(),
                                state.getJoint7Torque()};
                snapshot.ext_trq = {
                    state.getJoint1ExtTorque(), state.getJoint2ExtTorque(),
                    state.getJoint3ExtTorque(), state.getJoint4ExtTorque(),
                    state.getJoint5ExtTorque(), state.getJoint6ExtTorque(),
                    state.getJoint7ExtTorque()};
                snapshot.trq_der = {
                    state.getJoint1ExtTorqueDer(), state.getJoint2ExtTorqueDer(),
                    state.getJoint3ExtTorqueDer(), state.getJoint4ExtTorqueDer(),
                    state.getJoint5ExtTorqueDer(), state.getJoint6ExtTorqueDer(),
                    state.getJoint7ExtTorqueDer()};
                snapshot.gripper_width = state.getGripperWidth();

                {
                    std::lock_guard<std::mutex> lock(remote_state_mutex);
                    shared_follower_data = snapshot;
                }
                last_rx_ns.store(nowNanoseconds(), std::memory_order_release);
                sub_connected.store(true, std::memory_order_release);
            }
            catch (const kj::Exception &exception)
            {
                std::cerr << "Invalid follower packet: " << exception.getDescription().cStr()
                          << '\n';
            }
        }

        sub_connected.store(false, std::memory_order_release);
        close(socket_fd);
    }

    // Key listener function
    void keyListener()
    {
        termios old_settings{};
        if (tcgetattr(STDIN_FILENO, &old_settings) != 0)
        {
            perror("tcgetattr failed");
            running.store(false);
            return;
        }

        termios new_settings = old_settings;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

        const int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

        std::cout << "Running ... "
                  << "C = close grippers, "
                  << "O = open grippers, "
                  << "R = recording, "
                  << "Q = exit.\n";

        while (running.load(std::memory_order_relaxed))
        {
            const int key = getchar();
            if (key == 'c' || key == 'C')
            {
                gripper_command.store(GripperCommand::kClose, std::memory_order_release);
                std::cout << "Close grippers requested.\n";
            }
            else if (key == 'o' || key == 'O')
            {
                gripper_command.store(GripperCommand::kOpen, std::memory_order_release);
                std::cout << "Open grippers requested.\n";
            }
            else if (key == 'r' || key == 'R')
            {
                const bool new_state = !recording.load(std::memory_order_relaxed);
                recording.store(new_state, std::memory_order_release);
                std::cout << (new_state ? "Recording requested.\n" : "Stop recording requested.\n");
            }
            else if (key == 'q' || key == 'Q')
            {
                running.store(false);
                std::cout << "Q pressed.\n";
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
        fcntl(STDIN_FILENO, F_SETFL, old_flags);
    }

    void gripperThread(const YAML::Node &config)
    {

        try
        {
            franka::Gripper gripper(config["leader"]["robot"].as<std::string>());
            const double open_width = config["gripper"]["open_threshold"].as<double>();
            const double close_width = 0.0;
            const double speed = config["gripper"]["speed"].as<double>();

            while (running.load(std::memory_order_relaxed))
            {
                const GripperCommand command = gripper_command.exchange(GripperCommand::kNone, std::memory_order_acq_rel);

                if (command == GripperCommand::kClose)
                {
                    std::cout << "Closing leader gripper...\n";

                    const bool success = gripper.move(close_width, speed);

                    if (!success)
                    {
                        std::cerr << "Leader gripper close failed.\n";
                    }
                }
                else if (command == GripperCommand::kOpen)
                {
                    std::cout << "Opening leader gripper...\n";

                    const bool success =
                        gripper.move(open_width, speed);

                    if (!success)
                    {
                        std::cerr << "Leader gripper open failed.\n";
                    }
                }

                const franka::GripperState gripper_state = gripper.readOnce();

                {
                    std::lock_guard<std::mutex> lock(publish_state_mutex);
                    shared_leader_data.gripper_width =
                        gripper_state.width;
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(20));
            }
        }
        catch (const franka::Exception &exception)
        {
            std::cerr << "Leader gripper error: "
                      << exception.what() << '\n';

            running.store(false);
        }
    }

} // namespace end

int main()
{
    try
    {
        const YAML::Node config = YAML::LoadFile("../teleop_config.yml");

        const std::vector<double> c_q = config["global"]["C_q"].as<std::vector<double>>();
        const std::vector<double> c_v = config["global"]["C_v"].as<std::vector<double>>();
        const std::vector<double> c_y = config["global"]["C_y"].as<std::vector<double>>();
        const std::vector<double> c_f = config["global"]["C_f"].as<std::vector<double>>();

        if (c_q.size() != kJointCount || c_v.size() != kJointCount ||
            c_y.size() != kJointCount || c_f.size() != kJointCount)
        {
            throw std::runtime_error("C_q, C_v, C_y, and C_f must each contain 7 values");
        }

        const int stale_timeout_ms = readOptionalInt(config, "global", "stale_packet_timeout_ms", 100);
        const std::int64_t stale_timeout_ns = static_cast<std::int64_t>(stale_timeout_ms) * 1'000'000LL;

        // Disturbance-observer parameters. Start conservatively and tune gradually.
        const double g_dob = readOptionalDouble(config, "global", "g_dob", 50.0);
        const double dob_gain = readOptionalDouble(config, "global", "dob_gain", 1);
        const double max_dob_torque = readOptionalDouble(config, "global", "max_dob_torque", 50.0);

        if (!(g_dob > 0.0) || !(dob_gain >= 0.0) ||
            !(max_dob_torque > 0.0))
        {
            throw std::runtime_error("g_dob and max_dob_torque must be positive, and dob_gain must be non-negative");
        }

        franka::Robot robot(config["leader"]["robot"].as<std::string>());
        const franka::Model model = robot.loadModel();

        const std::array<double, kJointCount> home_position = {0.0, -0.78539816, 0.0, -2.35619449, 1.57, 1.57079633, 0.78539816};
        MotionGenerator motion_generator(0.5, home_position);
        robot.control(motion_generator);

        robot.setCollisionBehavior(
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        const franka::RobotState initial_state = robot.readOnce();
        {
            std::lock_guard<std::mutex> lock(publish_state_mutex);
            shared_leader_data.pos = initial_state.theta;
            shared_leader_data.vel.fill(0.0);
            shared_leader_data.trq = initial_state.tau_J;
            shared_leader_data.ext_trq = initial_state.tau_ext_hat_filtered;
            shared_leader_data.trq_der = initial_state.dtau_J;
        }

        std::thread publisher(publisherThread, std::cref(config));
        std::thread subscriber(subscriberThread, std::cref(config));
        std::thread gripper(gripperThread, std::cref(config));
        std::thread recorder(recorderThread);
        std::thread keyboard(keyListener);

        RemoteRobotData cached_follower_data;

        // Persistent state of the seven independent joint-space DOB filters.
        std::array<double, kJointCount> dob_lpf_output{};
        std::array<bool, kJointCount> dob_initialized{};

        // Persistent state for velocity obtained by differentiating joint position.
        std::array<double, kJointCount> previous_position = initial_state.theta;
        std::array<double, kJointCount> differentiated_velocity{};
        bool differentiator_initialized = false;

        // control loop
        const auto torque_callback = [&](const franka::RobotState &robot_state, franka::Duration period) -> franka::Torques
        {
            if (!running.load(std::memory_order_relaxed))
            {
                return franka::MotionFinished(
                    franka::Torques({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
            }

            // The first callback can report a zero period; use 1 ms in that case.
            const double measured_dt = period.toSec();
            const double dt = (std::isfinite(measured_dt) && measured_dt > 0.0 && measured_dt

                                                                                      <= 0.01)
                                  ? measured_dt
                                  : 0.001;

            // Backward-difference velocity estimate from motor-side position theta.
            if (!differentiator_initialized)
            {
                previous_position = robot_state.theta;
                differentiated_velocity.fill(0.0);
                differentiator_initialized = true;
            }
            else
            {
                for (std::size_t i = 0; i < kJointCount; ++i)
                {
                    differentiated_velocity[i] = (robot_state.theta[i] - previous_position[i]) / dt;
                    previous_position[i] = robot_state.theta[i];
                }
            }

            // Never block the 1 kHz callback on a publisher thread.
            if (publish_state_mutex.try_lock())
            {
                shared_leader_data.pos = robot_state.theta;
                shared_leader_data.vel = differentiated_velocity;
                shared_leader_data.trq = robot_state.tau_J;
                shared_leader_data.ext_trq = robot_state.tau_ext_hat_filtered;
                shared_leader_data.trq_der = robot_state.dtau_J;
                publish_state_mutex.unlock();
            }

            std::array<double, kJointCount> command_torques{};

            if (!sub_connected.load(std::memory_order_acquire))
            {
                return command_torques;
            }

            const std::int64_t packet_age_ns =
                nowNanoseconds() - last_rx_ns.load(std::memory_order_acquire);
            if (packet_age_ns < 0 || packet_age_ns > stale_timeout_ns)
            {
                sub_connected.store(false, std::memory_order_release);
                return command_torques;
            }

            // Never wait for the UDP thread. Reuse the last complete snapshot if busy.
            if (remote_state_mutex.try_lock())
            {
                cached_follower_data = shared_follower_data;
                remote_state_mutex.unlock();
            }

            const std::array<double, 49> mass_matrix = model.mass(robot_state);
            std::array<double, kJointCount> desired_acceleration{};

            for (std::size_t i = 0; i < kJointCount; ++i)
            {

#ifdef TELEOP_BILATERAL

                const double position_error = robot_state.theta[i] - cached_follower_data.pos[i];

                const double velocity_error = differentiated_velocity[i] - cached_follower_data.vel[i];

                const double velocity_sum = differentiated_velocity[i] + cached_follower_data.vel[i];

                const double external_torque_sum = robot_state.tau_ext_hat_filtered[i] + cached_follower_data.ext_trq[i];

                desired_acceleration[i] =
                    -(c_q[i] / 2.0) * position_error -
                    (c_v[i] / 2.0) * velocity_error -
                    (c_y[i] / 2.0) * velocity_sum -
                    (c_f[i] / 2.0) * external_torque_sum;

                const double nominal_inertia = mass_matrix[i * kJointCount + i];

                const double nominal_torque = nominal_inertia * desired_acceleration[i];

                const double omega = differentiated_velocity[i];

                if (!dob_initialized[i])
                {
                    dob_lpf_output[i] = nominal_inertia * g_dob * omega;
                    dob_initialized[i] = true;
                }

                const double dob_input = nominal_torque + nominal_inertia * g_dob * omega;

                const double lpf_output = (1.0 / (g_dob * dt + 1.0)) * (dob_lpf_output[i] + g_dob * dt * dob_input);

                double tau_dis_hat = lpf_output - nominal_inertia * g_dob * omega;

                tau_dis_hat = std::clamp(tau_dis_hat, -max_dob_torque, max_dob_torque);

                command_torques[i] = nominal_torque + dob_gain * tau_dis_hat;

                dob_lpf_output[i] = lpf_output;

#elif defined(TELEOP_UNILATERAL)

                desired_acceleration[i] = 0.0;

                const double nominal_inertia = mass_matrix[i * kJointCount + i];

                command_torques[i] = nominal_inertia * desired_acceleration[i];

#else
#error "A teleoperation mode must be selected"
#endif
            }

            const std::array<double, kJointCount> rate_limited_torques = franka::limitRate(franka::kMaxTorqueRate, command_torques, robot_state.tau_J_d);

            if (recording.load(std::memory_order_relaxed))
            {
                RecordSample sample;
                sample.local_time = nowSeconds();
                sample.robot_time = robot_state.time.toSec();
                sample.follower_connected = sub_connected.load(std::memory_order_relaxed);
                sample.leader_pos = robot_state.theta;
                sample.leader_vel = differentiated_velocity;
                sample.leader_ext_trq = robot_state.tau_ext_hat_filtered;
                sample.leader_trq = robot_state.tau_J;
                sample.follower_pos = cached_follower_data.pos;
                sample.follower_vel = cached_follower_data.vel;
                sample.follower_ext_trq = cached_follower_data.ext_trq;
                sample.follower_trq = cached_follower_data.trq;
                sample.desired_acc = desired_acceleration;
                sample.command_trq = rate_limited_torques;
                enqueueRecordSample(sample);
            }

            return rate_limited_torques;
        };

        while (running.load(std::memory_order_relaxed))
        {
            try
            {
                robot.control(torque_callback);
            }
            catch (const franka::Exception &exception)
            {
                std::cerr << exception.what() << '\n';
                if (!running.load(std::memory_order_relaxed))
                {
                    break;
                }
                robot.automaticErrorRecovery();
            }
        }

        recording.store(false, std::memory_order_release);
        running.store(false);

        if (publisher.joinable())
            publisher.join();
        if (subscriber.joinable())
            subscriber.join();
        if (gripper.joinable())
            gripper.join();
        if (keyboard.joinable())
            keyboard.join();
        if (recorder.joinable())
            recorder.join();
    }
    catch (const std::exception &exception)
    {
        running.store(false);
        std::cerr << "Exception: " << exception.what() << '\n';
        return -1;
    }
    catch (...)
    {
        running.store(false);
        std::cerr << "Unknown exception caught\n";
        return -1;
    }

    return 0;
}
