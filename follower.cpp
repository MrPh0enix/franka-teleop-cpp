
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

// Key listener
#include <termios.h>
#include <fcntl.h>

// for UDP socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

//capnp message
#include <capnp/message.h>
#include <capnp/serialize.h>
#include "messages/robot-state.capnp.h"

//franka libs
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include "examples_common.h"



#define BUFFER_SIZE 2048

RobotState::Reader shared_leader_state;
franka::RobotState shared_robot_state;
std::mutex state_mutex;
std::atomic<bool> running{true};
std::atomic<bool> sub_connected{false}; // detects if subscriber connected
std::atomic<char> control_rob{'L'}; //default to leader(L)



void pubThread (const YAML::Node& config) {

    //config
    std::string ip = config["follower"]["ip"].as<std::string>();
    int port = config["follower"]["port"].as<int>();

    // socket params
    int sockPub = socket(AF_INET, SOCK_DGRAM, 0);
    char buffer[BUFFER_SIZE];
    struct sockaddr_in send_addr{};
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &send_addr.sin_addr);

    const int pub_freq = config["global"]["freq"].as<int>(); //Hz 

    while (running.load()) {

        //build message
        capnp::MallocMessageBuilder message;
        RobotState::Builder follower_state = message.initRoot<RobotState>();
        franka::RobotState state_to_publish;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state_to_publish = shared_robot_state;
        }

        follower_state.setTime(123456);
        follower_state.setJoint1Pos(state_to_publish.q[0]);
        follower_state.setJoint2Pos(state_to_publish.q[1]);
        follower_state.setJoint3Pos(state_to_publish.q[2]);
        follower_state.setJoint4Pos(state_to_publish.q[3]);
        follower_state.setJoint5Pos(state_to_publish.q[4]);
        follower_state.setJoint6Pos(state_to_publish.q[5]);
        follower_state.setJoint7Pos(state_to_publish.q[6]);
        follower_state.setJoint1Vel(state_to_publish.dq[0]);
        follower_state.setJoint2Vel(state_to_publish.dq[1]);
        follower_state.setJoint3Vel(state_to_publish.dq[2]);
        follower_state.setJoint4Vel(state_to_publish.dq[3]);
        follower_state.setJoint5Vel(state_to_publish.dq[4]);
        follower_state.setJoint6Vel(state_to_publish.dq[5]);
        follower_state.setJoint7Vel(state_to_publish.dq[6]);
        follower_state.setJoint1Torque(state_to_publish.tau_J[0]);
        follower_state.setJoint2Torque(state_to_publish.tau_J[1]);
        follower_state.setJoint3Torque(state_to_publish.tau_J[2]);
        follower_state.setJoint4Torque(state_to_publish.tau_J[3]);
        follower_state.setJoint5Torque(state_to_publish.tau_J[4]);
        follower_state.setJoint6Torque(state_to_publish.tau_J[5]);
        follower_state.setJoint7Torque(state_to_publish.tau_J[6]);
        follower_state.setJoint1ExtTorque(state_to_publish.tau_ext_hat_filtered[0]);
        follower_state.setJoint2ExtTorque(state_to_publish.tau_ext_hat_filtered[1]);
        follower_state.setJoint3ExtTorque(state_to_publish.tau_ext_hat_filtered[2]);
        follower_state.setJoint4ExtTorque(state_to_publish.tau_ext_hat_filtered[3]);
        follower_state.setJoint5ExtTorque(state_to_publish.tau_ext_hat_filtered[4]);
        follower_state.setJoint6ExtTorque(state_to_publish.tau_ext_hat_filtered[5]);
        follower_state.setJoint7ExtTorque(state_to_publish.tau_ext_hat_filtered[6]);
        follower_state.setControlRobot(static_cast<uint8_t>(control_rob.load()));

        kj::VectorOutputStream state_message;
        capnp::writeMessage(state_message, message);
        kj::ArrayPtr<const kj::byte> sz_state_message = state_message.getArray();

        sendto(sockPub, sz_state_message.begin(), sz_state_message.size(), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/pub_freq));
    }

    close(sockPub);

}




void subThread (const YAML::Node& config) {

    int port = config["leader"]["port"].as<int>();

    //socket params
    int sockSub = socket(AF_INET, SOCK_DGRAM, 0);
    char buffer[BUFFER_SIZE];
    struct sockaddr_in recv_addr{};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(port); // port for leader publisher
    recv_addr.sin_addr.s_addr = INADDR_ANY; // listen on all addresses

    const int sub_freq = config["global"]["freq"].as<int>(); //Hz

    if (bind(sockSub, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
        perror("Subscriber socket bind failed");
        close(sockSub);
    }

    std::cout << "Subscriber lsitening on port: " << port << std::endl;

    while(running.load()) {

        ssize_t n = recvfrom(sockSub, buffer, sizeof(buffer), 0, nullptr, nullptr);

        if (n < 0) {
            perror("Socket failed to recv info!");
            break;
        } else {
            sub_connected.store(true);
        }

        // unpack message
        std::vector<capnp::word> alignedBuffer((n + sizeof(capnp::word) - 1) / sizeof(capnp::word));
        memcpy(alignedBuffer.data(), buffer, n);
        kj::ArrayPtr<const capnp::word> receivedData(alignedBuffer.data(), alignedBuffer.size());
        capnp::FlatArrayMessageReader reader(receivedData);
        RobotState::Reader leader_state = reader.getRoot<RobotState>();

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            shared_leader_state = leader_state;
        }

        //set the current control robot
        control_rob.store(static_cast<char>(leader_state.getControlRobot()));

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/sub_freq));

    }

    close(sockSub);

}



void keyListener() {

    termios newT, oldT;
    tcgetattr(STDIN_FILENO, &oldT); //current terminal settings for backup

    newT = oldT;
    newT.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newT);

    // Make thread non blocking
    int oldFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags | O_NONBLOCK);

    std::cout << "Running ... Press Q to exit .." << std::endl;


    char key;
    while(running.load()) {
        key = getchar();
        if (key == 'q' || key =='Q') {
            running.store(false);
            std::cout << "Q Pressed ...." << std::endl;
        }
        usleep(10000); //delay
    }

    // restore terminal on exit
    tcsetattr(STDIN_FILENO, TCSANOW, &oldT);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags);

}




int main () {

    // scale for P and D values
    constexpr double scale = 1.3;


    try {

        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");

        // Define PGain and DGain and velo_limits
        std::vector<double> P_gain = config["follower"]["p_vals"].as<std::vector<double>>();
        std::vector<double> D_gain = config["follower"]["d_vals"].as<std::vector<double>>();
        std::vector<double> velo_limits_vec = config["global"]["velo_limits"].as<std::vector<double>>();
        std::array<double, 7> velo_limits;
        std::copy(velo_limits_vec.begin(), velo_limits_vec.end(), velo_limits.begin()); // convert to array.

        //constants for force feedback
        std::vector<double> C_q = config["global"]["C_q"].as<std::vector<double>>();
        std::vector<double> C_v = config["global"]["C_v"].as<std::vector<double>>();
        std::vector<double> C_y = config["global"]["C_y"].as<std::vector<double>>();
        std::vector<double> C_f = config["global"]["C_f"].as<std::vector<double>>();

        // contact switch sensitivity
        const double contact_threshold = config["global"]["contact_threshold"].as<double>();

        //connect to robot and initialize vals
        franka::Robot robot(config["follower"]["robot"].as<std::string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // move robot to start
        const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
        MotionGenerator motion_generator(0.5, home_pos);
        robot.control(motion_generator);

        // start sub thread
        std::thread sub_thread(subThread, std::cref(config));
        // start pub thread
        std::thread pub_thread(pubThread, std::cref(config));
        //key listener thread
        std::thread key_thread(keyListener);

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


        

        // lambda function to compute torques
        auto computeUnilateralTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {


            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load()) {

                return torques;
        
            };


            RobotState::Reader leader_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                leader_state = shared_leader_state;
            }

            std::array<double, 7> leader_pos = {
                leader_state.getJoint1Pos(),
                leader_state.getJoint2Pos(),
                leader_state.getJoint3Pos(),
                leader_state.getJoint4Pos(),
                leader_state.getJoint5Pos(),
                leader_state.getJoint6Pos(),
                leader_state.getJoint7Pos()
            };

            // // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, leader_pos, joint_pos);
            // std::array<double, 7> target_pos = leader_pos;

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = (scale * P_gain[i] * pos_error) - (scale * D_gain[i] * vel);
            };

            return torques;

        };

        
        auto computeBilateralTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {


            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load() || control_rob.load() == 'F') {

                return torques;
        
            };


            RobotState::Reader leader_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                leader_state = shared_leader_state;
            }

            std::array<double, 7> leader_pos = {
                leader_state.getJoint1Pos(),
                leader_state.getJoint2Pos(),
                leader_state.getJoint3Pos(),
                leader_state.getJoint4Pos(),
                leader_state.getJoint5Pos(),
                leader_state.getJoint6Pos(),
                leader_state.getJoint7Pos()
            };

            // // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, leader_pos, joint_pos);
            // std::array<double, 7> target_pos = leader_pos;

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = (scale * P_gain[i] * pos_error) - (scale * D_gain[i] * vel);
            };

            return torques;

        };

        
        auto computeBilateralTrqs2 = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {


            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load()) {

                return torques;
        
            };


            RobotState::Reader leader_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                leader_state = shared_leader_state;
            }

            std::array<double, 7> leader_pos = {
                leader_state.getJoint1Pos(),
                leader_state.getJoint2Pos(),
                leader_state.getJoint3Pos(),
                leader_state.getJoint4Pos(),
                leader_state.getJoint5Pos(),
                leader_state.getJoint6Pos(),
                leader_state.getJoint7Pos()
            };

            // // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, leader_pos, joint_pos);
            // std::array<double, 7> target_pos = leader_pos;

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = (scale * P_gain[i] * pos_error) - (scale * D_gain[i] * vel);
            };

            return torques;

        };



        auto computeBilateralWithForceFeedback = [&](const franka::RobotState& robot_state) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // initialize acclerations
            std::array<double, 7> acc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load()) {

                return torques;
        
            };

            RobotState::Reader leader_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                leader_state = shared_leader_state;
            }

            std::array<double, 7> leader_pos = {
                leader_state.getJoint1Pos(),
                leader_state.getJoint2Pos(),
                leader_state.getJoint3Pos(),
                leader_state.getJoint4Pos(),
                leader_state.getJoint5Pos(),
                leader_state.getJoint6Pos(),
                leader_state.getJoint7Pos()
            };

            std::array<double, 7> leader_vel = {
                leader_state.getJoint1Vel(),
                leader_state.getJoint2Vel(),
                leader_state.getJoint3Vel(),
                leader_state.getJoint4Vel(),
                leader_state.getJoint5Vel(),
                leader_state.getJoint6Vel(),
                leader_state.getJoint7Vel()
            };

            std::array<double, 7> leader_ext_trq = {
                leader_state.getJoint1ExtTorque(),
                leader_state.getJoint2ExtTorque(),
                leader_state.getJoint3ExtTorque(),
                leader_state.getJoint4ExtTorque(),
                leader_state.getJoint5ExtTorque(),
                leader_state.getJoint6ExtTorque(),
                leader_state.getJoint7ExtTorque()
            };

            
            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> ext_trq = robot_state.tau_ext_hat_filtered;

            // moment of inertia matrix
            std::array<double, 49> MOI = model.mass(robot_state);


            // Compute accelerations
            for (int i = 0; i < 7; ++i) {
                double pos_error = joint_pos[i] - leader_pos[i];
                double vel_error = joint_vel[i] - leader_vel[i];
                double vel_tot = joint_vel[i] + leader_vel[i];
                double ext_trq_tot = ext_trq[i] + leader_ext_trq[i];
                if (i == 5) {
                    acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                            - ((C_y[i] / 2) * (vel_tot)) - ((C_f[i] / (2 * 10)) * (ext_trq_tot));
                }
                
            }


            // Compute torques
            for (int i = 0; i < 7; i++) {
                if (i == 5) {
                    for (int j = 0; j < 7; j++) {
                        torques[i] += MOI[i*7 + j] * acc[j];
                    }
                } 
            }


            return torques;

        };





        // control callback function
        auto trq_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            
            if (!running.load()) {

                std::cout << "Exiting .... " << std::endl;
                
                return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
                
            }

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                shared_robot_state = robot_state;
            }

            // detect contact
            std::array<double, 7> ext_trq = robot_state.tau_ext_hat_filtered;
            bool anyOf = std::any_of(ext_trq.begin(), ext_trq.end(), [&contact_threshold](double x){ return std::abs(x) > contact_threshold;});
            if (anyOf) {
                control_rob.store('F');
            };

            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> command_torques = computeBilateralWithForceFeedback(robot_state);

            return command_torques;

        };




        while (running.load()) {

            try {

                //execute control loop
                robot.control(trq_control_callback);

            } catch (const franka::Exception& ex) {

                // print exception
                std::cout << ex.what() << std::endl;

                // auto recover
                robot.automaticErrorRecovery();

            }

        }

        // stop thread
        running.store(false);
        sub_thread.join();
        pub_thread.join();
        key_thread.join();


    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;

}