
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>

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
#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include "examples_common.h"


#include <franka/lowpass_filter.h>

using namespace std;


#define BUFFER_SIZE 2048

RobotState::Reader shared_follower_state;
franka::RobotState shared_robot_state;
double shared_gripper_width = 0.08;
std::mutex state_mutex;
std::atomic<bool> running{true};
std::atomic<bool> sub_connected{false}; // detects if subscriber connected
std::atomic<char> control_rob{'L'}; // default to leader(L)

std::atomic<char> mode{'T'}; // current mode of operation; T: Start, R: Record, O: Reset, E: End

// recording file variables and fns
std::ifstream lookup_file;
std::filesystem::path curr_rec_dir;
std::atomic<int> trial_num{0};
std::atomic<int> offset_deg{0};
std::atomic<int> object_offset{0};


void pubThread (const YAML::Node& config) {

    //config
    std::string ip = config["leader"]["ip"].as<std::string>();
    int port = config["leader"]["port"].as<int>();

    
     
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
        RobotState::Builder leader_state = message.initRoot<RobotState>();
        franka::RobotState state_to_publish;
        double gripperWidth;


        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state_to_publish = shared_robot_state;
            gripperWidth = shared_gripper_width;
        }
        
        leader_state.setTime(123456);
        leader_state.setJoint1Pos(state_to_publish.q[0]);
        leader_state.setJoint2Pos(state_to_publish.q[1]);
        leader_state.setJoint3Pos(state_to_publish.q[2]);
        leader_state.setJoint4Pos(state_to_publish.q[3]);
        leader_state.setJoint5Pos(state_to_publish.q[4]);
        leader_state.setJoint6Pos(state_to_publish.q[5]);
        leader_state.setJoint7Pos(state_to_publish.q[6]);
        leader_state.setJoint1Vel(state_to_publish.dq[0]);
        leader_state.setJoint2Vel(state_to_publish.dq[1]);
        leader_state.setJoint3Vel(state_to_publish.dq[2]);
        leader_state.setJoint4Vel(state_to_publish.dq[3]);
        leader_state.setJoint5Vel(state_to_publish.dq[4]);
        leader_state.setJoint6Vel(state_to_publish.dq[5]);
        leader_state.setJoint7Vel(state_to_publish.dq[6]);
        leader_state.setJoint1Torque(state_to_publish.tau_J[0]);
        leader_state.setJoint2Torque(state_to_publish.tau_J[1]);
        leader_state.setJoint3Torque(state_to_publish.tau_J[2]);
        leader_state.setJoint4Torque(state_to_publish.tau_J[3]);
        leader_state.setJoint5Torque(state_to_publish.tau_J[4]);
        leader_state.setJoint6Torque(state_to_publish.tau_J[5]);
        leader_state.setJoint7Torque(state_to_publish.tau_J[6]);
        leader_state.setJoint1ExtTorque(state_to_publish.tau_ext_hat_filtered[0]);
        leader_state.setJoint2ExtTorque(state_to_publish.tau_ext_hat_filtered[1]);
        leader_state.setJoint3ExtTorque(state_to_publish.tau_ext_hat_filtered[2]);
        leader_state.setJoint4ExtTorque(state_to_publish.tau_ext_hat_filtered[3]);
        leader_state.setJoint5ExtTorque(state_to_publish.tau_ext_hat_filtered[4]);
        leader_state.setJoint6ExtTorque(state_to_publish.tau_ext_hat_filtered[5]);
        leader_state.setJoint7ExtTorque(state_to_publish.tau_ext_hat_filtered[6]);
        leader_state.setEndEffPoseVal1(state_to_publish.O_T_EE[0]);
        leader_state.setEndEffPoseVal2(state_to_publish.O_T_EE[1]);
        leader_state.setEndEffPoseVal3(state_to_publish.O_T_EE[2]);
        leader_state.setEndEffPoseVal4(state_to_publish.O_T_EE[3]);
        leader_state.setEndEffPoseVal5(state_to_publish.O_T_EE[4]);
        leader_state.setEndEffPoseVal6(state_to_publish.O_T_EE[5]);
        leader_state.setEndEffPoseVal7(state_to_publish.O_T_EE[6]);
        leader_state.setEndEffPoseVal8(state_to_publish.O_T_EE[7]);
        leader_state.setEndEffPoseVal9(state_to_publish.O_T_EE[8]);
        leader_state.setEndEffPoseVal10(state_to_publish.O_T_EE[9]);
        leader_state.setEndEffPoseVal11(state_to_publish.O_T_EE[10]);
        leader_state.setEndEffPoseVal12(state_to_publish.O_T_EE[11]);
        leader_state.setEndEffPoseVal13(state_to_publish.O_T_EE[12]);
        leader_state.setEndEffPoseVal14(state_to_publish.O_T_EE[13]);
        leader_state.setEndEffPoseVal15(state_to_publish.O_T_EE[14]);
        leader_state.setEndEffPoseVal16(state_to_publish.O_T_EE[15]);
        leader_state.setGripperWidth(gripperWidth);
        leader_state.setFollowerEEOffset(offset_deg.load());
        leader_state.setRobotMode(static_cast<uint8_t>(mode.load()));
        leader_state.setControlRobot(static_cast<uint8_t>(control_rob.load()));
        

        kj::VectorOutputStream state_message;
        capnp::writeMessage(state_message, message);
        kj::ArrayPtr<const kj::byte> sz_state_message = state_message.getArray();

        sendto(sockPub, sz_state_message.begin(), sz_state_message.size(), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/pub_freq));
    }

    close(sockPub);

}



void subThread(const YAML::Node& config) {

    int port = config["follower"]["port"].as<int>();

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
        RobotState::Reader follower_state = reader.getRoot<RobotState>();

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            shared_follower_state = follower_state;
        }

        //set the current control robot
        control_rob.store(static_cast<char>(follower_state.getControlRobot()));

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
            std::cout << "Q pressed ... " << std::endl;
        }
        usleep(10000); //delay
    }

    // restore terminal on exit
    tcsetattr(STDIN_FILENO, TCSANOW, &oldT);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags);

}


void setNextIter() { 
    
    std::string line;
    if (!std::getline(lookup_file, line)) {
        std::cout << "End of experiment !!!!!" << std::endl;
        mode.store('E');
        std::this_thread::sleep_for(std::chrono::seconds(5));
        running.store(false);
    } else {
        std::stringstream ss(line);
        std::vector<std::string> values;
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            values.push_back(cell);
        }

        trial_num.store(std::stoi(values[0])); 
        offset_deg.store(std::stoi(values[2]));
        object_offset.store(std::stoi(values[3]));
        std::cout << "Rec updated to" << "_" << trial_num.load() << "_" << offset_deg.load() << "_" << object_offset.load() << std::endl;
    };
}


void setGripperWidth(const YAML::Node& config) {

    //connect to the gripper
    franka::Gripper gripper(config["leader"]["robot"].as<std::string>());

    double grip_threshold = config["gripper"]["grip_threshold"].as<double>();
    

    while (running.load()) {

        //read gripper state
        franka::GripperState gripperState = gripper.readOnce();
        double gripperWidth =  gripperState.width;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            shared_gripper_width = gripperWidth;
        }

        if ((mode.load() != 'O') && (gripperWidth < grip_threshold)) {
            mode.store('O');
            std::this_thread::sleep_for(std::chrono::seconds(5));
            gripper.move(0.08, config["gripper"]["speed"].as<double>());
        }
            

    }

}



void recorder(const YAML::Node& config, const std::array<double, 7>& home_pos) {

    int rec_freq = config["global"]["rec_freq"].as<int>();

    franka::RobotState leader_state;
    RobotState::Reader follower_state;
    

    while (running.load()) {

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            leader_state = shared_robot_state;
            follower_state = shared_follower_state;
        }

        if (mode.load() == 'R') {

            std::ofstream file(curr_rec_dir.string() + "/trial" + std::to_string(trial_num.load()) + "_" + std::to_string(offset_deg.load()) + "_" + std::to_string(object_offset.load()) + ".txt", std::ios::app);

            // std::cout << "Rec.........." << std::endl;

            file << follower_state.getTime() << "," 
            << follower_state.getJoint1Pos() << "," << follower_state.getJoint2Pos() << "," << follower_state.getJoint3Pos() << "," << follower_state.getJoint4Pos() << "," << follower_state.getJoint5Pos() << "," << follower_state.getJoint6Pos() << "," << follower_state.getJoint7Pos() << "," 
            << follower_state.getJoint1Torque() << "," << follower_state.getJoint2Torque() << "," << follower_state.getJoint3Torque() << "," << follower_state.getJoint4Torque() << "," << follower_state.getJoint5Torque() << "," << follower_state.getJoint6Torque() << "," << follower_state.getJoint7Torque() << "," 
            << follower_state.getJoint1Vel() << "," << follower_state.getJoint2Vel() << "," << follower_state.getJoint3Vel() << "," << follower_state.getJoint4Vel() << "," << follower_state.getJoint5Vel() << "," << follower_state.getJoint6Vel() << "," << follower_state.getJoint7Vel() << "," 
            << follower_state.getEndEffPoseVal1() << "," << follower_state.getEndEffPoseVal2() << "," << follower_state.getEndEffPoseVal3() << "," << follower_state.getEndEffPoseVal4() << "," << follower_state.getEndEffPoseVal5() << "," << follower_state.getEndEffPoseVal6() << "," << follower_state.getEndEffPoseVal7() << "," 
            << follower_state.getEndEffPoseVal8() << "," << follower_state.getEndEffPoseVal9() << "," << follower_state.getEndEffPoseVal10() << "," << follower_state.getEndEffPoseVal11() << "," << follower_state.getEndEffPoseVal12() << "," << follower_state.getEndEffPoseVal13() << "," << follower_state.getEndEffPoseVal14() << "," << follower_state.getEndEffPoseVal15() << "," << follower_state.getEndEffPoseVal16() << "," 
            << "\n";
            

            file.close();
        } else {
            // std::cout << "Not rec" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/rec_freq));

    }
    
}




int main (int argc, char* argv[]) {

    // scale for P and D values
    constexpr double scale = 1.3;

    if (argc < 2) {
        std::cout << "Usage: ./leader <../lookups/lookup_file_name.csv>\n";
        return -1;
    }

    
    try {


        // config
        YAML::Node config = YAML::LoadFile("../teleop_config.yml");

        //lookup table initialization
        lookup_file.open(argv[1]);
        std::string line;
        std::getline(lookup_file, line); //remove header

        //recordings file initialization
        std::filesystem::path path_to_lookup(argv[1]);
        std::string file_name = path_to_lookup.stem().string();
        curr_rec_dir = "../recordings/" + file_name;
        if (std::filesystem::create_directories(curr_rec_dir)) {
            std::filesystem::permissions(
                curr_rec_dir,
                std::filesystem::perms::owner_all |
                std::filesystem::perms::group_read | std::filesystem::perms::group_exec |
                std::filesystem::perms::others_read | std::filesystem::perms::others_exec,
                std::filesystem::perm_options::replace
            );
            std::cout << "Current participant directory created\n";
        } else {
            std::cout << "Current participant directory already exists. Checking for partially completed experiment....\n";
            for (const auto& entry : std::filesystem::directory_iterator(curr_rec_dir)) {
                //remove experiment iteration if already recorded
                std::getline(lookup_file, line);
            }
        }

        setNextIter(); //set initial rec params
        

        // Define PGain and DGain and velo_limits
        std::vector<double> P_gain = config["leader"]["p_vals"].as<std::vector<double>>();
        std::vector<double> D_gain = config["leader"]["d_vals"].as<std::vector<double>>();
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
        franka::Robot robot(config["leader"]["robot"].as<std::string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // move robot to start
        // const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
        const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.0};
        MotionGenerator motion_generator_home(0.5, home_pos);
        robot.control(motion_generator_home);

        // start publisher thread
        std::thread pub_thread(pubThread, std::cref(config));
        // start sub thread
        std::thread sub_thread(subThread, std::cref(config));
        // start gripper thread
        std::thread gripper_thread(setGripperWidth, std::cref(config));
        // // key listener thread
        // std::thread key_thread(keyListener);
        // recorder thread
        std::thread rec_thread(recorder, std::cref(config), std::cref(home_pos));

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        

        
   


        // lambda functions to compute torques
        auto computeUnilateralTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            return torques;

        };


        auto computeBilateralTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load() || control_rob.load() == 'L') {

                return torques;
        
            };

            RobotState::Reader follower_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                follower_state = shared_follower_state;
            }

            std::array<double, 7> follower_pos = {
                follower_state.getJoint1Pos(),
                follower_state.getJoint2Pos(),
                follower_state.getJoint3Pos(),
                follower_state.getJoint4Pos(),
                follower_state.getJoint5Pos(),
                follower_state.getJoint6Pos(),
                follower_state.getJoint7Pos()
            };

            // // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, follower_pos, joint_pos);
            // std::array<double, 7> target_pos = follower_pos;

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = (scale * P_gain[i] * pos_error) - (scale * D_gain[i] * vel);
            };

            return torques;

        };



        // control callback function
        auto trq_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

            char m = mode.load();

            if ((!running.load()) || (m == 'O')) {
                
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
                control_rob.store('L');
            }

            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;

            
            //start rec
            double threshold = 0.015;
            if (m != 'R') {
                for (size_t i=0; i < home_pos.size(); i++) {
                    if ((std::abs(home_pos[i] - joint_pos[i])) > threshold) {
                        mode.store('R'); 
                    }
                }
            }
            
            

            std::array<double, 7> command_torques = computeUnilateralTrqs(joint_pos, joint_vel);

            return command_torques;

        };




        while (running.load()) {

            try {
                char m = mode.load();
                if (m == 'T' || m == 'R') {
                    //execute control loop
                    robot.control(trq_control_callback);
                } else if (m == 'O') {
                    //reset to home
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    robot.control(motion_generator_home);
                    mode.store('T');
                    setNextIter(); //set the next recording params
                    
                    
                }


            } catch (const franka::Exception& ex) {

                // print exception
                std::cout << ex.what() << std::endl;

                // auto recover
                robot.automaticErrorRecovery();
            }

        }   

        // stop thread
        running.store(false);
        pub_thread.join();
        sub_thread.join();
        gripper_thread.join();
        // key_thread.join();
        rec_thread.join();

    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;
    
}