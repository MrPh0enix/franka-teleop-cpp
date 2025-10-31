
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

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

#define PORT 49186
#define BUFFER_SIZE 2048


RobotState::Reader shared_leader_state;
franka::RobotState shared_robot_state;
std::mutex state_mutex;
std::atomic<bool> running{true};



void pubThread () {

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

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/sub_freq));

    }

    close(sockSub);

}



int main () {

    // scale for P and D values
    constexpr double scale = 1.3;

    // contact switch sensitivity
    const double contact_threshold = 2.5;

    // Define PGain and DGain
    std::array<double, 7> P_gain = {
        scale * 800.0,
        scale * 800.0,
        scale * 600.0,
        scale * 800.0,
        scale * 150.0,
        scale * 150.0,
        scale * 50.0
    };

    std::array<double, 7> D_gain = {
        scale * 50.0,
        scale * 50.0,
        scale * 50.0,
        scale * 50.0,
        scale * 30.0,
        scale * 25.0,
        scale * 10.0
    };

    // velocity limits
    const std::array<double, 7> velo_limits = {8.0, 7.0, 7.0, 7.0, 6.0, 6.0, 4.0};



    try {

        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");

        //connect to robot
        franka::Robot robot(config["follower"]["robot"].as<std::string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // start sub thread
        std::thread sub_thread(subThread, std::cref(config));

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        

        // lambda function to compute torques
        auto computeTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {};

            RobotState::Reader leader_state;

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                leader_state = shared_leader_state;
            }

            const std::array<double, 7>  leader_pos = {
                leader_state.getJoint1Pos(),
                leader_state.getJoint2Pos(),
                leader_state.getJoint3Pos(),
                leader_state.getJoint4Pos(),
                leader_state.getJoint5Pos(),
                leader_state.getJoint6Pos(),
                leader_state.getJoint7Pos()
            };

            // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, leader_pos, joint_pos);

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = P_gain[i] * pos_error - D_gain[i] * vel;
            };

            return torques;

        };


        // control callback function
        auto trq_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                shared_robot_state = robot_state;
            }

            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> command_torques = computeTrqs(joint_pos, joint_vel);

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


    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;

}