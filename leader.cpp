
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <termios.h>
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


#define PORT 49185
#define BUFFER_SIZE 2048


franka::RobotState shared_robot_state;
std::mutex state_mutex;
std::atomic<bool> running{true};


void pubThread (const YAML::Node& config) {

    //config
    std::string ip = config["leader"]["ip"].as<std::string>();
    int port = config["leader"]["port"].as<int>();
     
    // socket params
    int sockPub = socket(AF_INET, SOCK_DGRAM, 0);
    char buffer[BUFFER_SIZE];
    struct sockaddr_in send_addr{};
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(PORT);
    inet_pton(AF_INET, ip.c_str(), &send_addr.sin_addr);

    const int pub_freq = config["global"]["freq"].as<int>(); //Hz 


    while (running.load()) {

        //build message
        capnp::MallocMessageBuilder message;
        RobotState::Builder leader_state = message.initRoot<RobotState>();
        franka::RobotState state_to_publish;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state_to_publish = shared_robot_state;
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

        kj::VectorOutputStream state_message;
        capnp::writeMessage(state_message, message);
        kj::ArrayPtr<const kj::byte> sz_state_message = state_message.getArray();

        sendto(sockPub, sz_state_message.begin(), sz_state_message.size(), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/pub_freq));
    }

    close(sockPub);

}

void subThread() {

}


int main () {
    
    try {

        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");

        //connect to robot
        franka::Robot robot(config["leader"]["robot"].as<std::string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // start publisher thread
        std::thread pub_thread(pubThread, std::cref(config));

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


        // lambda function to compute torques
        auto computeTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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
        pub_thread.join();

    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;
    
}