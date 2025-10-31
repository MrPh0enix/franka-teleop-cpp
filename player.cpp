#include <iostream>
#include <fstream>
#include <typeinfo>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <capnp/message.h>
#include <capnp/serialize.h>

#include "messages/robot-state.capnp.h"


franka::RobotState shared_robot_state;
std::mutex state_mutex;
std::atomic<bool> running{true};


void pubThread() {

    //socket params
    int sockPub = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(49185);
    inet_pton(AF_INET, "224.3.29.71", &addr.sin_addr);

    const int pub_freq = 100; //Hz

    // build a capnp message
    capnp::MallocMessageBuilder message{};
    RobotState::Builder state_builder = message.initRoot<RobotState>();

    uint64_t robot_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()
        ).count();


    while (running.load()) {
        franka::RobotState state_to_publish;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state_to_publish = shared_robot_state;
        }

        // build message
        state_builder.setTime(robot_time);
        state_builder.setJoint1Pos(state_to_publish.q[0]);
        state_builder.setJoint2Pos(state_to_publish.q[1]);
        state_builder.setJoint3Pos(state_to_publish.q[2]);
        state_builder.setJoint4Pos(state_to_publish.q[3]);
        state_builder.setJoint5Pos(state_to_publish.q[4]);
        state_builder.setJoint6Pos(state_to_publish.q[5]);
        state_builder.setJoint7Pos(state_to_publish.q[6]);
        state_builder.setJoint1Vel(state_to_publish.dq[0]);
        state_builder.setJoint2Vel(state_to_publish.dq[1]);
        state_builder.setJoint3Vel(state_to_publish.dq[2]);
        state_builder.setJoint4Vel(state_to_publish.dq[3]);
        state_builder.setJoint5Vel(state_to_publish.dq[4]);
        state_builder.setJoint6Vel(state_to_publish.dq[5]);
        state_builder.setJoint7Vel(state_to_publish.dq[6]);
        state_builder.setJoint1Torque(state_to_publish.tau_J[0]);
        state_builder.setJoint2Torque(state_to_publish.tau_J[1]);
        state_builder.setJoint3Torque(state_to_publish.tau_J[2]);
        state_builder.setJoint4Torque(state_to_publish.tau_J[3]);
        state_builder.setJoint5Torque(state_to_publish.tau_J[4]);
        state_builder.setJoint6Torque(state_to_publish.tau_J[5]);
        state_builder.setJoint7Torque(state_to_publish.tau_J[6]);
        state_builder.setJoint1ExtTorque(state_to_publish.tau_ext_hat_filtered[0]);
        state_builder.setJoint2ExtTorque(state_to_publish.tau_ext_hat_filtered[1]);
        state_builder.setJoint3ExtTorque(state_to_publish.tau_ext_hat_filtered[2]);
        state_builder.setJoint4ExtTorque(state_to_publish.tau_ext_hat_filtered[3]);
        state_builder.setJoint5ExtTorque(state_to_publish.tau_ext_hat_filtered[4]);
        state_builder.setJoint6ExtTorque(state_to_publish.tau_ext_hat_filtered[5]);
        state_builder.setJoint7ExtTorque(state_to_publish.tau_ext_hat_filtered[6]);

        // Flatten to a single byte array
        auto flatArray = capnp::messageToFlatArray(message);
        auto bytes = flatArray.asBytes();

        sendto(sockPub, bytes.begin(), bytes.size(), 0,
                (sockaddr*)&addr, sizeof(addr));

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/pub_freq));
    }

    close(sockPub);
    
}




void subThread() {

    const int sub_freq = 100; //Hz

    // message size.
    char buffer[2048];

    //socket params
    int sockSub = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(49186);
    addr.sin_addr.s_addr = INADDR_ANY; //listen on all addresses
    
    if (bind(sockSub, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Socket bind failed");
        close(sockSub);
    }

    std::cout << "Subscriber listening on port: 49186" << std::endl;

    while (running.load()) {

        sockaddr_in senderAddr{};
        socklen_t senderLen = sizeof(senderAddr);

        ssize_t bytes = recvfrom(sockSub, buffer, sizeof(buffer), 0,
                                (struct sockaddr*)&senderAddr, &senderLen);

        std::cout << "Received : " << std::endl;

        // capn proto decode
        kj::ArrayPtr<capnp::word> words(
            reinterpret_cast<capnp::word*>(buffer),
            bytes / sizeof(capnp::word)
        );

        capnp::FlatArrayMessageReader reader(words);
        RobotState::Reader state = reader.getRoot<RobotState>();

        std::cout << "Timestamp: " << state.getTime() <<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/sub_freq));
    }

    close(sockSub);

}




int main(int argc, char** argv) {


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

    // Homing point (static point the robot tries to reach)
    const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};

    // velocity limits
    const std::array<double, 7> velo_limits = {8.0, 7.0, 7.0, 7.0, 6.0, 6.0, 4.0};


    try {

        // connect to robot
        franka::Robot robot("192.168.170.2");

        //initialize shared robot state
        shared_robot_state = robot.readOnce();

        //start publisher and subscriber thread
        std::thread pub_thread(pubThread);
        std::thread sub_thread(subThread);

        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        


        auto computeTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {};

            // limit velocity of joints
            std::array<double, 7> target_pos = franka::limitRate(velo_limits, home_pos, joint_pos);

            // Compute torques
            for (int i = 0; i < 7; ++i) {
                double pos_error = target_pos[i] - joint_pos[i];
                double vel = joint_vel[i];
                torques[i] = P_gain[i] * pos_error - D_gain[i] * vel;
            };

            return torques;

        };

        // trq callback functions
        double time = 0.0;
        int step = 0;
        bool contact_flag = false;
        double last_contact_time = 0.0;

        
        auto trq_control_callback = [&step, &time, &contact_flag, &last_contact_time, &computeTrqs, &contact_threshold]
                                    (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {

            {
                std::lock_guard<std::mutex> lock(state_mutex);
                shared_robot_state = robot_state;
            }

            // time passed since the loop started
            time += period.toSec();

            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> ext_trq = robot_state.tau_ext_hat_filtered;


            // send command torques after checking ext trq.
            bool anyOf = std::any_of(ext_trq.begin(), ext_trq.end(), [&contact_threshold](double x){ return std::abs(x) > contact_threshold;});
            if (anyOf) {

                contact_flag = true;
                last_contact_time = time;
                return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

            } else if (contact_flag){

                // check if 2 sec passed since stopped contact.
                if (time - last_contact_time > 2.0) {
                    
                    contact_flag = false;
                    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                } else {

                    return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                };

            } else {
                
                std::array<double, 7> command_torques = computeTrqs(joint_pos, joint_vel);
                return command_torques;

            };

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
        
        
        running.store(false);
        pub_thread.join();
        sub_thread.join();


    } catch (const franka::Exception& ex) {

        // print exception
        std::cout << ex.what() << std::endl;
        
        running.store(false);

        return -1;
    
    }


    return 0;

}