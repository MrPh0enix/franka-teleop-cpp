
#include <iostream>
#include <cstring>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>

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


#include <VelocityObserver.h>

using namespace std;


#define BUFFER_SIZE 2048

RobotState::Reader shared_follower_state;
franka::RobotState shared_robot_state;
double shared_gripper_width = 0.08;
std::mutex state_mutex;
std::atomic<bool> running{true};
std::atomic<bool> sub_connected{false}; // detects if subscriber connected
std::atomic<char> control_rob{'L'}; // default to leader(L)


// velocity estimators for each joint
std::array<VelocityObserver, 7> leader_vel_estimators = {
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER)
};
std::array<VelocityObserver, 7> follower_vel_estimators = {
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER),
    VelocityObserver(200, 0.001, VelocityObserver::Method::EULER)
};


// to store previous time step torques for DOB
std::array<double, 7> tau_in_prev;





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
        leader_state.setJoint1ExtTorque(state_to_publish.tau_J_d[0]);
        leader_state.setJoint2ExtTorque(state_to_publish.tau_J_d[1]);
        leader_state.setJoint3ExtTorque(state_to_publish.tau_J_d[2]);
        leader_state.setJoint4ExtTorque(state_to_publish.tau_J_d[3]);
        leader_state.setJoint5ExtTorque(state_to_publish.tau_J_d[4]);
        leader_state.setJoint6ExtTorque(state_to_publish.tau_J_d[5]);
        leader_state.setJoint7ExtTorque(state_to_publish.tau_J_d[6]);
        leader_state.setGripperWidth(gripperWidth);
        leader_state.setControlRobot(static_cast<uint8_t>(control_rob.load()));
        leader_state.setJoint7MeasuredTorqueDer(state_to_publish.dtau_J[6]);
        

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



void setGripperWidth(const YAML::Node& config) {

    //connect to the gripper
    franka::Gripper gripper(config["leader"]["robot"].as<std::string>());

    while (running.load()) {

        //read gripper state
        franka::GripperState gripperState = gripper.readOnce();
        double gripperWidth =  gripperState.width;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            shared_gripper_width = gripperWidth;
        }

    }

}




int main () {

    // scale for P and D values
    constexpr double scale = 1.3;

    
    try {


        // config
        YAML::Node config = YAML::LoadFile("teleop_config.yml");

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
        std::vector<double> vel_coeff = config["global"]["vel_coeff"].as<std::vector<double>>();
        
        // contact switch sensitivity
        const double contact_threshold = config["global"]["contact_threshold"].as<double>();

        //connect to robot and initialize vals
        franka::Robot robot(config["leader"]["robot"].as<std::string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // move robot to start
        const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
        MotionGenerator motion_generator(0.5, home_pos);
        robot.control(motion_generator);

        // start publisher thread
        std::thread pub_thread(pubThread, std::cref(config));
        // start sub thread
        std::thread sub_thread(subThread, std::cref(config));
        // start gripper thread
        std::thread gripper_thread(setGripperWidth, std::cref(config));
        // key listener thread
        std::thread key_thread(keyListener);

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        

        
        std::ofstream file("output_leader.txt", std::ios::app);
        if (!file.is_open()) {
            std::cerr << "Failed to open file\n";
            return 1;
        }

   


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


        auto computeBilateralTrqs2 = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load()) {

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


        auto computeBilateralWithForceFeedback = [&](const franka::RobotState& robot_state) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            // initialize acclerations
            std::array<double, 7> acc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            if (!sub_connected.load()) {

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

            std::array<double, 7> follower_vel = {
                follower_state.getJoint1Vel(),
                follower_state.getJoint2Vel(),
                follower_state.getJoint3Vel(),
                follower_state.getJoint4Vel(),
                follower_state.getJoint5Vel(),
                follower_state.getJoint6Vel(),
                follower_state.getJoint7Vel()
            };

            std::array<double, 7> follower_ext_trq = {
                follower_state.getJoint1ExtTorque(),
                follower_state.getJoint2ExtTorque(),
                follower_state.getJoint3ExtTorque(),
                follower_state.getJoint4ExtTorque(),
                follower_state.getJoint5ExtTorque(),
                follower_state.getJoint6ExtTorque(),
                follower_state.getJoint7ExtTorque()
            };

            std::array<double, 7> follower_trq = {
                follower_state.getJoint1Torque(),
                follower_state.getJoint2Torque(),
                follower_state.getJoint3Torque(),
                follower_state.getJoint4Torque(),
                follower_state.getJoint5Torque(),
                follower_state.getJoint6Torque(),
                follower_state.getJoint7Torque()
            };

            double follower6_trq_der = follower_state.getJoint7MeasuredTorqueDer();


            
            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> ext_trq = robot_state.tau_J_d;

            std::array<double, 7> trq_grav = robot_state.tau_J_d;
            std::array<double, 7> trq_der = robot_state.dtau_J;


            //write to file
            file << joint_pos[6] << "," << joint_vel[6] << "," << trq_grav[6] << "," << trq_der[6] << "," << follower_pos[6] << "," << follower_vel[6] << "," << follower6_trq_der << "," << follower6_trq_der << "\n";


            // moment of inertia matrix
            std::array<double, 49> MOI = model.mass(robot_state);

            //coriolis
            std::array<double, 7> coriolis = model.coriolis(robot_state);


            // Compute accelerationsext_trq
            for (int i = 0; i < 7; ++i) {
                double pos_error = joint_pos[i] - follower_pos[i];
                double vel_error = joint_vel[i] - follower_vel[i];
                double vel_tot = joint_vel[i] + follower_vel[i];
                double ext_trq_tot = ext_trq[i] + follower_ext_trq[i];
                // if ((i == 0) || (i == 1) || (i == 2))  {
                //     acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                //            - ((C_y[i] / 2) * (vel_tot)) - ((C_f[i] / (2 * 1)) * (ext_trq_tot));
                // }
                acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                            - ((C_y[i] / 2) * (vel_tot)) - ((C_f[i] / (2 * 1)) * (ext_trq_tot));
                
            }
            
        
            // Compute torques
            for (int i = 0; i < 7; i++) {
                // if ((i == 0) || (i == 1) || (i == 2))   {
                //     for (int j = 0; j < 7; j++) {
                //         torques[i] += MOI[i*7 + j] * acc[j] ;
                //     }
                //     torques[i] -= (vel_coeff[i] * joint_vel[i]);
                // }
                for (int j = 0; j < 7; j++) {
                    torques[i] += MOI[i*7 + j] * acc[j] ;
                }
                torques[i] -= (vel_coeff[i] * joint_vel[i]);
            }


            return torques;

        };


        



        auto computeBilateralWithDOB = [&](const franka::RobotState& robot_state) {

            // initialize torques and acclerations
            std::array<double, 7> torques;
            torques.fill(0.0);
            std::array<double, 7> acc;
            acc.fill(0.0);

            if (!sub_connected.load()) {

                return torques;
        
            };

            // get follower states
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

            std::array<double, 7> follower_vel = {
                follower_state.getJoint1Vel(),
                follower_state.getJoint2Vel(),
                follower_state.getJoint3Vel(),
                follower_state.getJoint4Vel(),
                follower_state.getJoint5Vel(),
                follower_state.getJoint6Vel(),
                follower_state.getJoint7Vel()
            };

            std::array<double, 7> follower_ext_trq = {
                follower_state.getJoint1ExtTorque(),
                follower_state.getJoint2ExtTorque(),
                follower_state.getJoint3ExtTorque(),
                follower_state.getJoint4ExtTorque(),
                follower_state.getJoint5ExtTorque(),
                follower_state.getJoint6ExtTorque(),
                follower_state.getJoint7ExtTorque()
            };

            std::array<double, 7> follower_trq = {
                follower_state.getJoint1Torque(),
                follower_state.getJoint2Torque(),
                follower_state.getJoint3Torque(),
                follower_state.getJoint4Torque(),
                follower_state.getJoint5Torque(),
                follower_state.getJoint6Torque(),
                follower_state.getJoint7Torque()
            };



            // get leader states
            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;
            std::array<double, 7> ext_trq = robot_state.tau_ext_hat_filtered;


            // moment of inertia matrix
            std::array<double, 49> MOI = model.mass(robot_state);

            // nominal inertia for DOB
            std::array<double, 7> a_n;
            for (int i = 0; i < 7; i++) {
                a_n[i] = MOI[i*7 + i];
            }


            // velocity estimation (using vel observer)
            std::array<double, 7> leader_vel_est;
            std::array<double, 7> follower_vel_est;

            for (int i = 0; i < 7; i++) {
                leader_vel_est[i] = joint_vel[i]; // franka already provides filtered vel
                // estimate follower vel from position as its state is sent through a connection
                follower_vel_est[i] = follower_vel_estimators[i].update(follower_pos[i]); 
            }


            // Compute desired accelerations
            for (int i = 0; i < 7; ++i) {
                double pos_error = joint_pos[i] - follower_pos[i];
                double vel_error = leader_vel_est[i] - follower_vel_est[i];
                double vel_tot = leader_vel_est[i] + follower_vel_est[i];
                double ext_trq_tot = ext_trq[i] + follower_ext_trq[i];
                // if ((i == 0) || (i == 1) || (i == 2))  {
                //     acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                //            - ((C_y[i] / 2) * (vel_tot)) - ((C_f[i] / (2 * 1)) * (ext_trq_tot));
                // }
                acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                            - ((C_y[i] / 2) * (vel_tot)) - ((C_f[i] / (2 * 1)) * (ext_trq_tot));
                
            }
            // Compute torques
            for (int i = 0; i < 7; i++) {
                for (int j = 0; j < 7; j++) {
                    torques[i] += MOI[i*7 + j] * acc[j] ;
                }
            }


            // persistent variable for DOB
            static std::array<double, 7> lpf_output_prev;
            static std::array<double, 7> lpf_input_prev;
            static std::array<double, 7> tau_in_prev;

            // DOB params
            constexpr double T = 0.001;
            constexpr double g_dob = 1000.0;


            // Disturbance Observer
            for (int i = 0; i < 7; i++) {

                double omega = joint_vel[i];
                double lpf_input = tau_in_prev[i] + a_n[i] * g_dob * omega;
                
                // Al-Alaoui low pass filter
                double lpf_output = (1.0 / (7.0*g_dob*T + 8.0)) * ((8.0 - g_dob*T)*lpf_output_prev[i] + 7.0*g_dob*T*lpf_input + g_dob*T*lpf_input_prev[i]);

                double tau_dis_hat = lpf_output - a_n[i]*g_dob*omega;

                torques[i] += tau_dis_hat;

                lpf_output_prev[i] = lpf_output;
                lpf_input_prev[i]  = lpf_input;

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
                control_rob.store('L');
            }
            


            std::array<double, 7> joint_pos = robot_state.q;
            std::array<double, 7> joint_vel = robot_state.dq;

            


            // std::array<double, 7> command_torques = computeBilateralWithForceFeedback(robot_state);
            std::array<double, 7> command_torques = computeUnilateralTrqs(joint_pos, joint_vel);
            // std::array<double, 7> command_torques = computeBilateralWithDOB(robot_state);

            std::array<double, 7> tau_cmd_rate_limited = franka::limitRate(franka::kMaxTorqueRate, command_torques, robot_state.tau_J_d);

            tau_in_prev = tau_cmd_rate_limited;

            return tau_cmd_rate_limited;

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
        sub_thread.join();
        key_thread.join();

        file.close();

    } catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    }

    return 0;
    
}