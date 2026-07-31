
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


using namespace std;


#define BUFFER_SIZE 2048



struct FollowerData {
    array<double, 7> pos{};
    array<double, 7> vel{};
    array<double, 7> ext_trq{};
    array<double, 7> trq{};
    array<double, 7> trq_der{};

    double time = 0.0;

    double gripper_width = 0.0;

    double joint7_torque_der = 0.0;
};


FollowerData shared_follower_data;
franka::RobotState shared_robot_state;
double shared_gripper_width = 0.08;
mutex state_mutex;
atomic<bool> running{true};
atomic<bool> sub_connected{false}; // detects if subscriber connected


void pubThread (const YAML::Node& config) {

    //config
    string ip = config["leader"]["ip"].as<string>();
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
            lock_guard<mutex> lock(state_mutex);
            state_to_publish = shared_robot_state;
            gripperWidth = shared_gripper_width;
        }

        leader_state.setTime(123456);
        leader_state.setJoint1Pos(state_to_publish.theta[0]);
        leader_state.setJoint2Pos(state_to_publish.theta[1]);
        leader_state.setJoint3Pos(state_to_publish.theta[2]);
        leader_state.setJoint4Pos(state_to_publish.theta[3]);
        leader_state.setJoint5Pos(state_to_publish.theta[4]);
        leader_state.setJoint6Pos(state_to_publish.theta[5]);
        leader_state.setJoint7Pos(state_to_publish.theta[6]);
        leader_state.setJoint1Vel(state_to_publish.dtheta[0]);
        leader_state.setJoint2Vel(state_to_publish.dtheta[1]);
        leader_state.setJoint3Vel(state_to_publish.dtheta[2]);
        leader_state.setJoint4Vel(state_to_publish.dtheta[3]);
        leader_state.setJoint5Vel(state_to_publish.dtheta[4]);
        leader_state.setJoint6Vel(state_to_publish.dtheta[5]);
        leader_state.setJoint7Vel(state_to_publish.dtheta[6]);
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
        leader_state.setGripperWidth(gripperWidth);
        leader_state.setJoint1ExtTorqueDer(state_to_publish.dtau_J[0]);
        leader_state.setJoint2ExtTorqueDer(state_to_publish.dtau_J[1]);
        leader_state.setJoint3ExtTorqueDer(state_to_publish.dtau_J[2]);
        leader_state.setJoint4ExtTorqueDer(state_to_publish.dtau_J[3]);
        leader_state.setJoint5ExtTorqueDer(state_to_publish.dtau_J[4]);
        leader_state.setJoint6ExtTorqueDer(state_to_publish.dtau_J[5]);
        leader_state.setJoint7ExtTorqueDer(state_to_publish.dtau_J[6]);
        

        kj::VectorOutputStream state_message;
        capnp::writeMessage(state_message, message);
        kj::ArrayPtr<const kj::byte> sz_state_message = state_message.getArray();

        sendto(sockPub, sz_state_message.begin(), sz_state_message.size(), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));

        // sleep
        this_thread::sleep_for(chrono::milliseconds(1000/pub_freq));
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

    cout << "Subscriber lsitening on port: " << port << endl;

    while(running.load()) {

        ssize_t n = recvfrom(sockSub, buffer, sizeof(buffer), 0, nullptr, nullptr);

        if (n < 0) {
            perror("Socket failed to recv info!");
            break;
        } else {
            sub_connected.store(true);
        }

        // unpack message
        vector<capnp::word> alignedBuffer((n + sizeof(capnp::word) - 1) / sizeof(capnp::word));
        memcpy(alignedBuffer.data(), buffer, n);
        kj::ArrayPtr<const capnp::word> receivedData(alignedBuffer.data(), alignedBuffer.size());
        capnp::FlatArrayMessageReader reader(receivedData);
        RobotState::Reader follower_state = reader.getRoot<RobotState>();

        FollowerData data;

        data.time = follower_state.getTime();

        data.pos = {
            follower_state.getJoint1Pos(),
            follower_state.getJoint2Pos(),
            follower_state.getJoint3Pos(),
            follower_state.getJoint4Pos(),
            follower_state.getJoint5Pos(),
            follower_state.getJoint6Pos(),
            follower_state.getJoint7Pos()
        };

        data.vel = {
            follower_state.getJoint1Vel(),
            follower_state.getJoint2Vel(),
            follower_state.getJoint3Vel(),
            follower_state.getJoint4Vel(),
            follower_state.getJoint5Vel(),
            follower_state.getJoint6Vel(),
            follower_state.getJoint7Vel()
        };

        data.trq = {
            follower_state.getJoint1Torque(),
            follower_state.getJoint2Torque(),
            follower_state.getJoint3Torque(),
            follower_state.getJoint4Torque(),
            follower_state.getJoint5Torque(),
            follower_state.getJoint6Torque(),
            follower_state.getJoint7Torque()
        };

        data.ext_trq = {
            follower_state.getJoint1ExtTorque(),
            follower_state.getJoint2ExtTorque(),
            follower_state.getJoint3ExtTorque(),
            follower_state.getJoint4ExtTorque(),
            follower_state.getJoint5ExtTorque(),
            follower_state.getJoint6ExtTorque(),
            follower_state.getJoint7ExtTorque()
        };

        data.trq_der = {
            follower_state.getJoint1ExtTorqueDer(),
            follower_state.getJoint2ExtTorqueDer(),
            follower_state.getJoint3ExtTorqueDer(),
            follower_state.getJoint4ExtTorqueDer(),
            follower_state.getJoint5ExtTorqueDer(),
            follower_state.getJoint6ExtTorqueDer(),
            follower_state.getJoint7ExtTorqueDer()
        };

        data.gripper_width = follower_state.getGripperWidth();
        

        {
            lock_guard<mutex> lock(state_mutex);
            shared_follower_data = data;
        }

        this_thread::sleep_for(chrono::milliseconds(1000/sub_freq));

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

    cout << "Running ... Press Q to exit .." << endl;


    char key;
    while(running.load()) {
        key = getchar();
        if (key == 'q' || key =='Q') {
            running.store(false);
            cout << "Q pressed ... " << endl;
        }
        usleep(10000); //delay
    }

    // restore terminal on exit
    tcsetattr(STDIN_FILENO, TCSANOW, &oldT);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags);

}



void setGripperWidth(const YAML::Node& config) {

    //connect to the gripper
    franka::Gripper gripper(config["leader"]["robot"].as<string>());

    while (running.load()) {

        //read gripper state
        franka::GripperState gripperState = gripper.readOnce();
        double gripperWidth =  gripperState.width;

        {
            lock_guard<mutex> lock(state_mutex);
            shared_gripper_width = gripperWidth;
        }

    }

}




int main () {

    // scale for P and D values
    constexpr double scale = 1.3;

    
    try {


        // config
        YAML::Node config = YAML::LoadFile("../teleop_config.yml");

        // Define PGain and DGain and velo_limits
        vector<double> P_gain = config["leader"]["p_vals"].as<vector<double>>();
        vector<double> D_gain = config["leader"]["d_vals"].as<vector<double>>();
        vector<double> velo_limits_vec = config["global"]["velo_limits"].as<vector<double>>();
        array<double, 7> velo_limits;
        copy(velo_limits_vec.begin(), velo_limits_vec.end(), velo_limits.begin()); // convert to array.

        //constants for force feedback
        vector<double> C_q = config["global"]["C_q"].as<vector<double>>();
        vector<double> C_v = config["global"]["C_v"].as<vector<double>>();
        vector<double> C_y = config["global"]["C_y"].as<vector<double>>();
        vector<double> C_f = config["global"]["C_f"].as<vector<double>>();
        vector<double> vel_coeff = config["global"]["vel_coeff"].as<vector<double>>();

        // disturbance observer constants
        double g_dob = config["disturbance_observer"]["g"].as<double>();
        double T_dob = config["disturbance_observer"]["T"].as<double>();

        
        // contact switch sensitivity
        const double contact_threshold = config["global"]["contact_threshold"].as<double>();

        //connect to robot and initialize vals
        franka::Robot robot(config["leader"]["robot"].as<string>());
        shared_robot_state = robot.readOnce();
        franka::Model model = robot.loadModel();

        // move robot to start
        const array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0 + 1.57, 1.57079633, 0.78539816}; 
        
        MotionGenerator motion_generator(0.5, home_pos);
        robot.control(motion_generator);

        // start publisher thread
        thread pub_thread(pubThread, cref(config));
        // start sub thread
        thread sub_thread(subThread, cref(config));
        // start gripper thread
        thread gripper_thread(setGripperWidth, cref(config));
        // key listener thread
        thread key_thread(keyListener);

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
        

        
        // ofstream file("output_leader.txt", ios::app);
        // if (!file.is_open()) {
        //     cerr << "Failed to open file\n";
        //     return 1;
        // }

   


        // lambda functions to compute torques
        auto computeUnilateralTrqs = [&](const franka::RobotState& robot_state) {

            // initialize trqs
            array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            return torques;

        };


        auto computeBilateralWithDOB = [&](const franka::RobotState& robot_state) {

            // initialize torques and acclerations
            array<double, 7> torques;
            torques.fill(0.0);
            array<double, 7> acc;
            acc.fill(0.0);

            if (!sub_connected.load()) {

                return torques;
        
            };

            FollowerData follower_data;

            {
                lock_guard<mutex> lock(state_mutex);
                follower_data = shared_follower_data;
            }

            array<double, 7> follower_pos = follower_data.pos;
            array<double, 7> follower_vel = follower_data.vel;
            array<double, 7> follower_ext_trq = follower_data.ext_trq;
            array<double, 7> follower_trq = follower_data.trq;
            array<double, 7> follower_trq_der = follower_data.trq_der;



            // get leader states
            array<double, 7> joint_pos = robot_state.theta;
            array<double, 7> joint_vel = robot_state.dtheta;
            array<double, 7> ext_trq = robot_state.tau_ext_hat_filtered;
            array<double, 7> jnt_trq = robot_state.tau_J;
            array<double, 7> jnt_trq_der = robot_state.dtau_J;


            // moment of inertia matrix
            array<double, 49> MOI = model.mass(robot_state);

            //coriolis
            array<double, 7> coriolis = model.coriolis(robot_state);


            vector<int> active_joints = {0,1,2,3,4,5,6};

            

            // nominal inertia for DOB
            array<double, 7> a_n;
            a_n.fill(0.0);
            for (int i = 0; i < 7; ++i) {
                a_n[i] = MOI[i*7 + i];
            }


            // velocity estimation (using vel observer)
            array<double, 7> leader_vel_est;
            leader_vel_est.fill(0.0);
            array<double, 7> follower_vel_est;
            follower_vel_est.fill(0.0);
            for (int i = 0; i < 7; ++i) {
                leader_vel_est[i] = joint_vel[i];
                follower_vel_est[i] = follower_vel[i];
            }


            // Compute desired accelerations
            for (int i: active_joints) {
                double pos_error = joint_pos[i] - follower_pos[i];
                double vel_error = leader_vel_est[i] - follower_vel_est[i];
                // double vel_tot = leader_vel_est[i] + follower_vel_est[i];
                double vel_tot = jnt_trq_der[i] + follower_trq_der[i];
                double ext_trq_tot = ext_trq[i] + follower_ext_trq[i];
                acc[i] = - ((C_q[i] / 2) * (pos_error)) - ((C_v[i] / 2) * (vel_error)) 
                            - ((C_y[i] / 2) * (vel_tot))  - ((C_f[i] / (2 * 1)) * (ext_trq_tot));
            }


            // // Compute torques
            // for (int i : active_joints) {
            //     for (int j = 0; j < 7; j++) {
            //         torques[i] += MOI[i*7 + j] * acc[j] ;
            //     }
            // }


            // Compute torques
            for (int i : active_joints) {
                torques[i] = a_n[i] * acc[i];
            }
            

            // // negating effects of gravity compensation
            // array<double, 7> gravity = model.gravity(robot_state);
            // for (int i : active_joints) {
            //     torques[i] -= gravity[i] ;
            // }



            // // Disturbance Observer

            // static array<double, 7> lpf_output_prev;
            // static array<double, 7> lpf_input_prev;

            
            // for (int i : active_joints) {

            //     double omega = joint_vel[i];
            //     double lpf_input = torques[i] + a_n[i] * g_dob * omega;
                
            //     // // Al-Alaoui low pass filter
            //     // double lpf_output = (1.0 / (7.0*g_dob*T_dob + 8.0)) * ((8.0 - g_dob*T_dob)*lpf_output_prev[i] + 7.0*g_dob*T_dob*lpf_input + g_dob*T_dob*lpf_input_prev[i]);
            //     // backward Euler
            //     double lpf_output = (1.0 / (g_dob*T_dob + 1.0)) * (lpf_output_prev[i] + g_dob*T_dob*lpf_input);

            //     double tau_dis_hat = lpf_output - a_n[i]*g_dob*omega;

            //     torques[i] += tau_dis_hat;

            //     lpf_output_prev[i] = lpf_output;
            //     lpf_input_prev[i]  = lpf_input;

            // }

            
            // array<double, 7> leader_pos = robot_state.theta;
            // array<double, 7> leader_trq = robot_state.tau_J;
            // array<double, 7> leader_vel = robot_state.dtheta;
            // array<double, 7> leader_ext_trq = robot_state.tau_ext_hat_filtered;
            // // write to file
            // file << leader_pos[0] << "," << leader_pos[1] << "," << leader_pos[2] << "," << leader_pos[3] << "," << leader_pos[4] << "," << leader_pos[5] << "," << leader_pos[6] << ","
            // << leader_trq[0] << "," << leader_trq[1] << "," << leader_trq[2] << "," << leader_trq[3] << "," << leader_trq[4] << "," << leader_trq[5] << "," << leader_trq[6] << ","
            // << leader_vel[0] << "," << leader_vel[1] << "," << leader_vel[2] << "," << leader_vel[3] << "," << leader_vel[4] << ","  << leader_vel[5] << "," << leader_vel[6] << "," 
            // << follower_pos[0] << "," << follower_pos[1] << "," << follower_pos[2] << "," << follower_pos[3] << "," << follower_pos[4] << "," << follower_pos[5] << "," << follower_pos[6] << "," 
            // << follower_trq[0] << ","  <<follower_trq[1] << "," <<follower_trq[2] << "," <<follower_trq[3] << "," <<follower_trq[4] << "," <<follower_trq[5] << "," <<follower_trq[6] << ","
            // << follower_vel[0] << "," << follower_vel[1] << "," << follower_vel[2] << "," << follower_vel[3] << "," << follower_vel[4] << "," << follower_vel[5] << "," << follower_vel[6] << ","
            // << leader_ext_trq[0] << "," << leader_ext_trq[1] << "," << leader_ext_trq[2] << "," << leader_ext_trq[3] << "," << leader_ext_trq[4] << "," << leader_ext_trq[5] << "," << leader_ext_trq[6] << ","
            // << follower_ext_trq[0] << ","  << follower_ext_trq[1] << "," << follower_ext_trq[2] << "," << follower_ext_trq[3] << "," << follower_ext_trq[4] << "," << follower_ext_trq[5] << "," << follower_ext_trq[6] << ","
            // << "\n";

            return torques;

        };




        // control callback function
        auto trq_control_callback = [&] (const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
            
            if (!running.load()) {

                cout << "Exiting .... " << endl;
                
                return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
                
            }


            {
                lock_guard<mutex> lock(state_mutex);
                shared_robot_state = robot_state;
            }
            
            //array<double, 7> command_torques = computeUnilateralTrqs(robot_state);
            array<double, 7> command_torques = computeBilateralWithDOB(robot_state);

            array<double, 7> tau_cmd_rate_limited = franka::limitRate(franka::kMaxTorqueRate, command_torques, robot_state.tau_J_d);

            return tau_cmd_rate_limited;

        };




        while (running.load()) {

            try {

                //execute control loop
                robot.control(trq_control_callback);

            } catch (const franka::Exception& ex) {

                // print exception
                cout << ex.what() << endl;

                // auto recover
                robot.automaticErrorRecovery();
            }

        }   

        // stop thread
        running.store(false);
        pub_thread.join();
        sub_thread.join();
        key_thread.join();

        // file.close();

    } catch (const exception& ex) {

        cerr << "Standard exception: " << ex.what() << endl;

        return -1;

    } catch (...) {

        cerr << "Unknown exception caught" << endl;

        return -1;

    }

    return 0;
    
}