//franka libs
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>

int main() {
    try {
        //connect to robot and initialize vals
        franka::Robot robot(config["leader"]["robot"].as<std::string>());
        franka::Model model = robot.loadModel();

        // move robot to start
        const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
        MotionGenerator motion_generator(0.5, home_pos);
        robot.control(motion_generator);


        // lambda functions to compute torques
        auto computeUnilateralTrqs = [&](std::array<double, 7>& joint_pos, std::array<double, 7>& joint_vel) {

            // initialize trqs
            std::array<double, 7> torques = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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


    }  catch (const std::exception& ex) {

        std::cerr << "Standard exception: " << ex.what() << std::endl;

        return -1;

    } catch (...) {

        std::cerr << "Unknown exception caught" << std::endl;

        return -1;

    
    }


