
#include <thread>
#include <yaml-cpp/yaml.h>

//franka libs
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include "examples_common.h"


int main () {

    // config
    YAML::Node config = YAML::LoadFile("teleop_config.yml");

    //connect to robot and initialize vals
    franka::Robot robot(config["follower"]["robot"].as<std::string>());
    franka::Model model = robot.loadModel();

    // // move robot to start
    // const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
    // MotionGenerator motion_generator(0.5, home_pos);
    // robot.control(motion_generator);

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    //start guiding mode
    std::array<bool, 6> guiding = {true, true, true, true, true, true};
    bool elbow = true;
    robot.setGuidingMode(guiding, elbow);


    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}