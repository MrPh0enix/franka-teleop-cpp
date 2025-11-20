import panda_py
import panda_py.controllers
import numpy as np
import time



robot = panda_py.Panda("172.22.2.3")
robot.move_to_start()


# movement params
TOP = [0.0197862, 0.475927, -0.0844572, -2.35928, 0.0855179, 2.85254, 0.781102]
BOTTOM = [0.0207323, 0.527758, -0.0849659, 2.34778, 0.084263, 2.85764, 0.774969]

AMPLITUDE = [0, 0, 0, 0, 0, 0, 0]
MID = [0, 0, 0, 0, 0, 0, 0]
for i in range(len(TOP)):
    AMPLITUDE[i] = (TOP[i] - BOTTOM[i]) / 2
    MID[i] = (TOP[i] + BOTTOM[i]) / 2


       
robot.move_to_joint_position(TOP)

robot_settings = robot.get_robot()
robot_settings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    upper_torque_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    lower_torque_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    upper_torque_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    lower_force_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]], 
                                    upper_force_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                    lower_force_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                    upper_force_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])


posController = panda_py.controllers.JointPosition()
robot.start_controller(posController)


startTime = time.time()
freq = 0.16  # 1 oscillation in 6 sec
omega = 2 * np.pi * freq


def calc_pos(t):
    
    pos = []
    for i in range(len(MID)):
        pos.append(MID[i]+(AMPLITUDE[i]*np.sin(omega * t)))
    
    return pos



with robot.create_context(frequency = 100) as ctx:

    print('Player running')

    while ctx.ok():

        t = time.time() - startTime

        pos = calc_pos(t) 

        posController.set_control(pos)


