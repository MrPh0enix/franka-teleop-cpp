import panda_py
import panda_py.controllers
import numpy as np
import time
import keyboard



robot = panda_py.Panda("172.22.2.3")
# robot.move_to_start()


# movement params
TOP = [0.020740420948779374, 0.4145359941825532, -0.01734157898473112, -2.463463510446381, 0.002364673442724678, 2.881874662201685, 0.7747591458078886]
BOTTOM = [0.019917532626674173, 0.4682788796884972, -0.013689355042289221, -2.4587234609838116, 0.0024766670220357397, 2.9283000761697866, 0.774480865395318]

AMPLITUDE = [0, 0, 0, 0, 0, 0, 0]
MID = [0, 0, 0, 0, 0, 0, 0]
for i in range(len(TOP)):
    AMPLITUDE[i] = (BOTTOM[i] - TOP[i]) / 2
    MID[i] = (TOP[i] + BOTTOM[i]) / 2


robot.move_to_joint_position(TOP)      
robot.move_to_joint_position(MID)



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

# trqController = panda_py.controllers.AppliedTorque()
# robot.start_controller(trqController)


startTime = time.time()
freq = 0.16  # 1 oscillation in 6 sec
omega = 2 * np.pi * freq


def calc_pos(t):
    
    pos = []
    for i in range(len(MID)):
        pos.append(MID[i]+(AMPLITUDE[i]*np.sin(omega * t)))
    
    return pos


def calc_trq():
    
    tau_desired = np.zeros((7,))
    
    return tau_desired

rec = []

with robot.create_context(frequency = 250) as ctx:

    print('Player running')

    while ctx.ok():

        if keyboard.is_pressed('q'):
            robot.stop_controller()
            break

        t = time.time() - startTime

        pos = calc_pos(t) 

        trq = calc_trq()
        state = robot.get_state()
        line = [x for x in state.O_F_ext_hat_K] + [time.time()]
        
        rec.append(line)

        posController.set_control(pos)
        # trqController.set_control(trq)

print('Exited')

with open("out.txt", "w") as f:
    for line in rec:
        for x in line:
            f.write(str(x)+',')
        
        f.write('\n')



