import pandas as pd
import matplotlib.pyplot as plt

file = "build/output_leader.txt"

cols = [
    *(f"motor_pos_{i}" for i in range(7)),
    *(f"motor_trq_{i}" for i in range(7)),
    *(f"motor_vel_{i}" for i in range(7)),
    *(f"follower_pos_{i}" for i in range(7)),
    *(f"follower_trq_{i}" for i in range(7)),
    *(f"follower_vel_{i}" for i in range(7)),
    *(f"motor_ext_trq_{i}" for i in range(7)),
    *(f"follower_ext_trq_{i}" for i in range(7)),
]

# Read the file (ignores the trailing comma)
data = pd.read_csv(file, header=None)
data = data.iloc[:, :56]
data.columns = cols

metrics = [
    ("Position", "motor_pos_0", "follower_pos_0"),
    ("Torque", "motor_trq_0", "follower_trq_0"),
    ("Velocity", "motor_vel_0", "follower_vel_0"),
    ("External Torque", "motor_ext_trq_0", "follower_ext_trq_0"),
]

for title, motor, follower in metrics:
    plt.figure(figsize=(10,4))
    plt.plot(data[motor], label="Motor")

    if title == "Torque":
        plt.plot(-data[follower], label="Follower × -1")
    else:
        plt.plot(data[follower], label="Follower")
        
    plt.title(f"Joint 0 - {title}")
    plt.xlabel("Sample")
    plt.ylabel(title)
    plt.grid(True)
    plt.legend()

plt.show()