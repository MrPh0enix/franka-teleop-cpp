import pandas as pd
import matplotlib.pyplot as plt

file = "teleop_recording_20260806_161322.csv"

expected_cols = [
    'local_time_s','robot_time_s','follower_connected', 
    *(f"leader_q{i}" for i in range(1, 8)),
    *(f"leader_dq{i}" for i in range(1, 8)),
    *(f"leader_tau_ext{i}" for i in range(1, 8)),
    *(f"leader_tau{i}" for i in range(1, 8)),
    *(f"follower_q{i}" for i in range(1, 8)),
    *(f"follower_dq{i}" for i in range(1, 8)),
    *(f"follower_tau_ext{i}" for i in range(1, 8)),
    *(f"follower_tau{i}" for i in range(1, 8)),
    *(f"desired_ddq{i}" for i in range(1, 8)),
    *(f"command_tau{i}" for i in range(1, 8)),
    
]

# Read the file (ignores the trailing comma)
data = pd.read_csv(file)
data.columns = data.columns.str.strip()
data = data.loc[:, ~data.columns.str.startswith("Unnamed:")]

missing_cols = [col for col in expected_cols if col not in data.columns]

if missing_cols:
    print("Missing columns:")
    for col in missing_cols:
        print(f"  {col}")

    print("\nColumns actually found:")
    print(data.columns.tolist())

    raise ValueError("CSV header does not match the expected column names.")



metrics = [
    ("Position", "leader_q1", "follower_q1"),
    ("Torque", "leader_tau1", "follower_tau1"),
    ("Velocity", "leader_dq1", "follower_dq1"),
    ("External Torque", "leader_tau_ext1", "follower_tau_ext1"),
]

for title, leader, follower in metrics:
    # Convert invalid values to NaN instead of leaving mixed string/float data.
    leader_values = pd.to_numeric(data[leader], errors="coerce")
    follower_values = pd.to_numeric(data[follower], errors="coerce")

    valid = leader_values.notna() & follower_values.notna()

    plt.figure(figsize=(10, 4))
    plt.plot(leader_values[valid].to_numpy(), label="Leader")

    if title == "Torque":
        plt.plot(
            -follower_values[valid].to_numpy(),
            label="Follower × -1",
        )
    else:
        plt.plot(
            follower_values[valid].to_numpy(),
            label="Follower",
        )

    plt.title(f"Joint 1 - {title}")
    plt.xlabel("Sample")
    plt.ylabel(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

plt.show()