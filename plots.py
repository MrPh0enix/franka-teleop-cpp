import pandas as pd
import matplotlib.pyplot as plt

file = "teleop_recording_20260806_161322.csv"

expected_cols = [
    'local_time_s', 'robot_time_s', 'follower_connected',
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
    ("Position", "q"),
    ("Torque", "tau"),
    ("Velocity", "dq"),
    ("External Torque", "tau_ext"),
]

for joint in range(1, 8):

    for title, metric in metrics:

        leader_col = f"leader_{metric}{joint}"
        follower_col = f"follower_{metric}{joint}"

        leader_values = pd.to_numeric(
            data[leader_col],
            errors="coerce"
        )

        follower_values = pd.to_numeric(
            data[follower_col],
            errors="coerce"
        )

        valid = leader_values.notna() & follower_values.notna()

        plt.figure(figsize=(10, 4))

        plt.plot(
            leader_values[valid].to_numpy(),
            label="Leader"
        )

        # Flip follower torque sign
        if title == "Torque":
            plt.plot(
                -follower_values[valid].to_numpy(),
                label="Follower × -1"
            )
        else:
            plt.plot(
                follower_values[valid].to_numpy(),
                label="Follower"
            )

        plt.title(f"Joint {joint} - {title}")
        plt.xlabel("Sample")
        plt.ylabel(title)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

plt.show()