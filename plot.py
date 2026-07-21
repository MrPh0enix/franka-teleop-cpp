import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


FILE_PATH = "build/output_leader.txt"
SAMPLE_PERIOD = 0.001

data = pd.read_csv(FILE_PATH, header=None)

data = data.dropna(axis=1, how="all")


motor_trq_6 = data.iloc[:, 13].to_numpy()
tau_fil = data.iloc[:, 56].to_numpy()

time = np.arange(len(data)) * SAMPLE_PERIOD


plt.figure(figsize=(12,6))
plt.plot(time, motor_trq_6, label="link_side_trq[6]", linewidth=1)
plt.plot(time, tau_fil, label="TauFil", linewidth=2)

plt.xlabel("Time (s)")
plt.ylabel("Torque")
plt.title("link_side_trq[6] vs TauFil")
plt.grid(True)
plt.legend()

plt.show()