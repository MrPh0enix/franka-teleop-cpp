# Teleoperation Framework – Grasping Experiment

This repository contains a modified teleoperation framework for a dual-Franka grasping experiment.

---

## Starting the Robots

1. Make sure **both Franka robots** and their **control PCs** are powered on.
2. Open a web browser (preferably **Google Chrome**).
3. Connect to the Franka Control Interfaces:
   - `http://172.22.2.3/desk`
   - `http://172.22.2.4/desk`
4. Unlock the brakes on both robot arms.
5. Activate **FCI (Franka Control Interface)** on both robots.

---

## Compiling the Project

⚠️ **This step is only required if you are switching back from the `master` branch**  
or if the project has not been compiled yet.

From the **base project directory**, open a terminal and run:

```bash
rm -r build
mkdir build && cd build
cmake ..
make
```

### After compiling, run the leader and follower program

1. In the build folder in two separate terminals, run:
   ```bash
   ./leader <../lookups/lookup_file.csv>
   ```
   and 

   ```bash
   ./follower
   ```
