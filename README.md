# Teleoperation Framework – Grasping Experiment

This repository contains a modified teleoperation framework for a dual-Franka grasping experiment.

---

## Starting the Robots

1. Make sure **both Franka robots** and their **control PCs** are powered on.
2. Open a web browser (preferably **Google Chrome**).
3. Connect to the Franka Desks:
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

## After compiling, run the leader and follower program

1. In the build folder in two separate terminals, run:
   ```bash
   ./leader <../lookups/lookup_file.csv>
   ```
   and 

   ```bash
   ./follower
   ```
2. The program will automatically stop once the experiment has run its course. Run Ctrl+C on both the terminals to ensure they have exited correctly.

## Things to note

- Recordings will be stored in a **recordings** folder created automatically, with naming conventions taken from the lookup file.
- In case a iteration goes wrong, stop both the programs, delete the corrupted recording and restart. It will pick up from where it left off.
- Ask participants to refrain from touching the physical buttons on the robot as this will stop the program. In that case, follow the above instruction and restart.
- If the emergency stop has been pressed, follow the same process to restart.
- The `teleop_config.yml` file controls the parameters of the teleoperation setup.

