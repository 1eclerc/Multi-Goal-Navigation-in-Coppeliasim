# 🚗 Multi-Goal Navigation in CoppeliaSim

This project simulates an autonomous mobile robot navigating to multiple goal positions in a 2D environment using pose feedback and control logic in CoppeliaSim.

## 🎯 Missions

Three navigation missions are included:

1. **Letter C Drawing** – Navigate through markers to form the letter "C".
2. **Letter S Drawing** – Navigate through markers to form the letter "S".
3. **Student ID Pattern** – Navigate according to a custom path derived from student IDs.

## 🧠 Features

- 🚙 Pose-based control using position and yaw feedback
- 📈 Real-time trajectory plotting
- 📂 Auto-logging of motion data to CSV
- 🧭 Goal-specific heading alignment
- 📸 Visualization of all markers and navigation paths

## 🛠 Requirements

- Python 3.x
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- Required Python packages:
  ```bash
  pip install numpy matplotlib
  ```
- `coppeliasim_zmqremoteapi_client` module (comes with CoppeliaSim)

## 🧩 Simulation Setup

Your CoppeliaSim scene should contain:

- A differential-drive robot named `/LineTracer`
- Two motors: `/DynamicLeftJoint` and `/DynamicRightJoint`
- A front-mounted lidar (or dummy) for orientation: `/fastHokuyo`
- ArUco-like marker positions defined virtually in the script

## ▶️ How to Run

1. Start CoppeliaSim and load a compatible scene.
2. Run the Python script:
   ```bash
   python 07_03_go-to-multiple-goal.py
   ```
3. Choose which mission to execute when prompted.

## 📌 Marker Positions

Markers are pre-defined in the script in meters with `(x, y, theta)` coordinates.
Examples:
- `0: (1.0, 1.0, 0.0)` → Top Right
- `24: (0.0, 0.0, 0.0)` → Start & Finish marker

## 📊 Output

- Saves trajectory plots for each mission.
- Logs robot motion in `.csv` format for analysis.
- Displays a live matplotlib window of the robot’s movement.

## 👨‍🎓 Example Mission: Student ID Navigation

For student numbers `211805131` and `211805078`, the robot follows:
```
24 → 21 → 3 → 18 → 0 → 20 → 3 → 18 → 1 → 24
```

## 📄 Files Generated

- `robot_log_<mission>_<timestamp>.csv`: Time-series motion data.
- `<mission>_trajectory_<timestamp>.png`: Saved trajectory plot.

## 👥 Contributors

- @1eclerc | [GitHub](https://github.com/1eclerc)
- @ishowkenobi | [GitHub](https://github.com/ishowkenobi)

## 📜 License

This project is intended for educational use.
