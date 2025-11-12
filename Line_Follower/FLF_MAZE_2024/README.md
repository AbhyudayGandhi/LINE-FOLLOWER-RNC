# Meshmerize

The **Meshmerize** is a high-performance robot designed to efficiently navigate right-angled mazes. It uses an IMU sensor for precise turn detection and optimized path planning. Powered by an Arduino, this bot utilizes advanced sensor data, modular software, and configurable algorithms to solve mazes with both accuracy and speed.

## Features

### IMU-Based Turn Detection
- Uses an IMU sensor to detect and validate precise 90° turns.

### Redundant Path Optimization
- Implements a string replacement algorithm to eliminate unnecessary backtracks, minimizing the path length for efficient maze-solving.

### Dual Algorithm Modes
- Toggle between left-priority and right-priority maze-solving algorithms for different maze configurations.

### High-Speed Navigation
- Equipped with N20 12V 600 RPM motors for smooth and fast movement through the maze.

### Accurate Line Detection
- Uses the LSA08 Line Sensor Array for reliable line tracking.

### Extended Power
- Powered by an Orange 11.1V 2200mAh 30C 3S LiPo Battery, providing consistent and long-lasting performance.

## Table of Contents

- [Hardware Components](#hardware-components)
- [Software Installation](#software-installation)
- [Getting Started](#getting-started)
  - [Hardware Setup](#hardware-setup)
  - [Algorithm Overview](#algorithm-overview)
  - [Path Optimization](#path-optimization)
- [Testing and Calibration](#testing-and-calibration)
  - [Sensor Calibration](#sensor-calibration)
  - [IMU Calibration](#imu-calibration)
  - [Motor Testing](#motor-testing)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Hardware Components

| Component          | Description                                                  |
|--------------------|--------------------------------------------------------------|
| **Microcontroller** | Arduino (Uno/Nano/compatible)                               |
| **Line Sensor**     | LSA08 Line Sensor Array                                     |
| **IMU Sensor**      | Inertial Measurement Unit (IMU) for angle detection         |
| **Motor Driver**    | TB6612 Motor Driver                                          |
| **Motors**          | N20 12V 600 RPM Motors                                       |
| **Battery**         | Orange 11.1V 2200mAh 30C 3S LiPo Battery Pack               |
| **Buttons**         | Mode switch and restart button                              |
| **Switch**          | Left/Right priority algorithm toggle                        |

### Open the Code in Arduino IDE

1. Load `main.ino` for the complete maze-solving functionality.
2. Use `LSA_test.ino` for sensor calibration and testing.
3. Upload the code to the Arduino using the appropriate COM port.

## Getting Started

### Hardware Setup

1. **Assemble the Robot:**
   - Connect the LSA08 sensor, IMU sensor, motor driver, and motors to the Arduino as per the circuit diagram.
   
2. **Power the Bot:**
   - Connect the 11.1V LiPo battery to the power input of the robot.
   
3. **Configure the Algorithm:**
   - Toggle the left/right switch to select the maze-solving priority (left or right).

4. **Run the Bot:**
   - Press the mode button to switch between dry run and actual run modes.
   - Use the restart button to reset the bot during an actual run.

## Algorithm Overview

1. Initialize sensors (LSA08 and IMU), motors, and algorithm settings.
2. Select the left or right priority based on the hardware switch.
3. Continuously read sensor values to detect line positions and orientation.
4. Use the IMU sensor to detect and validate 90° turns.
5. Execute turns or corrections based on line deviation and IMU feedback.
6. Recalculate position and optimize the path at intersections.
7. Continue the process until the maze's end box is reached.

## Path Optimization

The robot employs a string replacement algorithm to optimize the path by eliminating unnecessary turns and backtracking:

- **Simplify Path:** Merge consecutive right/left turns into single optimized turns.
- **Update Navigation:** Continuously optimize the path during maze traversal.

### Replacement Rules

The following replacement rules are used to optimize the path by eliminating redundant moves and turning directions:

| Sequence | Replacement | Description                     |
|----------|-------------|---------------------------------|
| `RBL`    | `B`         | Simplifies U-turn sequence      |
| `RBR`    | `S`         | Replaces with a straight path   |
| `RBS`    | `L`         | Converts to a left turn         |
| `LBR`    | `B`         | Simplifies left U-turn          |
| `LBL`    | `S`         | Converts to a straight path     |
| `LBS`    | `R`         | Converts to a right turn        |
| `SBR`    | `L`         | Converts straight to left turn  |
| `SBS`    | `B`         | Simplifies redundant back turn  |
| `SBL`    | `R`         | Converts straight to right turn |

By applying these transformations, the bot efficiently navigates the maze with minimal backtracking and optimized turns.

## Testing and Calibration

### Sensor Calibration

1. Use the `LSA_test.ino` file to calibrate the line sensors:
   - Place the bot on a black line and white surface.
   - Adjust the threshold values in the code for optimal line detection.

### IMU Calibration

- Ensure the IMU sensor accurately detects 90° turns by calibrating it during the initial setup.

### Motor Testing

- Test the motor driver connections and motor speeds by loading simple movement scripts.

## Contact

For any queries or suggestions, feel free to reach out:

- Email: [roboticsandcircuits@gmail.com](mailto:roboticsandcircuits@gmail.com)
- GitHub: [github.com/RNCManipal](https://github.com/RNCManipal)
- LinkedIn: [linkedin.com/in/roboticsandcircuits](https://www.linkedin.com/company/robotics-and-circuits)
