# Mobile Robot with Arm Simulation

## Overview

This project implements a mobile robot with a 2-DoF arm in PyBullet that detects, approaches, and taps red cubes using computer vision, state machines, and kinematic control.

## Features

* **Real-time Red Cube Detection** via HSV color thresholding and morphological filtering
* **Proportional Steering Control** for base navigation with dynamic gain adjustment
* **2-DoF Planar Inverse Kinematics** with joint limit checking
* **OpenCV Visualization** of the robot’s camera feed, masked blobs, and control overlays
* **State Machine** architecture for sequenced behaviors

## Requirements

* Python 3.8+
* [PyBullet](https://pypi.org/project/pybullet/)
* [OpenCV-Python](https://pypi.org/project/opencv-python/)
* NumPy

## Installation

1. **Create and activate a Conda environment**:

   ```bash
   conda create -n robo2dof python=3.8 -y
   conda activate robo2dof
   ```
2. **Install dependencies**:

   ```bash
   pip install pybullet opencv-python numpy
   ```
3. **Clone this repository** and ensure `main_updated_fixed.py` and your URDF are in the same folder.

## Usage

Run the main script:

```bash
python main_updated_fixed.py
```

* A window will appear showing the camera feed with overlays.
* Press `ESC` to exit.

## Technical Approach

### 1. Perception

* **Camera Setup**: Mounted on the `camera_link` in the URDF with 256×256 resolution and a 60° field of view. Internally uses PyBullet’s `getCameraImage` interface.
* **Color Space Conversion**: RGB → HSV conversion for robust color segmentation under varying lighting.
* **Threshold & Noise Reduction**: HSV thresholds isolate red hues; followed by morphological erosion and dilation to remove spurious blobs.
* **Centroid & Area Filtering**: Compute image moments to find the largest red blob’s centroid and area to reject small artifacts.

### 2. Coordinate Transformations

* **Pixel-to-World Mapping**: Using the camera’s intrinsic matrix and depth buffer from PyBullet, project the 2D centroid into 3D coordinates relative to `camera_link`.
* **Frame Conversion**: Transform 3D points from camera frame into base link frame via homogeneous transformation matrices extracted from the URDF joint poses.

### 3. Base Control

* **State Machine**:

  * **`search`**: Rotate at a fixed rate until the red blob’s area exceeds a minimum threshold.
  * **`approach`**: Use a proportional controller on angular velocity:
    $\omega = -k_p \times x_{error}$
    where $x_{error}$ = normalized pixel offset, and linear velocity scales inversely with angular error.
  * **`tap`**: Stop base movement when Euclidean distance to target $d < d_{thresh}$.
* **Control Gains**: Tuned $k_p$ and linear velocity bounds via grid search to ensure smooth convergence without oscillation.

### 4. Arm Control & Inverse Kinematics

* **Planar Arm Model**: Two revolute joints with link lengths \$L\_1\$ and \$L\_2\$.
* **Analytical IK**: Solve via the Law of Cosines:

$$
\theta_2 = \cos^{-1}\left(\frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}\right)
$$

$$
\theta_1 = \arctan2(y, x) - \arctan2\left(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2\right)
$$

where \$(x, y)\$ is the target position in the arm’s base frame.

* **Joint Limits & Collision Avoidance**: Clamp each \$\theta\_i\$ within URDF-specified ranges and verify no self-collisions using PyBullet’s `getClosestPoints`.
* **Trajectory Smoothing (Future Work)**: Fit a cubic B-spline through intermediate IK waypoints to generate smooth, continuous joint-space paths.

### 5. Trajectory Smoothing (Future Work)

* **B-Spline Generation**: Compute intermediate waypoints between current arm pose and tap pose; parameterize via a cubic spline for smooth velocity and acceleration profiles.
* **Time Parameterization**: Use uniform sampling along spline based on maximum joint velocity limits.

### 6. Sensor Noise & Robustness

* **Depth Noise Filtering**: Apply a median filter on the depth buffer to reduce pixel-level jitter before point projection.
* **Dynamic Threshold Adaptation**: Adjust HSV thresholds in real-time using histogram analysis to adapt to lighting changes.

### 7. Coordinate Calibration

* **Hand‑Eye Calibration**: (If available) Use a checkerboard calibration to refine the extrinsic transform between the camera and base link for more accurate world projection.

## Visualization

* **OpenCV Overlays**: Draw bounding circles, centroids, and crosshairs on the RGB feed; display current state and error metrics.
* **Debug Logging**: Log state transitions, control commands, and IK angles to a CSV file for post-run analysis.

## Limitations & Future Work

* **Multi-Cube TSP**: Extend the state machine to detect and queue multiple red cubes, then apply a Traveling Salesman heuristic for path ordering.
* **Dynamic Obstacles**: Integrate A\* or RRT path planners to navigate around obstacles in real time.
* **Spline-Based Arm Trajectories**: Implement cubic or quintic spline interpolation for smoother tapping motions.
* **ROS Integration**: Wrap perception and control in ROS2 nodes and leverage `tf2` for frame broadcasting.

## Submission Contents

* `main_updated_fixed.py`: Python script containing perception, control, and IK routines.
* `mycar.urdf`: URDF description of the mobile base and 2-DoF arm.
* `README.md`: This file.

<img width="2878" height="1534" alt="image" src="https://github.com/user-attachments/assets/420079ea-56f0-482f-a32b-2a4b38120a23" />
