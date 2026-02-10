# ROS 2 Visual Servoing with MoveIt & Gazebo

This project demonstrates a basic model visual servoing system using ROS 2, OpenCV, and MoveIt. The Python script (`visual_servoing.py`) runs a ROS 2 node that controls a robot arm to center a colored object in its camera's field of view and approach it.


## 1. Features added for now

*   **Color-Based Object Detection**: Uses OpenCV to detect an object based on a defined HSV color range. (replaced with deep learning models)
*   **Proportional Control**: Implements a simple P-controller to generate joint movements based on the visual error (position and size of the object in the image).
*   **MoveIt Integration**: Leverages the MoveIt 2 framework for robust and collision-aware motion planning.
*   **Joint Limit Safety**: Includes checks to prevent the robot from exceeding its joint limits.
*   **Real-time Visualization**: Provides a live camera feed with overlays showing the detected object, target location, and node status.

## 2. How It Works

The system is composed of two main classes within the `visual_servoing.py` script: `VisualPlannerNode` and `MoveItHelper`.

### `MoveItHelper` Class

This class acts as a simplified wrapper for the MoveIt `MoveGroup` action client.

*   It initializes a connection to the `/move_action` server provided by `move_group`.
*   The `move_to_joints` method is the core function. It takes target joint values, constructs a `MoveGroup.Goal`, and sends it to MoveIt.
*   It sets basic OMPL planner parameters for robust planning, such as planning time, velocity scaling, and planning attempts.
*   Callbacks (`goal_response_callback`, `get_result_callback`) handle the asynchronous nature of MoveIt actions, logging the result and setting a flag (`ready_for_next_move`) when the motion is complete.

### `VisualPlannerNode` Class

This is the main node that orchestrates the visual servoing logic.

#### Initialization (`__init__`)
*   Sets up subscriptions to `/camera1/image_raw` (for images) and `/joint_states` (for the robot's current configuration).
*   Initializes the `MoveItHelper`.
*   Creates a 0.5s timer that calls the main `planning_loop`.
*   Defines tunable parameters like `TARGET_AREA`, direction multipliers, and joint `LIMITS`.

#### Callbacks (`img_cb`, `joint_cb`)
*   These are simple callbacks that run in the background, continuously updating the node with the latest image message and joint positions from the robot.

#### Main Logic (`planning_loop`)

This function is called periodically by the timer and contains the core visual servoing algorithm.

1.  **Pre-condition Check**: The loop first checks if the system is ready for a new move, if a camera image has been received, and if joint states are available. If not, it exits early.

2.  **Image Processing**:
    *   The ROS `Image` message is converted to an OpenCV `bgr8` image.
    *   The image is converted to the HSV (Hue, Saturation, Value) color space, which is more robust for color detection under varying lighting conditions.
    *   A binary `mask` is created, isolating pixels that fall within a specified blue/purple color range.
    *   `cv2.findContours` is used to find all distinct shapes in the binary mask.

3.  **Object Analysis**:
    *   The largest contour is assumed to be the target object.
    *   Image moments (`cv2.moments`) are calculated for this contour to find its centroid (`cx`, `cy`) and `area`.

4.  **Error Calculation**:
    *   The script calculates the "error" between the object's current state and the desired state:
        *   `error_x`: The horizontal pixel distance from the object's center to the image center.
        *   `error_y`: The vertical pixel distance from the object's center to the image center.
        *   `error_area`: The difference between the object's current pixel area and the `TARGET_AREA`. This error is used to move the robot closer or further away.

5.  **Control & Movement Calculation**:
    *   **Goal Check**: If the errors are within a small tolerance, the node considers the goal reached and stops planning.
    *   **Proportional Control**: A simple proportional controller calculates the required change in joint angles. The joint step is directly proportional to the error (`step = error * gain`).
        *   `step_j1` (pan) is proportional to `error_x`.
        *   `step_j2` (tilt) is proportional to `error_y`.
    *   **Forward Motion**: The robot only moves forward/backward (`step_j3`) if it is reasonably centered on the target (`error_x` and `error_y` are small). This prevents it from moving toward the object when it's not properly aligned.

6.  **Safety & Execution**:
    *   **Joint Limit Check**: Before sending the command, the script calculates the `next` joint positions and checks if they violate the predefined `LIMITS`. If a limit is hit, that specific joint's movement is cancelled for the current cycle.
    *   **Clamping**: The final joint values are clamped to ensure they are strictly within the allowed range.
    *   **Execution**: The `moveit.move_to_joints` method is called with the final calculated joint values. The `ready_for_next_move` flag is set to `False` to prevent new plans from being made until the current one is finished.

## 3. Dependencies

*   **ROS 2**: Humble
*   **MoveIt 2**: The motion planning framework.
*   **OpenCV**: For image processing (`python3-opencv`).
*   **cv_bridge**: To convert between ROS Image messages and OpenCV images (`python3-cv-bridge`).
*   A ROS 2 workspace containing:
    *   A robot description (URDF/xacro).
    *   A Gazebo simulation launch file that spawns the robot and a camera.
    *   MoveIt configuration for your robot.

## 4. Setup & Installation

1.  **Clone the Repository**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/nipuna3p-hash/ros2_ws.git
    ```

2.  **Install Dependencies**:
    From the root of your workspace (`~/ros2_ws`), run `rosdep` to install any missing system dependencies.
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src -y --ignore-src
    ```

3.  **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_robot_description
    ```

4.  **Source the Workspace**:
    Open a new terminal and source your workspace to make the packages available.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## 5. How to Run

1.  **Launch Simulation & MoveIt**:
    In one terminal, launch your robot's simulation environment, which should include Gazebo, `robot_state_publisher`, and the MoveIt `move_group` node.
    ```bash
    # Example launch command, replace with your actual launch file
    ros2 launch my_robot_bringup robot_simulation.launch.py
    ```

2.  **Run the Visual Servoing Node**:
    In a second, sourced terminal, run the Python script.
    ```bash
    ros2 run my_robot_description visual_servoing
    ```
    A window titled "Robot Eye" should appear, showing the camera's view and the detection process. The robot arm should start moving to track the colored object.

## 6. Configuration & Tuning

Several parameters in the `__init__` method of the `VisualPlannerNode` can be tuned for your specific robot and environment.

```python
# --- TUNING ---
self.TARGET_AREA = 30000 
# Direction Multipliers (1 or -1)
self.DIR_X = -1 
self.DIR_Y = -1
self.DIR_Z = -1

# Strict Limits to prevent "Stuck" behavior
self.LIMITS = { 
    "joint1": (-3.0, 3.0), 
    "joint2": (-1.5, 1.5), 
    "joint3": (-1.5, 1.5) 
}
```

*   **`TARGET_AREA`**: An integer representing the desired area of the object's contour in pixels. A larger value will make the robot get closer to the object.
*   **`DIR_X`, `DIR_Y`, `DIR_Z`**: These are multipliers (`1` or `-1`) that correct the direction of movement. You may need to flip them depending on:
    *   The mounting orientation of your camera.
    *   The defined positive/negative direction of rotation for each joint in your URDF.
*   **`LIMITS`**: A dictionary defining the safe operational range (in radians) for each joint. **These should be set to values slightly inside the actual limits defined in your URDF** to prevent MoveIt from failing to find a plan near the edge.
*   **Color Range**: To detect a different color, modify the `np.array` values in the `cv2.inRange` function call inside `planning_loop`.

    ```python
    # From: Blue/Purple
    mask = cv2.inRange(hsv, np.array(), np.array())

    # To: Green
    # mask = cv2.inRange(hsv, np.array(), np.array())
    ```

## 5. Experiments
#### Test v1.6
[exp1.webm](https://github.com/user-attachments/assets/43d2ace7-621f-4e93-9f57-4343f8dd2df5)
#### Test v1.4
[testv1.0.webm](https://github.com/user-attachments/assets/ba094020-852b-4bd3-ab8c-1e714419f5d3)
#### Test v1.2
#### Test v1.1
#### Test v1.0
