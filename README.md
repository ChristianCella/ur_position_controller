# Position controller 🕹️
This package contains a position controller for the robot ur5e from Universal Robots. The work was strated by [Niccolò Lucci](https://github.com/pucciland95) and is one of the key packages in case you want to perform teleoperation with the robot. In that case, refer to [this](https://github.com/ChristianCella/quest2ros/tree/main) repository.

---

### **Installation procedure** <a name="install"></a> 🏗️
1. **Create the merlin_ws workspace:**

    Follow the procedure described at the following [link](https://github.com/MerlinLaboratory/ur5e_mics). In the end, you should have the following structure inside the src folder:

    ```
    src/
        ├── aruco_tracking_and_calibration/      # package for the calibration
        ├── cnr_control_toolbox/                 # package with cnr controllers
        ├── ur_msgs/                             # package with the definition of messages
        ...
        └── ur5e_mics/                           # package for the ur5e robot
    ```

    At the moment, the tests have been performed by working inside the branch ```dev_christian``` of the package ```ur5e_mics```.
2. **Clone this repository:**

    Also for what concerns the position controller, the repo is still developed on this branch:
    ```
    cd (your_path_to_ws)/src 
    git clone -b dev_chris git@github.com:ChristianCella/ur_position_controller.git
    ```
    
---

### **Usage** <a name="usage"></a> ▶️

As of now, only some tests can be performed. In particular, you can test the position controller defined in [controller.cpp](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/src/controller.cpp) by publishing on the topic ```/desired_pose``` a set of different cartesian poses specified in [test_static_poses.py](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/test_static_poses.py). To test this repo, open four different terminals: 

1. **Terminal for the build:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    cd (your_path_to_ws)
    catkin build -cs
    ```
    The command '-cs' skips all the packages that cannot be built.

2. **Terminal to bring-up the robot:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    roslaunch ur5e_mics_bringup raw_control.launch
    ```

3. **Terminal to publish the desired pose:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    rosrun ur_position_controller test_static_poses.py
    ```

4. **Terminal to control the robot:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    rosrun ur_position_controller position_controller
    ```

As an alternative, the position controller can also be activated by running the python script [position_controller_python.py](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/position_controller_python.py), instead of running the node defined in [controller.cpp](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/src/controller.cpp):

    ```
    source (your_path_to_ws)/devel/setup.bash
    rosrun ur_position_controller position_controller_python.py
    ```

---

### **For teleoperation** <a name="usage"></a> 🎮

This package is pivotal to teleoperate the ur5e robot. For further details, llok at the branch ```dev_chris``` at the following [link](https://github.com/ChristianCella/quest2ros/tree/dev_chris). At the moment, you can either use the [position controller](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/src/controller.cpp) in c++ or the one implemented in python (defined [here](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/position_controller_python.py)); remember to impose ```teleop_link``` instead of ```base_link```, regardless of code you are using!

--- 

### **General comments** <a name="comments"></a> 📑

At the moment, both the controllers are thought for a 'relative' reference trajectory and not an 'absolute' one: in short, the trajectory must be 'attached' to the intial tcp position (for x, y, z coordinates), while the quaternion is simply imposed.  The situation is the following:

- The controller implemented in python is correct and allows to choose the time law encoded in the abscissa s(t) for the interpolation. The choices are:
    - Linear time law: you only need the reference position;
    - Cubic time law: you need position and velocity;
    - Quintic time law: you need position, velocity and acceleration.

    Despite the possibility to perform a better interpolation, the computation of the control action does not quite keep up with the 'real-time' publishing of the trajectory, due to some problems of the ros framework for python. 

- The controller implemented in c++, allows to compute with the reference values only the linear time law: despite there is the possibility to use higher order ploynomials, the message published on the topic ```/generate_motion_service_node/cartesian_path``` is a 'PoseStamped()', and only contains position. Some modifications are yet to be implemented, with the aim of accessing messages of type 'CartesianTrajectory' from the topic ```/generate_motion_service_node/cartesian_trajectory```, that also contains velocity and acceleration.
As of now, if you enable the use of a Cubic or Quintic polynomial, velocity and acceleration are computed with the finite difference method starting from position.