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