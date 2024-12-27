# Teleoperation 🕹️
This repo allows to control the ur5e robot with teleoperation.  The work was strated by [Niccolò Lucci](https://github.com/pucciland95) and it relies on Oculus Quest 2 from Meta (Look at the following [lnk](https://www.digitalmosaik.com/it/blog/oculus-quest-2/) for more details on the device).

---

### **quest2ros** <a name="quest2ros"></a> 👓

The repository at the following [link](https://github.com/pucciland95/quest2ros) still needs to be cloned in the src folder of the complete project (the 'src' folder mentioned here is the one that is created during the procedure detailed [here](https://github.com/MerlinLaboratory/ur5e_mics) and described more in detail in [Installation procedure](#install)).

---

### **Installation procedure** <a name="install"></a> 🏗️
1. **Create the merlin_ws workspace:**

    Follow the procedure described at the following [link](https://github.com/MerlinLaboratory/ur5e_mics). In the end, you should have the following structure inside the src folder:

    ```
    src/
        ├── aruco_tracking_and_calibration/      # package for the calibration
        ├── cnr_control_toolbox/                 # package with cnr controllers
        ├── ur_msgs/                             # package with the definition of messages
        └── ur5e_mics/                           # package for the ur5e robot
    ```
2. **Clone this repository:**

    At the moment, the repo is still developed on this branch:
    ```
    cd ../merlin_ws/src (navigate to your directory)
    git clone -b dev_chris git@github.com:ChristianCella/ur_position_controller.git
    ```

---

### **Usage** <a name="usage"></a> ▶️

As of now, only some tests can be performed. In particular, you can test the position controller defined in [controller.cpp](https://github.com/ChristianCella/ur_position_controller/blob/dev_chris/src/controller.cpp) by publishing on the topic ```/desired_pose``` a set of different cartesian poses specified in [test.py]()

Open 4 terminals and 