#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Optional TF broadcaster (set USE_TF_BROADCAST = True to enable)
USE_TF_BROADCAST = False
if USE_TF_BROADCAST:
    import tf
    import tf.transformations as tft

# ----------------------------
# Default settings (can be overridden with ROS params)
# ----------------------------
DEFAULTS = {
    "json_path": "/home/christian/projects/merlin_ws/src/ur_position_controller/traj_001.json",    # e.g. $(find your_pkg)/data/traj.json
    "topic_name": "/desired_pose",
    "base_frame": "base_link",                  # reference frame at the robot base
    "target_frame": "tool0",                    # frame the poses are meant for
    "publish_rate_hz": 100.0,                   # how fast to publish poses
    "loop_forever": False,                      # publish once or loop
    "latch_publisher": False                    # latched topic?
}

def load_params():
    p = dict(DEFAULTS)
    p["json_path"]       = rospy.get_param("~json_path",       p["json_path"])
    p["topic_name"]      = rospy.get_param("~topic_name",      p["topic_name"])
    p["base_frame"]      = rospy.get_param("~base_frame",      p["base_frame"])
    p["target_frame"]    = rospy.get_param("~target_frame",    p["target_frame"])
    p["publish_rate_hz"] = float(rospy.get_param("~publish_rate_hz", p["publish_rate_hz"]))
    p["loop_forever"]    = bool(rospy.get_param("~loop_forever",    p["loop_forever"]))
    p["latch_publisher"] = bool(rospy.get_param("~latch_publisher", p["latch_publisher"]))
    return p

def load_trajectory(json_path):
    if not os.path.isfile(json_path):
        rospy.logfatal("JSON file not found: %s", json_path)
        rospy.signal_shutdown("missing_json")
        return []

    with open(json_path, "r") as f:
        data = json.load(f)

    # Expected structure (based on your file): {'avg_lin_vel': float, 'trajectory': [ { 'time': float, 'pose': { 'position': {...}, 'orientation': {...}}}, ... ]}
    traj = data.get("trajectory", [])
    if not traj:
        rospy.logfatal("No 'trajectory' found in JSON or it's empty.")
        rospy.signal_shutdown("empty_trajectory")
        return []

    return traj

def make_pose_stamped(item, base_frame, target_frame):

    pos = item["pose"]["position"]
    ori = item["pose"]["orientation"]

    msg = PoseStamped()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = base_frame  # pose is expressed in base frame
    msg.pose.position.x = float(pos["x"])
    msg.pose.position.y = float(pos["y"])
    msg.pose.position.z = float(pos["z"])
    msg.pose.orientation.x = float(ori["x"])
    msg.pose.orientation.y = float(ori["y"])
    msg.pose.orientation.z = float(ori["z"])
    msg.pose.orientation.w = float(ori["w"])
    return msg

def main():
    rospy.init_node("pose_replayer", anonymous=False)

    params = load_params()
    json_path       = params["json_path"]
    topic_name      = params["topic_name"]
    base_frame      = params["base_frame"]
    target_frame    = params["target_frame"]
    publish_rate_hz = params["publish_rate_hz"]
    loop_forever    = params["loop_forever"]
    latch_pub       = params["latch_publisher"]

    rospy.loginfo("Pose replayer starting")
    rospy.loginfo("  json_path: %s", json_path)
    rospy.loginfo("  topic_name: %s", topic_name)
    rospy.loginfo("  base_frame: %s", base_frame)
    rospy.loginfo("  target_frame: %s", target_frame)
    rospy.loginfo("  publish_rate_hz: %.3f", publish_rate_hz)
    rospy.loginfo("  loop_forever: %s", loop_forever)
    rospy.loginfo("  latch_publisher: %s", latch_pub)

    traj = load_trajectory(json_path)
    if not traj:
        return

    pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10, latch=latch_pub)

    if USE_TF_BROADCAST:
        br = tf.TransformBroadcaster()

    rate = rospy.Rate(publish_rate_hz)

    index = 0
    n = len(traj)
    while not rospy.is_shutdown():
        item = traj[index]
        msg = make_pose_stamped(item, base_frame, target_frame)
        # refresh timestamp to "now" for deterministic rate playback
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

        if USE_TF_BROADCAST:
            # Broadcast the pose as a TF from base_frame -> target_frame
            p = msg.pose.position
            q = msg.pose.orientation
            br.sendTransform(
                (p.x, p.y, p.z),
                (q.x, q.y, q.z, q.w),
                msg.header.stamp,
                target_frame,   # child
                base_frame      # parent
            )

        index += 1
        if index >= n:
            if loop_forever:
                index = 0
            else:
                rospy.loginfo("Finished publishing %d poses. Shutting down.", n)
                break

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
