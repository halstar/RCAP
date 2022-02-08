#!/bin/bash

echo ""
echo "***** Starting setting several goals in turn ******"
echo ""

ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 1.107, y: -1.423, z: 0.0}, orientation: {z:  0.00, w: 1.0}}}' -1
sleep 14
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 1.587, y: -0.757, z: 0.0}, orientation: {z:  0.78, w: 1.0}}}' -1
sleep 12
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -1.09, y: 0.7637, z: 0.0}, orientation: {z: -1.00, w: 1.0}}}' -1
sleep 24
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -2.64, y: -0.280, z: 0.0}, orientation: {z: -1.70, w: 1.0}}}' -1
sleep 14
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -5.00, y:  0.180, z: 0.0}, orientation: {z:  1.00, w: 1.0}}}' -1
sleep 14
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -6.65, y: -1.106, z: 0.0}, orientation: {z: -0.97, w: 1.0}}}' -1
sleep 18
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 1.107, y: -1.423, z: 0.0}, orientation: {z:  0.00, w: 1.0}}}' -1

echo ""
echo "************ Done with goals testing **************"
echo ""
