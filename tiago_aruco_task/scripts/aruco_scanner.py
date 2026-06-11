#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

MARKERS = {
    1: {"name": "Delivery Zone",   "x":  2.9813, "y":  2.1394},
    2: {"name": "Processing Zone", "x": -1.3212, "y": -0.9821},
    3: {"name": "Pickup Zone",     "x": -3.7234, "y":  1.6094},
}

DETECTION_RADIUS = 1.5  # metres

class ArucoScanner:
    def __init__(self):
        rospy.init_node("aruco_scanner")
        self.result_pub = rospy.Publisher("/aruco_scan_result", String,
                                          queue_size=10, latch=True)
        self.in_range = set()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                         self.pose_callback)
        rospy.loginfo("ArUco scanner ready. Monitoring checkpoint proximity...")
        rospy.spin()

    def pose_callback(self, msg):
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        for marker_id, cp in MARKERS.items():
            dist = math.sqrt((rx - cp["x"])**2 + (ry - cp["y"])**2)

            if dist < DETECTION_RADIUS and marker_id not in self.in_range:
                self.in_range.add(marker_id)
                result = (f"SCAN RESULT: ArUco Marker ID {marker_id} "
                          f"-> {cp['name']} (distance: {dist:.2f}m)")
                rospy.loginfo(result)
                self.result_pub.publish(String(data=result))

            elif dist >= DETECTION_RADIUS and marker_id in self.in_range:
                self.in_range.discard(marker_id)
                rospy.loginfo(f"Left range of Marker {marker_id} ({cp['name']})")

if __name__ == "__main__":
    ArucoScanner()
