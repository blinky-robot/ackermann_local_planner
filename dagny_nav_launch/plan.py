#!/usr/bin/env python
# visualize the direction of the global plan

import rospy

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

marker_id = 0

def main():
    rospy.init_node('path')

    pub = rospy.Publisher('path_marker', Marker)

    def path_sub(msg):
        global marker_id
        for pose in msg.poses:
            marker_id += 1
            print pose
            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.ARROW
            marker.pose = pose.pose
            o = marker.pose.orientation
            if o.x == 0 and o.y == 0 and o.z == 0 and o.w == 0:
                marker.pose.orientation.w = 1
            marker.id = marker_id
            marker.scale.x = 1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.g = 1.0
            marker.color.a = 1.0
            print marker
            pub.publish(marker)

    rospy.Subscriber('/move_base/SBPLLatticePlanner/plan', Path, path_sub)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_sub)

    rospy.spin()

if __name__ == '__main__':
    main()
