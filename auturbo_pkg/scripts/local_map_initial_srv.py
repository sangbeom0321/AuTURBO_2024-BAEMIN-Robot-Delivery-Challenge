#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def clear_costmap():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        clear_costmaps()
        rospy.loginfo("Cleared the costmaps")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('clear_costmap_timer')
    rospy.loginfo("Starting costmap clearing node...")
    # 2초마다 clear_costmap 함수 호출
    rospy.Timer(rospy.Duration(0.5), lambda event: clear_costmap())
    rospy.spin()