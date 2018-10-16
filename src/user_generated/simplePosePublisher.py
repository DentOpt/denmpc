#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion

def talker():
    pub = rospy.Publisher('/usma_ardrone/mpc/desiredpose', PoseStamped, queue_size=10)
    rospy.init_node('desiredpose_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub_msg = PoseStamped()
    pub_msg.pose.position = Vector3(2.0, 0.25, 1.20)
    pub_msg.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    while not rospy.is_shutdown():
        pub_msg.header.stamp = rospy.get_rostime()
        pub.publish(pub_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# {
#     header: 
#     {
#         seq: 1,
#         stamp: {secs: 1, nsecs: 0},
#         frame_id: ''
#     },
#     pose: 
#     {
#         position: {x: 1.0, y: 0.0, z: 1.0}, 
#         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
#     }

# }