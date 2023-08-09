# python3 script that publishes sinusoidal velocity commands to the topic bigss_maxon_can_ros_node/servo_jv which is a sensor_msgs/JointState message


import rospy
from sensor_msgs.msg import JointState
import math

def main():
    rospy.init_node('example_sinusoidal_pos_script', anonymous=True)
    pub = rospy.Publisher('/act_unit/move_jp', JointState, queue_size=10)
    rate = rospy.Rate(200) 
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['joint1']

        max_amplitude_pos = 0.5
        speed_radpsec = 200.0
        freq=10.0
        msg.position = [max_amplitude_pos * math.sin(rospy.get_time()*freq)]
        msg.velocity = [speed_radpsec]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


