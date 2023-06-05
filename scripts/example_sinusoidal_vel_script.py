# python3 script that publishes sinusoidal velocity commands to the topic bigss_maxon_can_ros_node/servo_jv which is a sensor_msgs/JointState message


import rospy
from sensor_msgs.msg import JointState
import math

def main():
    rospy.init_node('example_sinusoidal_vel_script', anonymous=True)
    pub = rospy.Publisher('bigss_maxon_can_ros_node/servo_jv', JointState, queue_size=10)
    rate = rospy.Rate(200) 
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        max_amplitude_rpm = 20
        max_amplitude_radpsec = max_amplitude_rpm * 2 * math.pi / 60
        msg.velocity = [max_amplitude_radpsec * math.sin(rospy.get_time())]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


