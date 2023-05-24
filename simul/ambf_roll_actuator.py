#!/usr/bin/env python3
import numpy as np
from ambf_client import Client
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import time
from scipy.spatial.transform import Rotation


class BIGSS_ROLL_AMBF:
    def __init__(self, client, name, use_simul_pos_for_vel=False):
        self.client = client
        self.name = name
        self.base = self.client.get_obj_handle(name + '/roll_base')
        self.use_simul_pos_for_vel = use_simul_pos_for_vel
        time.sleep(0.5)
        self.rate_hz = 120
        self.rate = rospy.Rate(self.rate_hz)

        self._base_pose_updated = False
        self._num_joints = 1
        self.servo_jv_cmd = [0.0]
        self.servo_jp_cmd = [0.0]
        self.servo_jp_flag = False
        self.pub_measured_js = rospy.Publisher(
            "/ambf/env/"+name+"/measured_js", JointState, queue_size=1)
        self.sub_servo_jp = rospy.Subscriber(
            "/ambf/env/"+name+"/servo_jp", JointState, self.sub_servo_jp_callback)
        self.sub_servo_jv = rospy.Subscriber(
            "/ambf/env/"+name+"/servo_jv", JointState, self.sub_servo_jv_callback)
        self.pub_measured_cp = rospy.Publisher(
            "/ambf/env/"+name+"/measured_cp", PoseStamped, queue_size=1)
        self.pub_jacobian = rospy.Publisher(
            "/ambf/env/"+name+"/jacobian", Float64MultiArray, queue_size=1)

    def is_present(self):
        if self.base is None:
            return False
        else:
            return True

    def servo_jp(self, jp):
        if (not self.is_present()):
            return
        self.base.set_joint_pos(0, jp[0])

    def servo_jv(self, jv):
        if self.use_simul_pos_for_vel:
            jp = [p + v/self.rate_hz for p,v in zip(self.measured_js(),jv)]
            self.servo_jp(jp)
            return
        if (not self.is_present()):
            return
        self.base.set_joint_vel(0, jv[0])

    def measured_js(self):
        j0 = self.base.get_joint_pos(0)
        q = [j0]
        self.js = q
        return q

    def measured_jv(self):
        j0 = self.base.get_joint_vel(0)
        return [j0]

    def get_joint_names(self):
        return self.base.get_joint_names()

    def sub_servo_jv_callback(self, msg):  # JointState
        self.servo_jv_cmd = msg.velocity

    def sub_servo_jp_callback(self, msg):  # JointState
        self.servo_jp_cmd = msg.position
        self.servo_jp_flag = True

    def publish_measured_js(self):
        msg = JointState()
        msg.name = 'BIGSS_ROLL_AMBF'
        msg.position = self.measured_js()
        msg.header.stamp = rospy.Time.now()
        self.pub_measured_js.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_present():
                self.base = self.client.get_obj_handle(
                    self.name + '/roll_base')
                print("Tried to reconnect")
            self.publish_measured_js()
            self.FK(self.measured_js())
            if self.servo_jp_flag:
                self.servo_jp(self.servo_jp_cmd)
                self.servo_jp_flag = False
            else:
                self.servo_jv(self.servo_jv_cmd)

            self.rate.sleep()

    def FK(self, th):
        FK_T = np.eye(4)  # Starting the transformations
        
        # rotation about z axis by theta
        R = Rotation.from_rotvec([0, 0, th[0]]).as_matrix()
        FK_T[0:3, 0:3] = R
        fk_msg = PoseStamped()
        fk_msg.header.stamp = rospy.Time.now()
        (fk_msg.pose.position.x, fk_msg.pose.position.y,
         fk_msg.pose.position.z) = FK_T[0:3, 3]
        q = Rotation.from_matrix(FK_T[0:3, 0:3]).as_quat()  # x,y,z,w

        (fk_msg.pose.orientation.x, fk_msg.pose.orientation.y,
         fk_msg.pose.orientation.z, fk_msg.pose.orientation.w) = q
        self.pub_measured_cp.publish(fk_msg)

        jac = np.zeros((6,1))
        jac[-1,0] = 1
        jac_msg = Float64MultiArray()
        jac_msg.layout.dim.append(MultiArrayDimension())
        jac_msg.layout.dim[0].size = 6
        jac_msg.layout.dim[0].stride = 1
        jac_msg.data = jac.flatten()
        self.pub_jacobian.publish(jac_msg)
        return FK_T, jac


if __name__ == "__main__":
    # Create a instance of the client
    while (not rospy.is_shutdown()):
        _client = Client("BIGSS_ROLL_AMBF")
        _client.connect()
        roll_act = BIGSS_ROLL_AMBF(_client, 'roll_act')
        time.sleep(0.5)
        if roll_act.base is not None:
            print("Found AMBF client and loaded")
            break
        else:
            print("Assuming ambf client still loading and waiting...")
            _client.clean_up()
            time.sleep(0.5)

    while (not rospy.is_shutdown()):
        while (roll_act.base.get_num_joints() < 1):
            if rospy.is_shutdown():
                quit()
            print("Waiting for roll_act model to load...")
            time.sleep(0.5)
        print("BIGSS_ROLL_AMBF loaded")
        # set init pose
        roll_act.servo_jp([0.0])
        time.sleep(2.0)  # let the move command finish
        roll_act.run()
        _client.clean_up()
