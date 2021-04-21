#! /usr/bin/python
import numpy as np
import rospy
from absl import app
from absl import flags
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Pose, Point, Vector3
from kortex_driver.srv import GetMeasuredCartesianPose
from bcap_ros_tools.srv import SetEePose, SetEePoseRequest, SetHandPos, SetHandPosRequest

FLAGS = flags.FLAGS
flags.DEFINE_float("scale", 1.0, "Coordinate transofrm scale, ex) if scale is 0.1"\
                   ", cobotta will move 0.1[mm] when kinova moved 1.0[mm]")

                   
class CobottaKinovaTeleop(object):

    def __init__(self, scale=1.0):
        self.scale = scale
        self.center_kinova = np.array([0.497, 0.0, 0.104])
        self.center_cobotta = np.array([225.0, -12.0, 121.0])        
        self.counter = 0
        self.setup_ros()

    def setup_ros(self):
        rospy.init_node('CobottaKinovaTeleop')
        rospy.loginfo("Kinova-Cobotta position based connection\n"\
                      "Scale of movement kinova : cobotta is {} : 1.0".format(self.scale))
        rospy.Subscriber("/arm_gen3/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber("/cobotta/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.get_kinova_ee_pose_srv = rospy.ServiceProxy('/arm_gen3/base/get_measured_cartesian_pose', GetMeasuredCartesianPose)
        self.set_cobotta_ee_pose_srv = rospy.ServiceProxy('/cobotta/set_end_effector_pose', SetEePose)
        self.set_cobotta_hand_srv = rospy.ServiceProxy('/cobotta/set_hand_pos', SetHandPos)
        rospy.wait_for_service('/arm_gen3/base/get_measured_cartesian_pos')
        rospy.wait_for_service('/cobotta/set_end_effector_pose')
        rospy.wait_for_service('/cobotta/set_hand_pos')

    def joy_cb(self, msg):
        open_gripper = msg.buttons[1]   # O to open gripper
        close_gripper = msg.buttons[0]  # X to close gripper
        if open_gripper:
            rospy.loginfo("Opening hand")
            req = SetHandPosRequest(grasp=False)
            self.set_cobotta_hand_srv(req)
        if close_gripper:
            rospy.loginfo("Closing hand")
            req = SetHandPosRequest(grasp=True)
            self.set_cobotta_hand_srv(req)
        
    def joint_state_cb(self, msg):
        pose = self.get_kinova_ee_pose_srv()
        target_pos = self._convert_pos(np.array([pose.output.x, pose.output.y, pose.output.z]))
        target_rpy = Vector3(pose.output.theta_x,
                             pose.output.theta_y,
                             pose.output.theta_z)
        req = SetEePoseRequest(overwrite=True,
                               pos=target_pos,
                               rpy=target_rpy)
        if self.counter % 3 == 0:
            ret = self.set_cobotta_ee_pose_srv(req)
        self.counter += 1

    def _convert_pos(self, orig_pos):  # orig_pos: np.ndarray with shape of (3,)
        target_pos = self.scale * (orig_pos - self.center_kinova) * 1000.0 + self.center_cobotta
        return Point(x=target_pos[0], y=target_pos[1], z=target_pos[2])        

    def run(self):
        rospy.spin()

    def __del__(self):
        rospy.loginfo("Finish kinova-cobtta connection...")


def main(argv):
    cobotta_kinova_teleop = CobottaKinovaTeleop(scale=FLAGS.scale)
    cobotta_kinova_teleop.run()
    
    
if __name__ == '__main__':
    app.run(main)
