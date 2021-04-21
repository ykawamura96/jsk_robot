#! /usr/bin/python
from bcap_ros_tools.srv import SetEePose, SetEePoseRequest, SetHandPos, SetHandPosRequest
from bcap_ros_tools.msg import EndEffectorStatus
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, WrenchStamped
from jsk_rviz_plugins.msg import OverlayText
from omni_msgs.msg import OmniState


def get_status_overlay(moving, arm_name='larm'):
    text = OverlayText()
    text.width = 400
    text.height = 600
    text.left = 10 if arm_name == 'larm' else 600
    text.top = 10
    text.text_size = 25
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    on_txt = """<span style="color: red;">ON</span>"""
    off_txt = """<span style="color: white;">OFF</span>"""
    on_or_off = on_txt if moving else off_txt
    text.text = "TouchUSB control {}".format(on_or_off)
    return text

class CobottaTouchUSB(object):
    """Egg opening teleoperation demo.
    """

    def __init__(self, vel_scale=1.0, send_rot=True):
        rospy.init_node('CobottaTouchUSB')
        self.vel_scale = 0.001 * 0.001 * vel_scale  # [1000Hz] * [m/mm] * [1.0 (scaler)]
        self.send_rot = send_rot

        # setup ros
        rospy.Subscriber("/left_device/phantom/state", OmniState, self.device_state_sub)
        rospy.Subscriber('/cobotta/end_effector_status', EndEffectorStatus, self.ee_statsu_sub)

        self.status_overlay_pub = rospy.Publisher('/robot/status_overlay', OverlayText, queue_size=1)
        self.target_pose_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=1)
        self.set_cobotta_ee_pose_srv = rospy.ServiceProxy('/cobotta/set_end_effector_pose', SetEePose)
        self.set_cobotta_hand_srv = rospy.ServiceProxy('/cobotta/set_hand_pos', SetHandPos)
        
        rospy.wait_for_service('/cobotta/set_end_effector_pose')
        rospy.wait_for_service('/cobotta/set_hand_pos')
        

        self.target_pose = PoseStamped()
        self.moving = False
        self.is_gripper_closed = False
        # open gripper
        self.open_gripper()

    def ee_status_sub(self, msg):
        pass
        
    def device_state_sub(self, msg):
        # control gripper
        if msg.close_gripper and (not self.is_gripper_closed):
            rospy.loginfo("Close {} gripper")
            self.close_gripper()
            self.is_gripper_closed = True
        if (not msg.close_gripper) and self.is_gripper_closed:
            rospy.loginfo("Open {} gripper")
            self.open_gripper()
            self.is_gripper_closed = False
        # display status
        self.display_status()            
        if msg.locked:  # not move target pose when locked
            self.moving = False
            return
        # update self.target_pose
        self.moving = True        
        self.target_pose.pose.position.x += msg.velocity.y * self.vel_scale
        self.target_pose.pose.position.y += -msg.velocity.x * self.vel_scale
        self.target_pose.pose.position.z += msg.velocity.z * self.vel_scale
        if self.send_rot:
            cur_q = R.from_quat([msg.pose.orientation.x,
                                 msg.pose.orientation.y,
                                 msg.pose.orientation.z,
                                 msg.pose.orientation.w])
            tar_q = self.q_m_p * cur_q    # apply conversion
            self.target_pose.pose.orientation.x = -tar_q.as_quat()[0]
            self.target_pose.pose.orientation.y = tar_q.as_quat()[1]
            self.target_pose.pose.orientation.z = -tar_q.as_quat()[2]
            self.target_pose.pose.orientation.w = tar_q.as_quat()[3]
            # publish target pose
        self.target_pose_pub.publish(self.target_pose)
    
    def display_status(self):
        msg = get_status_overlay(moving=self.moving)
        self.status_overlay_pub.publish(msg)

    def open_gripper(self):
        req = SetHandPosRequest(grasp=False)
        self.set_cobotta_hand_srv(req)

    def close_gripper(self):
        req = SetHandPosRequest(grasp=True)
        self.set_cobotta_hand_srv(req)
