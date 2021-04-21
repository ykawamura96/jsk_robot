#! /usr/bin/env python3
import rospy
import numpy as np
import quaternion
import threading
from math import pi
from collections import namedtuple
try:
    from bcap_client import bcapclient
    from bcap_client.orinexception import ORiNException
except ImportError:
    print("run ../install_pybcap_package.sh")

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from bcap_ros_tools.srv import *
from bcap_ros_tools.msg import EndEffectorStatus

## bcap config
BcapConnectionConfig = namedtuple('BcapConnectionConfig', "host port timeout name provider machine option")
ROS_PUBISH_RATE = 125 # [hz]
HOST_IP = '133.11.216.51'

def deg2rad(deg):
    return deg / 360.0 * 2.0 * pi

def rad2deg(rad):
    return rad * 180.0 / pi

def rpm2rads(rpm):
    return rpm / 2.0 / pi * 60.0


class BcapRosBridgeNode:
    """Bridge ROS <-> Bcap
    1) Get sensor data from robot and pubslish them into ROS message.
    2) Recive service calls from ROS and execute them thro bcap protcol.
    TODO:
    gripper service
    """
    def __init__(self, joint_state_prefix=''):
        rospy.init_node('BcapRosBridgeNode', anonymous=True)
        host = HOST_IP
        self.joint_state_prefix = joint_state_prefix

        ## TODO: load from .yaml
        self.bcap_connection_config = BcapConnectionConfig(host=host, port=5007, timeout=2000,
                                                           name="", provider="CaoProv.DENSO.VRC", machine = ("localhost"),
                                                           option = ("Server = " + host))
        self.hand_width_max = 25
        self.hand_width_min = 2
        self.stop_thread = False
        self.arm_control = False
        self.setup_ros_node()
        self.setup_bcap_connection()
        # self.set_arm_speed()
        self.rate = ROS_PUBISH_RATE

    def setup_bcap_connection(self):
        """Start bcap connection.
        Make sure that the starting authority of robot is given to 'internet' instead of
        teaching pendant.
        """
        config = self.bcap_connection_config
        # start TCP
        self.m_bcapclient = bcapclient.BCAPClient(config.host, config.port, config.timeout)
        rospy.loginfo("Open TCP Connection")

        # start b_cap Service
        self.m_bcapclient.service_start("")
        rospy.loginfo("Send SERVICE_START packet")

        # conenct to RC8
        self.hCtrl = self.m_bcapclient.controller_connect(config.name, config.provider,
                                                          config.machine, config.option)
        rospy.loginfo("Connect RC8")

        self.HRobot = self.m_bcapclient.controller_getrobot(self.hCtrl, "Arm", "")
        rospy.loginfo("AddRobot")

    def take_arm_control_authority(self):
        """ On motor.
        """
        self.arm_control = True
        self.m_bcapclient.robot_execute(self.HRobot, "TakeArm", [0,1])
        rospy.loginfo("Taking arm contrl authority")

    def release_arm_control_authority(self):
        """Off motor.
        """
        self.arm_control = False
        self.m_bcapclient.robot_execute(self.HRobot, "GiveArm", None)
        rospy.loginfo("Releasing arm contrl authority")

    def set_arm_speed(self, s=57.0, a=32.49, d=32.48):
        rospy.logwarn("Setting robto arm speed: speed: {}, accel:{}, decel: {} [%]".format(s, a, d))
        self.m_bcapclient.robot_execute(self.HRobot, "ExtSpeed", [s, a, d])

    def setup_ros_node(self):
        """Register publishers and start services.
        """
        rospy.loginfo("regitering publishers and service host")
        self.joint_state_pub = rospy.Publisher('{}/joint_states'.format(self.joint_state_prefix),
                                               JointState,
                                               queue_size=10)
        self.ee_status_pub = rospy.Publisher('{}/end_effector_status'.format(self.joint_state_prefix),
                                             EndEffectorStatus,
                                             queue_size=10)
        self.gripper_pub = None  # TODO
        rospy.Service('{}/set_end_effector_pose'.format(self.joint_state_prefix),
                      SetEePose, self.set_target_pose_srv)
        rospy.Service('{}/set_hand_pos'.format(self.joint_state_prefix),
                      SetHandPos, self.set_hand_pos_srv)

    def publish_joint_state(self):
        """publish joint angle, velocity, and effort.
        """
        msg = self.get_joint_state()
        ret1 = self.m_bcapclient.controller_execute(self.hCtrl,"HandHoldState")
        
        self.joint_state_pub.publish(msg)

    def publish_ee_status(self):
        """publish pose of end-effector.
        """
        msg = self.get_ee_status()
        self.ee_status_pub.publish(msg)

    def get_joint_state(self):
        """Get joint posistion, velocity, effort including hand.
        Returns:
            sensor_msgs/JointState.msg
        """
        msg = JointState()
        rcv = self.m_bcapclient.robot_execute(self.HRobot,"GetAllSrvData")
        hand_pos = self.m_bcapclient.controller_execute(self.hCtrl,"HandCurPos")
        msg.header = Header(stamp=rospy.Time.now())
        msg.position = [deg2rad(p) for p in rcv[2][:6]] + [hand_pos]
        msg.velocity = [rpm2rads(v) for v in rcv[4][:6]] + [0]
        msg.effort = [eff/10.0 for eff in rcv[7][:6]] + [0.0]
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_gripper']
        return msg

    def get_ee_status(self):
        """Get end effector status including pos, orientation, rpy(euler), linear velocity.
        Returns:
            bcap_ros_bridge/EndEffectorStatus.msg
        """
        msg = EndEffectorStatus()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='world')
        rcv = self.m_bcapclient.robot_execute(self.HRobot,"CurPos")
        msg.pos.x = rcv[0]  # [mm]
        msg.pos.y = rcv[1]  # [mm]
        msg.pos.z = rcv[2]  # [mm]
        msg.rpy.x = rcv[3]  # [degree]
        msg.rpy.y = rcv[4]  # [degree]
        msg.rpy.z = rcv[5]  # [degree]
        q = quaternion.from_euler_angles([deg2rad(d) for d in rcv[3:6]])
        msg.orientation = Quaternion(q.w, q.x, q.y, q.z)
        msg.fig = rcv[6]
        rcv = self.m_bcapclient.robot_execute(self.HRobot,"GetSrvData",17)
        msg.linear_velocity.x = rcv[0]
        msg.linear_velocity.y = rcv[1]
        msg.linear_velocity.z = rcv[2]
        return msg

    def set_target_pose_srv(self, req):
        """Set target end effector position from service call.
        """
        overwrite = req.overwrite
        comp = 1 if not req.comp else req.comp  # comp = 1  (PTP) by default
        # TODO fixed fig wouldn't be good?
        fig = 261 # if not req.fig else req.fig  # fig = 261 by default
        m_bcapclient = self.m_bcapclient
        HRobot = self.HRobot

        x = req.pos.x  # [mm]
        y = req.pos.y  # [mm]
        z = req.pos.z  # [mm]

        if req.rpy:  # if specicied in rpy
            rx = req.rpy.x
            ry = req.rpy.y
            rz = req.rpy.z
        else:  # if specified in quaternion
            ori = req.orientation
            q = np.quaternion(ori.x, ori.y, ori.z, ori.w)
            rpy = quaternion.as_euler_angles(q)
            rx = rad2deg(rpy.x)  # degree
            ry = rad2deg(rpy.y)  # degree
            rz = rad2deg(rpy.z)  # degree
        target_pose = '@0 P('+ str(x)  + ',' + str(y)  +',' + str(z)  + ',' \
                      + str(rx) + ',' + str(ry) +',' + str(rz) + ',' \
                      + str(fig) + ')'

        # not executing out of range target pose
        out_range = m_bcapclient.robot_execute(HRobot, "OutRange", target_pose)
        if not out_range == 0:
            rospy.logwarn("Target pose {} is out of range".format(target_pose))
            return SetEePoseResponse(status=SetEePoseResponse.OUTRANGE)

        # turn on motor if it's off
        if not self.arm_control:
            rospy.loginfo("Motor ON")
            self.take_arm_control_authority()
        rospy.loginfo("Seding move command with target: {}".format(target_pose))
        try:
            if overwrite:
                isMotionComp = m_bcapclient.robot_execute(HRobot, "MotionComplete", [-1,1])
                if isMotionComp == 0 :                                        #0:Moving 1:Stop
                    rospy.loginfo("Overwriting target with motion-skip")
                    m_bcapclient.robot_execute(HRobot, "MotionSkip", [-1,3])  #StopMove Command
                m_bcapclient.robot_move(HRobot, comp, target_pose, "Next")    #Move Command
            else:
                m_bcapclient.robot_move(HRobot, comp, target_pose, "")
            return SetEePoseResponse(status=SetEePoseResponse.SUCCESS)
        except Exception as e:
            rospy.logerr("RC8 returnd error: {}".format(e))
            return SetEePoseResponse(status=e)

    def set_hand_pos_srv(self, req):
        """Set hand width, with speciying velocity and force
        """
        w_min = self.hand_width_min
        w_max = self.hand_width_max
        w = req.width
        grasp = req.grasp
        s = 100 if not req.speed else req.speed  # default 100[%]
        f = 6 if not req.force else req.force  # default 

        # if holding object and the target is smaller than current had pos, not moving
        hold = self.m_bcapclient.controller_execute(self.hCtrl,"HandHoldState")
        cur_pos = self.m_bcapclient.controller_execute(self.hCtrl,"HandCurPos")
        try:
            if not w:  # if width is not spesified, close hand until graps or open hand.
                if grasp == True:
                    # width = 25
                    # # while not hold
                    # while not self.m_bcapclient.controller_execute(self.hCtrl,"HandHoldState"):
                    #     width -= 3.0
                    #     self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveAH", [width, s, f])
                    #     if width <= 2.0:
                    #         break
                    self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveH", [f, True]) 
                    return SetHandPosResponse(status=SetHandPosResponse.SUCCESS)
                if grasp == False:
                    self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveAH", [25.0, s, f]) # open 25 [mm]
                    return SetHandPosResponse(status=SetHandPosResponse.SUCCESS)
            else:  # if width is speficied
                if hold and w < cur_pos:
                    rospy.logwarn("Hand is already holding object.")
                    return SetHandPosResponse(status=SetHandPosResponse.ALREADY_HOLDS)
                if (w < w_min) or (w_max < w):  # check range
                    rospy.logwarn("Hand width must be in range of {} ~ {}, but you set {}".format(w_min, w_max, w))
                    return SetHandPosResponse(status=SetHandPosResponse.OUTRANGE)
                else:
                    self.m_bcapclient.controller_execute(self.hCtrl, "HandMoveAH", [w, s, f])
                    return SetHandPosResponse(status=SetHandPosResponse.SUCCESS)
        except Exception as e:
            rospy.logerr("RC8 returnd error: {}".format(e))
            return SetHandPosResponse(status=e)
    
    def publisher_thread(self):
        """Publishe joint_states and ee_sattu in self.rate [hz]
        Working in different therad.
        """
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown() and not self.stop_thread:
            self.publish_joint_state()
            self.publish_ee_status()
            rate.sleep()
        rospy.loginfo("stopping publisher thread")

    def run(self):
        """Create publisher thread and spin.
        """
        self.publisher_worker = threading.Thread(target=self.publisher_thread)
        self.publisher_worker.start()
        rospy.on_shutdown(self.disconnect)
        rospy.spin()

    def disconnect(self):
        """Disconnection before shutting down node.
        """
        self.stop_thread = True
        self.publisher_worker.join()
        self.release_arm_control_authority()
        if(self.HRobot != 0):
            self.m_bcapclient.robot_release(self.HRobot)
            rospy.loginfo("Release Robot")
        if(self.hCtrl != 0):
            self.m_bcapclient.controller_disconnect(self.hCtrl)
            rospy.loginfo("Release Controller")
        self.m_bcapclient.service_stop()


if __name__ == '__main__':
    node = BcapRosBridgeNode(joint_state_prefix='cobotta')
    node.run()
