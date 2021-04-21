#!/usr/bin/env python
import os
import os.path as osp
import threading
import signal
import rospy
import subprocess
import shlex
import quaternion
import time
import numpy as np
from math import pi
from transitions import Machine
from datetime import datetime
from pynput import keyboard 

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, Point
from std_msgs.msg import Header

from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.msg import Pictogram

from bcap_ros_bridge.srv import *
from bcap_ros_bridge.msg import EndEffectorStatus

import helpers
from helpers import overlaytext_from_string
from helpers import create_pictogram_msg

from absl import app
from absl import flags

FLAGS = flags.FLAGS

flags.DEFINE_string('task_name',
                    None,
                    'Name of task')
flags.DEFINE_integer('variation',
                    0,
                    'Variation of task')
flags.DEFINE_string('save_dir',
                    None,
                    'Name of task')


TEXT_START_MOVE = "MOVING... Press (r) to start recording, press (h) to stop moving, press (i) to return to initial pose."
TEXT_STOP_MOVE = "STOP MOVING... Press (s) to start moving"
TEXT_NO_IK_SOLTUTION = "CANNOT FIND IK SOLUTION"
TEXT_SLEF_COLLISION = "DETECTED SELF COSSITION"
TEXT_WELCOME = "WELCOME. Press (s) move to initial pose... "
TEXT_MOVE_WITH_RECORD = "RECORDING DEMONSTRATION... Press (x) to stop recording, press (i) to return to initial pose."
TEXT_RETURN_TO_INITIAL_POSE = "RETURNING TO INITIAL POSE..."


def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()  # we wait for children to terminate


def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s + "_") or str == s):
            os.system("rosnode kill " + str)
    
def yes_no_input_with_message(message):
    while True:
        choice = raw_input("{}\nPlease respond with yes or no [y/N]: ".format(message)).lower()
        if choice in ["y", "ye", "yes"]:
            return True
        elif choice in ["n", "no"]:
            return False


def check_and_make(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)


def deg2rad(deg):
    return deg / 360.0 * 2 * pi


class TeleopStateMachine(object):
    states = ['on_init', 'stop', 'move', 'move_to_initial_pose', 'move_with_record']
    def __init__(self, name):
        self.name = name
        self.machine = Machine(model=self, states=TeleopStateMachine.states, initial='on_init', ignore_invalid_triggers=True)
        self.machine.add_transition(trigger='start_move',   source='on_init',              dest='move_to_initial_pose')
        self.machine.add_transition(trigger='reset_move',   source='*',                    dest='move_to_initial_pose')
        self.machine.add_transition(trigger='done',         source='move_to_initial_pose', dest='stop')               
        self.machine.add_transition(trigger='start_move',   source='stop',                 dest='move')              
        self.machine.add_transition(trigger='stop_move',    source='*',                    dest='stop')               
        self.machine.add_transition(trigger='stop_record',  source='move_with_record',     dest='move')
        self.machine.add_transition(trigger='start_record', source='move',                 dest='move_with_record')


class CobottaTeleop(object):
    """Teleoperate cobotta using 3D mouse.
    """
    def __init__(self, save_dir, save_episode_start_count, pos_scale=5.0, rot_scale=0.1, use_keyboard=False):
        rospy.init_node('CobottaTeleopNode', anonymous=True)
        self.m = TeleopStateMachine("name",)

        self.save_dir = save_dir
        self.save_episode_start_count = save_episode_start_count

        # for mouse_cb
        self.hand_pos_scale = 5.0
        self.arm_command_period_sec = 300.0 / 1000.0  # 200[ms]
        self.last_mouse_stamp = None
        self.target_ee_state = None
        self.grasp = False
        self.pos_scale = pos_scale
        self.rot_scale = rot_scale

        # for record
        self.record_mode = False
        
        self.setup_ros()
        self.use_keyboard = use_keyboard
        if self.use_keyboard:
            self.setup_keyboard_listenr()
        else:
            self.setup_joybad_listener()
        self.return_to_initial_pose()
        
    def setup_keyboard_listenr(self):
        """ setup and start keyboard monitorthread
        """
        rospy.loginfo("Start capturing keyboard")
        self.key_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.key_listener.start()

    def setup_joybad_listener(self):
        rospy.Subscriber('/joy', Joy, self.joypad_cb, queue_size=1)
        
    def setup_ros(self):
        """setup ros publisehrs and subscribers
        """
        rospy.loginfo("setting up publishers...")
        self.text_pub = rospy.Publisher("teleop_info", OverlayText, queue_size=1)
        self.pict_pub = rospy.Publisher("teleop_pictgram", Pictogram, queue_size=1)
        rospy.Subscriber('/spacenav/joy', Joy, self.mouse_cb, queue_size=1)
        rospy.Subscriber('/end_effector_status', EndEffectorStatus, self.ee_state_cb, queue_size=1)
        self.target_ee_pose_pub = rospy.Publisher('/target_ee_pose', PoseStamped,queue_size=1)
        rospy.loginfo("waiting for servie...")
        rospy.wait_for_service('set_end_effector_pose')
        self.set_ee_pose_service = rospy.ServiceProxy('set_end_effector_pose', SetEePose)
        self.set_hand_pos_service = rospy.ServiceProxy('set_hand_pos', SetHandPos)
        self.update_info()

    def return_to_initial_pose(self):
        # end effector
        self.initial_pose_req = SetEePoseRequest()
        self.initial_pose_req.pos = Point(255.0, -44.50, 77.65)
        self.initial_pose_req.rpy = Vector3(180.0, 0, 180.0)
        self.initial_pose_req.fig = 261
        self.initial_pose_req.overwrite = False
        self.initial_pose_req.comp = 1
        self.set_ee_pose_service(self.initial_pose_req)
        # hand
        self.set_hand_pos_service(SetHandPosRequest(grasp=False))
        # update info
        self.m.done()
        self.update_info()
        # self.target_ee_state = self.current_ee_state
        self.grasp = False
        # self.publish_target_ee_pose()

    def mouse_cb(self, msg):
        """3D Mouse callback.
        Accumulate commanded offset and send command to achive that offset in fixed frequency.
        """        
        if not (self.m.is_move() or self.m.is_move_with_record()):
            self.target_ee_state = self.current_ee_state
            return
        # process hand
        if msg.buttons[0] or msg.buttons[1]:
            self.grasp = not self.grasp
        if msg.buttons[0] or msg.buttons[1]:  # if updated
            self.set_hand_pos_service(SetHandPosRequest(grasp=self.grasp))
        # process end effector
        if not self.last_mouse_stamp:
            self.last_mouse_stamp = msg.header.stamp
            return
        if self.target_ee_state is None:
            self.target_ee_state = self.current_ee_state

        now_sec = rospy.Time.now().to_sec()
        if now_sec - self.last_mouse_stamp.to_sec() < self.arm_command_period_sec:
            self._accumulate_target_diff(msg)
        else:
            self.last_mouse_stamp = self.target_ee_state.header.stamp
            self.execute_target_ee_state()
        self.publish_target_ee_pose()

    def ee_state_cb(self, msg):
        """listen current end effector state
        """
        self.current_ee_state = msg

    def execute_target_ee_state(self):
        """Execute bcap command to achive self.target_ee_state.
        """
        req = SetEePoseRequest()
        req.pos = self.target_ee_state.pos
        req.rpy = self.target_ee_state.rpy
        req.fig = self.target_ee_state.fig
        req.overwrite = True
        req.comp = 1
        self.set_ee_pose_service(req)

    def publish_target_ee_pose(self):
        """Convert target_ee_state to geometry_msgs/Pose to visualzie tareget pose in Rviz
        """
        msg = PoseStamped()
        msg.header = self.target_ee_state.header
        msg.pose.position.x  = self.target_ee_state.pos.x / 1000.0  # [mm] -> [m]
        msg.pose.position.y  = self.target_ee_state.pos.y / 1000.0  # [mm] -> [m]
        msg.pose.position.z  = self.target_ee_state.pos.z / 1000.0  # [mm] -> [m]
        msg.pose.orientation = self.target_ee_state.orientation
        self.target_ee_pose_pub.publish(msg)
        
    def _accumulate_target_diff(self, msg):
        """Update self.target_ee_state to accumulate diff (or offset) from 3D mouse
        """
        self.target_ee_state.header.stamp = rospy.Time.now()
        self.target_ee_state.pos.x += self.pos_scale * msg.axes[0]  # [mm]
        self.target_ee_state.pos.y += self.pos_scale * msg.axes[1]  # [mm]
        self.target_ee_state.pos.z += self.pos_scale * msg.axes[2]  # [mm]
        self.target_ee_state.rpy.x += self.rot_scale * msg.axes[3]  # [degree]
        self.target_ee_state.rpy.y += self.rot_scale * msg.axes[4]  # [degree]
        self.target_ee_state.rpy.z += self.rot_scale * msg.axes[5]  # [degree]
        q = quaternion.from_euler_angles([deg2rad(self.target_ee_state.rpy.x),
                                          deg2rad(self.target_ee_state.rpy.y),
                                          deg2rad(self.target_ee_state.rpy.z)])
        self.target_ee_state.orientation = Quaternion(q.w, q.x, q.y, q.z)

    def execute_state(self):
        """Do whatever things in state
        """
        if self.m.is_move_to_initial_pose():
            self.return_to_initial_pose()
        if self.m.is_move_with_record():
            self.start_record()
        if self.m.is_move():
            self.stop_record()
        if self.m.is_stop():
            self.target_ee_state = self.current_ee_state
            self.publish_target_ee_pose()
            self.stop_record()

    def start_record(self):
        # TODO bagfile name, how to spesify them?
        if self.record_mode:
            return
        """dafault path:
        /home/$USER/integral_data/cobotta_data/rosbag/{task_name}/{variation}/{num_episode}_{YYMMDD}.bag
        """
        now_str = datetime.now().strftime('%m%d%Y.%H%M%S.%f')
        bagfile_name = '{}_{}.bag'.format(self.save_episode_start_count, now_str)
        self.bagfile_path = osp.join(self.save_dir, bagfile_name)
        rospy.loginfo('start recording')
        self.record_mode = True
        topics = ['/front_camera/image_raw/compressed',
                  '/top_camera/image_raw/compressed',
                  '/joint_states']
        topics = ' '.join(topics)
        command = "rosbag record -O {} {} -q".format(self.bagfile_path, topics)
        command = shlex.split(command)
        self.rosbag_proc = subprocess.Popen(command)
    
    def stop_record(self):
        if not self.record_mode:
            return
        rospy.loginfo('stop recording')
        self.record_mode = False
        if self.rosbag_proc is not None:
            print("kill rosbag process")
            terminate_process_and_children(self.rosbag_proc)
            save = yes_no_input_with_message("Do you want to save this demonstration?")
            if save:
                rospy.loginfo("saving {} ...".format(self.bagfile_path))
                self.save_episode_start_count += 1
            else:
                rospy.logwarn("{} was not successful demonstration, removeing...".format(self.bagfile_path))
                command = "rm -rf {}".format(self.bagfile_path)
                command = shlex.split(command)
                subprocess.Popen(command)

    def update_info(self):
        """Update information on Rviz
        """
        m = self.m
        if m.is_on_init():
            self._publish_info(TEXT_WELCOME, 'fa-angellist', 'blue')
        if m.is_stop():
            self._publish_info(TEXT_STOP_MOVE, 'fa-stop', 'red')
        if m.is_move():
            self._publish_info(TEXT_START_MOVE, 'fa-play', 'blue')
        if m.is_move_to_initial_pose():
            self._publish_info(TEXT_RETURN_TO_INITIAL_POSE, 'fa-refresh', 'yellow')
        if m.is_move_with_record():
            self._publish_info(TEXT_MOVE_WITH_RECORD, 'fa-file-video', 'blue')
            
    def on_press(self, key):
        """Keyboard event. Make transition in self.m according to key inputs.
        """
        print("\n===================================")
        if key ==keyboard.Key.esc:            
            self.listener.stop()  # stop listener
        if key == keyboard.Key.space:
            self.target_ee_state = self.current_ee_state
        try:
            k = key.char
        except AttributeError:  # special key are pressed
            return True
        if k == 's':
            self.m.start_move()
        if k == 'h':
            self.m.stop_move()
        if k == 'r':
            self.m.start_record()
        if k == 'x':
            self.m.stop_record()
        if k == 'i':
            self.m.reset_move()
        self.update_info()
        self.execute_state()

    def joypad_cb(self, msg):
        """joypad event. Make transition in self.m according to key inputs.
        """
        a = list(msg.buttons)  # tuple ->list
        if a[4]:
            self.target_ee_state = self.current_ee_state
        if a[0]:
            self.m.start_move()
        if a[1]:
            self.m.stop_move()
        if a[6]:
            self.m.start_record()
        if a[7]:
            self.m.stop_record()
        if a[8]:
            self.m.reset_move()
        self.update_info()
        self.execute_state()

    def _publish_info(self, text, icon, color):
        """helper function to print information to screen
        """
        self.text_pub.publish(overlaytext_from_string(text, color=color))
        self.pict_pub.publish(create_pictogram_msg(icon_name=icon, color=color))

    def on_release(self, key):
        pass

    def on_shutdown(self):
        if self.use_keyboard:
            rospy.loginfo("Shutting down, stopping keyboard listener...")
            self.key_listener.stop()
        rospy.loginfo("Shutting down, returning to initial pose...")
        # self.return_to_initial_pose()
        
    def run(self):
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()


def main(argv):
    assert FLAGS.task_name is not None, "Please specify name of task (eg, push_button)"
    task_name = FLAGS.task_name
    variation = FLAGS.variation
    save_dir = FLAGS.save_dir
    """dafault path:
    /home/$USER/integral_data/cobotta_data/rosbag/{task_name}/{variation}/{num_episode}_{YYMMDD}.bag
    """
    if not save_dir:
        user = os.environ['USER']
        save_dir = osp.join('/home', user, 'integral_data', 'cobotta_data', 'rosbag', task_name, 'vatiatoin{}'.format(variation))
    check_and_make(save_dir)

    _, _, files = next(os.walk(save_dir))
    save_episode_start_count = len(files)

    cobotta_teleop = CobottaTeleop(save_dir=save_dir,
                                   save_episode_start_count=save_episode_start_count)
    cobotta_teleop.run()
         

if __name__ == '__main__':
    app.run(main)
 
