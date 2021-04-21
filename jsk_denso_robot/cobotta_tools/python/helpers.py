import rospy
import numpy as np

import quaternion

from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.msg import Pictogram

def pose_stamped_to_pos_rpy(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    q = np.quaternion(o.x, o.y, o.z, o.w)
    rpy = quaternion.as_euler_angles(q)
    pos = np.array([p.x, p.y, p.z])
    rpy = np.array([rpy[0], rpy[1], rpy[2]])
    return pos, rpy


def pos_rpy_to_pose_msg(pos, rpy):
    p = Point(pos[0], pos[1], pos[2])
    q = quaternion.from_euler_angles(rpy[0], rpy[1], rpy[2])
    ori = Quaternion(q.w, q.x, q.y, q.z)
    return Pose(position=p, orientation=ori)


def overlaytext_from_string(string, color='blue'):
    text = OverlayText()
    text.width = 1000
    text.height = 120
    text.left = 10
    text.top = 780
    text.text_size = 20
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    text.text = string
    text.fg_color = _chose_color(color)
    text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
    return text


def create_pictogram_msg(icon_name='fa-stop', color='red'):
    msg = Pictogram()
    msg.action = Pictogram.ROTATE_X
    msg.header.frame_id = 'world'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.z = 0.7
    msg.pose.orientation.w = 0.7
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = -0.7
    msg.pose.orientation.z = 0
    msg.mode = Pictogram.PICTOGRAM_MODE
    msg.speed = 0.5
    msg.size = 0.3
    msg.color = _chose_color(color)

    msg.character = icon_name
    return msg


def pose_from_transform(trans):
    pos = Point(trans.translation.x, trans.translation.y, trans.translation.z)
    ori = Quaternion(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w)
    return Pose(position=pos, orientation=ori)


def _chose_color(color):
    if color == 'blue':
       return  ColorRGBA(25/255.0, 1.0, 240.0/255.0, 1.0)
    elif color == 'red':
        return ColorRGBA(255/ 255.0, 20/255.0, 20/255.0, 1.0)
    elif color == 'green':
        return ColorRGBA(20/ 255.0, 255/255.0, 20/255.0, 1.0)
    elif color == 'yellow':
        return ColorRGBA(255/ 255.0, 255/255.0, 20/255.0, 1.0) 
    else:  # white
        return ColorRGBA(255/ 255.0, 255/255.0, 255/255.0, 1.0)
