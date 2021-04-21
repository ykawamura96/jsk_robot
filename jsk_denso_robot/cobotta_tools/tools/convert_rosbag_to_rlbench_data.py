#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Convert rosbag file to RLBench style data file.
"""
import os
import cv2
import pickle

import numpy as np
import os.path as osp
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import glob
from progress.bar import Bar

from absl import app
from absl import flags
from rlbench.backend.observation import Observation
from rlbench.demo import Demo
try:
    import rosbag
except ImportError:
    print("run 'pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag'")

FLAGS = flags.FLAGS

flags.DEFINE_string('task_name',
                    None,
                    'Name of task')
flags.DEFINE_integer('variation',
                    0,
                    'Variation of task')


def check_and_make(*dirs):
    for dir in list(dirs):
        if not os.path.exists(dir):
            os.makedirs(dir)

class RosbagConverter(object):

    def __init__(self, rosbag_dir=None, dest_dir=None, use_center_only=False,
                 resize_width=128, resize_height=128):
        self.rosbag_dir = rosbag_dir
        self.dest_dir = dest_dir
        self.use_center_only = use_center_only
        self.frame_resize_shape = (resize_width, resize_height)
        self.bridge = CvBridge()
        if self.rosbag_dir is None:
            self.rosbag_dir = '/tmp/rosbags/'
        if self.dest_dir is None:
            self.dest_dir = '/tmp/rlbench_like_data/'
        check_and_make(self.rosbag_dir, self.dest_dir)

    def read_and_save_one_bagfile(self, bag_filepath, dest_episode_dir):
        # TODO fixed time step is no good
        # create img dir
        img_dir = osp.join(dest_episode_dir, 'front_rgb')
        check_and_make(img_dir)

        # setup vartiables
        joint_count = 0
        img_count = 0
        save_joint = False
        observations = []
        with rosbag.Bag(bag_filepath, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == '/front_camera/image_raw/compressed':
                    save_file_path = osp.join(img_dir, '{}.png'.format(img_count))
                    self._process_and_save(msg, save_file_path)
                    img_count += 1
                    save_joint =True
                if topic == '/joint_states' and save_joint:
                        joint_count += 1
                        save_joint = False
                        # gripper state are determied by effor from gripper
                        # TODO fix hard-coding in threshold force
                        gripper_open = 1.0 if msg.effort[-1] >= 0.1 else 0.0
                        observations.append(
                            Observation(
                                left_shoulder_rgb=None,
                                left_shoulder_depth=None,
                                left_shoulder_mask=None,
                                right_shoulder_rgb=None,
                                right_shoulder_depth=None,
                                right_shoulder_mask=None,
                                wrist_rgb=None,
                                wrist_depth=None,
                                wrist_mask=None,
                                front_rgb=None,
                                front_depth=None,
                                front_mask=None,
                                joint_velocities=np.array(msg.velocity[:6]),
                                joint_positions=np.array(msg.position[:6]),
                                joint_forces=np.array(msg.effort[:6]),
                                gripper_open=gripper_open,
                                gripper_pose=None,
                                gripper_joint_positions=np.array([msg.position[-1]/2.0, msg.position[-1]/2.0]),
                                gripper_touch_forces=None,
                                task_low_dim_state=None))
            if img_count != joint_count:
                diff = img_count - joint_count
                last_obs = observations[-1]
                observations += [last_obs] * diff
            assert img_count == len(observations), "Number of img data {} and joint data {} needs to match".format(img_count, joint_count)
            demo = Demo(observations=observations)
            with open(osp.join(dest_episode_dir, 'low_dim_obs.pkl'), 'wb') as f:
                pickle.dump(demo, f)

    def _process_and_save(self, msg, path):
        # decompress image
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        # extract center if use_center_only ,and make the image shpae square, then resize
        w, h = cv_img.shape[0], cv_img.shape[1]
        if self.use_center_only:
            cv_img = cv_img[0:w, int((h-w)/2):int((h+w)/2)]
        cv_img = cv2.resize(cv_img, self.frame_resize_shape, interpolation = cv2.INTER_AREA)
        cv2.imwrite(path, cv_img)

    def run(self):
        rosbag_path_list = glob.glob(osp.join(self.rosbag_dir, '*.bag'))
        i = 0
        total = len(rosbag_path_list)
        print("Saving {} episodes from {} to {}".format(total, self.rosbag_dir, self.dest_dir))
        with Bar('Processing', max=total, suffix='%(index)d/%(max)depisodes, eta: %(eta).1fs') as bar:
            for i, rosbag_path in enumerate(rosbag_path_list):
                dest_episode_dir = osp.join(self.dest_dir, 'episodes/episodes{}'.format(i))
                self.read_and_save_one_bagfile(rosbag_path, dest_episode_dir)
                bar.next()


def main(argv):
    task_name = FLAGS.task_name
    if not task_name.startswith('cobotta'):
        task_name = 'cobotta_' + task_name
    variation = FLAGS.variation
    user = os.environ.get('USER')
    dest_episode_dir = '/home/{}/integral_data/rlbench_data/{}/default/variation{}/'.format(user, task_name, variation)
    if osp.exists(dest_episode_dir):
        print("ERROR: dest path already exists!!!\n Check original data")
        return
    rosbag_converter = RosbagConverter(rosbag_dir='/tmp/rosbags/', dest_dir=dest_episode_dir)
    rosbag_converter.run()


if __name__ == '__main__':
    app.run(main)


