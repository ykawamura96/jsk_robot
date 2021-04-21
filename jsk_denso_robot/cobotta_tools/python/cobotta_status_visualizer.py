#!/usr/bin/env python
import rospy
import tf2_ros

from math import pi
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

def rad2deg(rad):
    return  rad * 360 / 2 / pi

class CobottaStatusVizualizer(object):
    def __init__(self):
        rospy.init_node('cobotta_status_visualizer', anonymous=True)
        self.register_callbacks()
        self.register_publishers()

    def register_callbacks(self):
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def register_publishers(self):
        self.jpos_pubs = [rospy.Publisher('/cobotta_status/joint_states/pos/{}'.format(i),
                                          Float32,
                                          queue_size=10)
                          for i in range(1,7)]
        self.jvel_pubs = [rospy.Publisher('/cobotta_status/joint_states/vel/{}'.format(i),
                                          Float32,
                                          queue_size=10)
                          for i in range(1,7)]
        self.ee_pos = [rospy.Publisher('/cobotta_status/ee_pos/{}'.format(i),
                                       Float32,
                                       queue_size=10)
                       for i in ['x', 'y', 'z']]
        self.ee_rot = [rospy.Publisher('/cobotta_status/ee_pos/{}'.format(i),
                                       Float32,
                                       queue_size=10)
                       for i in ['roll', 'pitch', 'yaw']]
        
    def joint_callback(self, msg):
        for i, j_pos in enumerate(msg.position):
            self.jpos_pubs[i].publish(rad2deg(j_pos))
            if i == 5:
                break
        for i, j_vel in enumerate(msg.velocity):            
            self.jvel_pubs[i].publish(j_vel)
            if i == 5:
                break

        try:
            t = self.tfBuffer.lookup_transform('world','J6',rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        try:
            for i, p in enumerate([t.transform.translation.x,
                                   t.transform.translation.y,
                                   t.transform.translation.z]):
                self.ee_pos[i].publish(p)
            rpy = euler_from_quaternion((t.transform.rotation.x,
                                         t.transform.rotation.y,
                                         t.transform.rotation.z,
                                         t.transform.rotation.w))
            for i, p in enumerate(['roll', 'pitch', 'yaw']):
                self.ee_rot[i].publish(rpy[i])
        except:
            pass
            
            
    def run(self):
        rospy.spin()

        
def main():
    csv = CobottaStatusVizualizer()
    csv.run()

if __name__ == '__main__':
    main()
        

