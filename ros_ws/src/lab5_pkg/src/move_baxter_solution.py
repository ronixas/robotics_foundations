#!/usr/bin/env python

# ROS Python API
import rospy
import sys
import copy
import struct

# Baxter SDK that provides high-level functions to control Baxter
import baxter_interface
from baxter_interface import CHECK_VERSION

from lab5_pkg.srv import *

#The Header message comprise a timestamp, message sequence id and frame id where the message originited from (see http://docs.ros.org/api/std_msgs/html/msg/Header.html)
from std_msgs.msg import Header

# Pose is the defacto message to store a 3D point and a quaternion, head ove ROS API to find out the structure!
# PoseStamped consists of a Header and a Pose messages. 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

# Baxter ROS messages and services that allows to communicate with the robot. The Inverse Kinematic function runs inside the robot and it is closed-source. To access it, the developers have made available the SolvePositionIK service; hence we need to import it, and also, its "request" message (see: https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_core_msgs/srv/SolvePositionIK.srv)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self.alternateX = 1
        self.alternateY = -1

    def move_to_start(self, start_angles=None):
        
        print("Moving the {0} arm...".format(self._limb_name))
        if start_angles is None:
            # Starting Joint angles for left arm
            start_angles = {'left_w0': 0.00659044387543,
                                     'left_w1': 1.25747370743,
                                     'left_w2': -0.00505950146978,
                                     'left_e0': -0.0120000614243,
                                     'left_e1': 0.76406783177,
                                     'left_s0': -0.000809555189524,
                                     'left_s1': -0.55002984575}
            
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.loginfo("Done!")
        rospy.sleep(0.5)
        
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()
        
    def pose_callback(self, pose_msg):
        # An orientation for gripper fingers to be overhead and parallel to the obj
        overhead_orientation = Quaternion(
                                 x=-0.0249590815779,
                                 y=0.999649402929,
                                 z=0.00737916180073,
                                 w=0.00486450832011)
        beforepicking_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
        
        # Move Baxter to a pre-picking position
        self.move_to_start(beforepicking_joint_angles)
        
        # Define the block pose. You have to move 2cm down in order to pick it as the centroid is on the surface of the block
        block_pose_pick = Pose(
        position=Point(x=pose_msg.pose.position.x, y=pose_msg.pose.position.y, z=pose_msg.pose.position.z - 0.02),
        orientation=overhead_orientation)
        # Define the placing pose, 5 cm to the right or left (self.alternate sets the direction)
        block_pose_place = Pose(
        position=Point(x=pose_msg.pose.position.x + (self.alternateX * 0.05), y=pose_msg.pose.position.y + (self.alternateY * 0.05), z=pose_msg.pose.position.z - 0.02),
        orientation=overhead_orientation)
        
        self.alternateX *= -1
        self.alternateY *= -1
        
        # Execute pick and place
        print("\nPicking...")
        self.pick(block_pose_pick)
        print("\nPlacing...")
        self.place(block_pose_place)
        # Move the arm away from the camera field of view
        self.move_to_start()
        
        # *************** SOLUTION
        self.call_capture_srv()
        
        
    def call_capture_srv(self):
        # Capture image
        rospy.loginfo("Wating for image capture service")
        rospy.wait_for_service('/lab5/capture')
        rospy.loginfo("Done!")
        try:
            capture_srv = rospy.ServiceProxy('/lab5/capture', capture)
            capture_srv(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        rospy.loginfo("Capture completed!")
        # ***************

def main():
    rospy.init_node("pnp_lab5")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.10 # meters
    pnp = PickAndPlace(limb, hover_distance)
    
    # Subscriber
    rospy.Subscriber('/lab5/pose', PoseStamped, pnp.pose_callback, queue_size=1)
    
    # Move to the desired starting angles
    pnp.move_to_start()
    
    # *************** SOLUTION
    pnp.call_capture_srv()
    # ***************
    
    rospy.loginfo("Node ready!")
    rospy.spin()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())




