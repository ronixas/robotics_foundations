#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg
import tf
import time

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import moveit_commander

# constants
noOfBlocks = 5
baseName = "block"

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
	# add listener
	self.listener = tf.TransformListener()
        # let listener buffer fill before using  
        time.sleep(1)
        
        self._robot = moveit_commander.RobotCommander()

        # used for motion planning and execution
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        self._group.set_max_velocity_scaling_factor(0.2) # This is to make Baxter move slower


    def move_to_start(self, start_angles=None):

        print("Moving the {0} arm to over the object position".format(self._limb_name))
        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)

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
        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)


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
        
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)

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


def main():

    # initialize moveit comander and a node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("assessed")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # decide on the limb (left or right)
    limb = 'left'
    hover_distance = 0.2 # meters

    # create the sorting object
    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # block starting positions (as retrieved by tf listener) will be held in a dictionary
    block_poses = dict()

    
    for objNo in range(0, noOfBlocks):
        try:
            # block name
            objName = baseName + str(objNo)

            # retrieve block transformation with help of a listener
            (trans,rot) = pnp.listener.lookupTransform('/world', '/' + objName, rospy.Time(0))

            # use the transforms to create poses
            start_position = Point(x=trans[0], y=trans[1], z=float(trans[2])+0.3) # 20cm above the object
            obj_position = Point(x=trans[0], y=trans[1], z=trans[2])
            orientation = Quaternion(x=rot[0], y=1, z=rot[2], w=0.00486450832011)

            # update the block positions
            block_poses[objName] = dict()
            block_poses[objName]["hover"] = Pose(position=start_position, orientation=orientation)
            block_poses[objName]["obj"] = Pose(position=obj_position, orientation=orientation)
 
        except Exception as e:
            print e
            return 1


    delta = 0     # target position offset counter
    # move block one by one
    for name in block_poses.keys():
       
       print "preparing to move {}".format(name)
       # move over the block
       pnp.move_to_start(block_poses[name]["hover"])

       print("\nPicking block {}". format(name))
       # implement picking 
       pnp.pick(block_poses[name]["obj"])

       print("\nPlacing block {}".format(name))
       # plan and execute position
       target_position = Point(x=-0.2, y=0.7+(0.1*delta), z=trans[2])
       target_orientation = Quaternion(x=0, y=1, z=0, w=0.00486450832011)
       target_pose = Pose(position=target_position, orientation=target_orientation)

       # plan and execute placing
       pnp.place(target_pose)
       
       # increment offset
       delta = delta + 1
    return 0

if __name__ == '__main__':
    sys.exit(main())
