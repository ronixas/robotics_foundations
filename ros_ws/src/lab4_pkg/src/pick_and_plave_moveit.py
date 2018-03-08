#!/usr/bin/env python

import argparse
import struct
import sys
import copy

import rospy
import rospkg

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
        #self._rs.enable()
        
        # *************** SOLUTION
        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        #We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")
        self._group.set_max_velocity_scaling_factor(0.2) # This is to make Baxter move slower
        # ***************

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        
        # *************** SOLUTION
#         if not start_angles:
#             start_angles = dict(zip(self._joint_names, [0]*7))
#         self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
        # ***************

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
        
        # *************** SOLUTION
        #joint_angles = self.ik_request(approach)
        #self._guarded_move_to_joint_position(joint_angles)
        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)
        # ***************

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
        
        # *************** SOLUTION
        #joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        #self._guarded_move_to_joint_position(joint_angles)
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)
        # ***************

    def _servo_to_pose(self, pose):
        # servo down to release
        #joint_angles = self.ik_request(pose)
        #self._guarded_move_to_joint_position(joint_angles)
        # *************** SOLUTION
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)
        # ***************

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

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    # *************** SOLUTION
    moveit_commander.roscpp_initialize(sys.argv)
    # ***************
    
    rospy.init_node("ik_pick_and_place_moveit")
    
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    #starting_joint_angles = {'left_w0': 0.6699952259595108,
#                              'left_w1': 1.030009435085784,
#                              'left_w2': -0.4999997247485215,
#                              'left_e0': -1.189968899785275,
#                              'left_e1': 1.9400238130755056,
#                              'left_s0': -0.08000397926829805,
#                              'left_s1': -0.9999781166910306}
    
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    # *************** SOLUTION
    starting_pose = Pose(
        position=Point(x=0.7, y=0.15, z=1.2),
        orientation=overhead_orientation)
    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    # ***************
    
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.7, y=0.15, z=0.801),
        orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.75, y=0.0, z=0.801),
        orientation=overhead_orientation))
    
    # Move to the desired starting angles
    # *************** SOLUTION
    # pnp.move_to_start(starting_joint_angles)
    pnp.move_to_start(starting_pose)
    # ***************
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        pnp.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())