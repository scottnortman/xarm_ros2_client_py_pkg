#!/usr/bin/env python3
'''
File:   xarm_ros2_client_py.py
Desc:   Python implementation of the c++ version of the xarm path planning client
Auth:   scott@restfulrobotics.com
Date:   Aug 28 2021
Note:

See https://github.com/xArm-Developer/xarm_ros2

This is the c++ version that this is based on
https://github.com/xArm-Developer/xarm_ros2/blob/master/xarm_planner/test/test_xarm_planner_client_pose.cpp

To build

$ colcon build --packages-select xarm_ros2_client_py_pkg --symlink-install

Example

In first terminal, launch a simulated xarm (6 dof shown)
$ ros2 launch xarm_planner xarm6_planner_fake.launch.py add_gripper:=true

In a second terminal, execute this node
$ ros2 run xarm_ros2_client_py_pkg xarm_ros2_client_py

The simulated xarm in rviz should cycle through planning and pose execution

'''

from numpy.matrixlib.defmatrix import matrix
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from xarm_msgs.srv import PlanPose  # send Pose target, get bool success
from xarm_msgs.srv import PlanPoses # send array of Poses targets, get bool success
from xarm_msgs.srv import PlanJoint # send float64[] target, get bool success
from xarm_msgs.srv import PlanExec  # send bool wait, get bool success

import sys
sys.path.append('/home/snortman/code/facetest')
from CartesianTrajectories import CartesianTrajectory

import numpy as np
from pyquaternion import Quaternion

class XArmRos2ClientPy( Node ):

    def __init__(self, args):

        super().__init__('xarm_ros2_client_node')

        self.plan_joint_srv_name = 'xarm_joint_plan'
        self.plan_pose_srv_name = 'xarm_pose_plan'
        self.exec_plan_srv_name = 'xarm_exec_plan'
        self.plan_poses_srv_name = 'xarm_poses_plan'

        self.cart_traj = CartesianTrajectory()

        # todo: get DOF from config file or parameter?
        self.dof = 7

        # see xarm example code 
        self.tar_joint1 = {}
        self.tar_joint1[5] = [1.570796, -1.570796, -1.047198, 2.792527, -1.570796]
        self.tar_joint1[6] = [1.570796, -1.570796, -1.047198, 2.967060, 2.792527, -3.124139]
        self.tar_joint1[7] = [1.570796, -1.570796, -1.570796, 1.396263, 2.967060, 2.792527, -1.570796]

        self.tar_joint2 = {}
        self.tar_joint2[5] = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.tar_joint2[6] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tar_joint2[7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.tar_joint3 = {}
        self.tar_joint3[5] = [-1.570796, -1.570796, -1.047198, -0.349066, 2.617994]
        self.tar_joint3[6] = [-1.570796, -1.570796, -1.047198, -2.967060, -0.349066, 3.124139]
        self.tar_joint3[7] = [-1.570796, -1.570796, 1.570796, 1.396263, -2.967060, -0.349066, 2.617994]

        self.tar_joints = [self.tar_joint1, self.tar_joint2, self.tar_joint3]


        # Target poses, matches examples from xarm example code
        self.target_pose1 = Pose()
        self.target_pose1.position.x = 0.3
        self.target_pose1.position.y = -0.1
        self.target_pose1.position.z = 0.2
        self.target_pose1.orientation.x = 1.0
        self.target_pose1.orientation.y = 0.0
        self.target_pose1.orientation.z = 0.0
        self.target_pose1.orientation.w = 0.0
        base_tx_flange_pose1 = np.identity(4)
        base_tx_flange_pose1[0,3] = self.target_pose1.position.x
        base_tx_flange_pose1[1,3] = self.target_pose1.position.y
        base_tx_flange_pose1[2,3] = self.target_pose1.position.z
        q = Quaternion(w=self.target_pose1.orientation.w, x=self.target_pose1.orientation.x, 
            y=self.target_pose1.orientation.y, z=self.target_pose1.orientation.z)
        base_tx_flange_pose1[0:3,0:3] = q.rotation_matrix

        self.target_pose2 = Pose()
        self.target_pose2.position.x = 0.3
        self.target_pose2.position.y = 0.1
        self.target_pose2.position.z = 0.2
        self.target_pose2.orientation.x = 1.0
        self.target_pose2.orientation.y = 0.0
        self.target_pose2.orientation.z = 0.0
        self.target_pose2.orientation.w = 0.0
        base_tx_flange_pose2 = np.identity(4)
        base_tx_flange_pose2[0,3] = self.target_pose2.position.x
        base_tx_flange_pose2[1,3] = self.target_pose2.position.y
        base_tx_flange_pose2[2,3] = self.target_pose2.position.z
        q = Quaternion(w=self.target_pose2.orientation.w, x=self.target_pose2.orientation.x, 
            y=self.target_pose2.orientation.y, z=self.target_pose2.orientation.z)
        base_tx_flange_pose2[0:3,0:3] = q.rotation_matrix


        self.target_pose3 = Pose()
        self.target_pose3.position.x = 0.3
        self.target_pose3.position.y = 0.1
        self.target_pose3.position.z = 0.4
        self.target_pose3.orientation.x = 1.0
        self.target_pose3.orientation.y = 0.0
        self.target_pose3.orientation.z = 0.0
        self.target_pose3.orientation.w = 0.0
        base_tx_flange_pose3 = np.identity(4)
        base_tx_flange_pose3[0,3] = self.target_pose3.position.x
        base_tx_flange_pose3[1,3] = self.target_pose3.position.y
        base_tx_flange_pose3[2,3] = self.target_pose3.position.z
        q = Quaternion(w=self.target_pose3.orientation.w, x=self.target_pose3.orientation.x, 
            y=self.target_pose3.orientation.y, z=self.target_pose3.orientation.z)
        base_tx_flange_pose3[0:3,0:3] = q.rotation_matrix


        self.target_pose4 = Pose()
        self.target_pose4.position.x = 0.3
        self.target_pose4.position.y = -0.1
        self.target_pose4.position.z = 0.4
        self.target_pose4.orientation.x = 1.0
        self.target_pose4.orientation.y = 0.0
        self.target_pose4.orientation.z = 0.0
        self.target_pose4.orientation.w = 0.0
        base_tx_flange_pose4 = np.identity(4)
        base_tx_flange_pose4[0,3] = self.target_pose4.position.x
        base_tx_flange_pose4[1,3] = self.target_pose4.position.y
        base_tx_flange_pose4[2,3] = self.target_pose4.position.z
        q = Quaternion(w=self.target_pose4.orientation.w, x=self.target_pose4.orientation.x, 
            y=self.target_pose4.orientation.y, z=self.target_pose4.orientation.z)
        base_tx_flange_pose4[0:3,0:3] = q.rotation_matrix

        self.target_poses = [self.target_pose1, self.target_pose2, self.target_pose3, self.target_pose4]

        # Create smooth cartesian trajectories for testing
        # todo: better way to calculate number of steps
        # go to pose2,4,1,3
        nsteps = 2
        base_tx_flange_segment1 = self.cart_traj.cart_trans_slerp( base_tx_flange_pose2, base_tx_flange_pose4, nsteps )
        base_tx_flange_segment2 = self.cart_traj.cart_trans_slerp( base_tx_flange_pose4, base_tx_flange_pose1, nsteps )
        base_tx_flange_segment3 = self.cart_traj.cart_trans_slerp( base_tx_flange_pose1, base_tx_flange_pose3, nsteps )
        base_tx_flange_segment4 = self.cart_traj.cart_trans_slerp( base_tx_flange_pose3, base_tx_flange_pose2, nsteps )

        # combine into one array
        txs = base_tx_flange_segment1 + base_tx_flange_segment2 + base_tx_flange_segment3 + base_tx_flange_segment4

        # Convert all to type Pose
        self.base_poses_flange = []
        for ii in range(0,len(txs)):
            tx = txs[ii]
            R = tx[0:3,0:3]
            q = Quaternion(matrix=R)
            t = tx[0:3,3]
            p = Pose()
            p.position.x = t[0]
            p.position.y = t[1]
            p.position.z = t[2]
            p.orientation.w = q.w
            p.orientation.x = q.x
            p.orientation.y = q.y
            p.orientation.z = q.z
            self.base_poses_flange.append(p)

        self.plan_poses_req = PlanPoses.Request()

        self.future = None

        # create clients to call services
        # /xarm_pose_plan
        # /xarm_joint_plan
        # /xarm_exec_plan

        self.plan_joint_req = PlanJoint.Request()
        self.plan_joint_client = self.create_client( PlanJoint, self.plan_joint_srv_name)
        while not self.plan_joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service [{self.plan_joint_srv_name}] not ready; waiting...')
        else:
            self.get_logger().info(f'Service [{self.plan_joint_srv_name}] ready...')
        
        self.plan_pose_req = PlanPose.Request()
        self.plan_pose_client = self.create_client( PlanPose, self.plan_pose_srv_name )
        while not self.plan_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service [{self.plan_pose_srv_name}] not ready; waiting...')
        else:
            self.get_logger().info(f'Service [{self.plan_pose_srv_name}] ready...')

        self.plan_poses_req = PlanPoses.Request()
        self.plan_poses_client = self.create_client( PlanPoses, self.plan_poses_srv_name)
        while not self.plan_poses_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service [{self.plan_poses_srv_name}] not ready; waiting...')
        else:
            self.get_logger().info(f'Service [{self.plan_poses_srv_name}] ready...')

        self.exec_plan_req = PlanExec.Request()
        self.exec_plan_client = self.create_client( PlanExec, self.exec_plan_srv_name )
        while not self.exec_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service [{self.exec_plan_srv_name}] not ready; waiting...')
        else:
            self.get_logger().info(f'Service [{self.exec_plan_srv_name}] ready...')


    def execPlan( self, wait=True ):
        self.get_logger().info(f'calling exec plan')
        self.exec_plan_req.wait = wait
        self.future = self.exec_plan_client.call_async(self.exec_plan_req)
        rclpy.spin_until_future_complete( self, self.future )
        success = self.future.result().success
        self.get_logger().info(f'Service returned: [{success}]')
        return success

    def planJointTarget( self, joint_target):
        self.get_logger().info(f'Calling plan joint')
        self.plan_joint_req.target = joint_target
        self.future = self.plan_joint_client.call_async(self.plan_joint_req)
        rclpy.spin_until_future_complete( self, self.future )
        success = self.future.result().success
        self.get_logger().info(f'Service returned: [{success}]')
        return success

    def planPoseTarget( self, pose_target ):
        self.get_logger().info(f'Calling plan pose')
        self.plan_pose_req.target = pose_target
        self.future = self.plan_pose_client.call_async(self.plan_pose_req)
        rclpy.spin_until_future_complete( self, self.future )
        success = self.future.result().success
        self.get_logger().info(f'Service returned: [{success}]')
        return success

    def planPosesTarget( self, poses_target ):
        self.get_logger().info(f'Calling plan poses')
        self.plan_poses_req.targets = poses_target
        self.future = self.plan_poses_client.call_async(self.plan_poses_req)
        rclpy.spin_until_future_complete( self, self.future )
        success = self.future.result().success
        self.get_logger().info(f'Service returned: [{success}]')
        return success
    

def main(args=None):

    rclpy.init(args=args)
    xarm_ros2_client_node = XArmRos2ClientPy(args=args)
    xarm_ros2_client_node.get_logger().info('Created XArmRos2ClientNode...')

    while(rclpy.ok()):

        # Joint space motions
        #if True:
        if False:
            for targets in xarm_ros2_client_node.tar_joints:
                if xarm_ros2_client_node.planJointTarget( targets[xarm_ros2_client_node.dof] ):
                    xarm_ros2_client_node.execPlan()
                else:
                    xarm_ros2_client_node.get_logger().info(f'Plan joint failed for: [{targets[xarm_ros2_client_node.dof]}]')

        #if True:
        if False:
            # Restore to 0 
            if xarm_ros2_client_node.planJointTarget(xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof] ):
                xarm_ros2_client_node.execPlan()
            else:
                xarm_ros2_client_node.get_logger().info(f'Plan failed for: [{xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof]}]')

        #if True:
        if False:
            #task space motions
            for pose in xarm_ros2_client_node.target_poses:
                if xarm_ros2_client_node.planPoseTarget( pose ):
                    xarm_ros2_client_node.execPlan()
                else:
                    xarm_ros2_client_node.get_logger().info(f'Plan failed for: [{pose}]')

        #if True:
        if False:    
            # Restore to 0 
            if xarm_ros2_client_node.planJointTarget(xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof] ):
                xarm_ros2_client_node.execPlan()
            else:
                xarm_ros2_client_node.get_logger().info(f'Plan failed for: [{xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof]}]')

        if True:
            # Multi pose motions
            if xarm_ros2_client_node.planPosesTarget( xarm_ros2_client_node.base_poses_flange ):
                xarm_ros2_client_node.execPlan()
            else:
                xarm_ros2_client_node.get_logger().info('Plan failed for base_poses_flange...')

        

    xarm_ros2_client_node.get_logger().info('Shutting down XArmRos2ClientNode...')
    xarm_ros2_client_node.destroy_node()

if __name__ == '__main__':
    main()
