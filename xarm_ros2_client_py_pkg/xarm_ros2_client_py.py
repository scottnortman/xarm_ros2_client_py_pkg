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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from xarm_msgs.srv import PlanPose  # send Pose target, get bool success
from xarm_msgs.srv import PlanJoint # send float64[] target, get bool success
from xarm_msgs.srv import PlanExec  # send bool wait, get bool success

class XArmRos2ClientPy( Node ):

    def __init__(self, args):

        super().__init__('xarm_ros2_client_node')

        self.plan_joint_srv_name = 'xarm_joint_plan'
        self.plan_pose_srv_name = 'xarm_pose_plan'
        self.exec_plan_srv_name = 'xarm_exec_plan'

        # todo: get DOF from config file or parameter?
        self.dof = 6

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

        self.target_pose2 = Pose()
        self.target_pose2.position.x = 0.3
        self.target_pose2.position.y = 0.1
        self.target_pose2.position.z = 0.2
        self.target_pose2.orientation.x = 1.0
        self.target_pose2.orientation.y = 0.0
        self.target_pose2.orientation.z = 0.0
        self.target_pose2.orientation.w = 0.0

        self.target_pose3 = Pose()
        self.target_pose3.position.x = 0.3
        self.target_pose3.position.y = 0.1
        self.target_pose3.position.z = 0.4
        self.target_pose3.orientation.x = 1.0
        self.target_pose3.orientation.y = 0.0
        self.target_pose3.orientation.z = 0.0
        self.target_pose3.orientation.w = 0.0

        self.target_pose4 = Pose()
        self.target_pose4.position.x = 0.3
        self.target_pose4.position.y = -0.1
        self.target_pose4.position.z = 0.4
        self.target_pose4.orientation.x = 1.0
        self.target_pose4.orientation.y = 0.0
        self.target_pose4.orientation.z = 0.0
        self.target_pose4.orientation.w = 0.0

        self.target_poses = [self.target_pose1, self.target_pose2, self.target_pose3, self.target_pose4]


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
    

def main(args=None):

    rclpy.init(args=args)
    xarm_ros2_client_node = XArmRos2ClientPy(args=args)
    xarm_ros2_client_node.get_logger().info('Created XArmRos2ClientNode...')

    while(rclpy.ok()):

        # Joint space motions
        for targets in xarm_ros2_client_node.tar_joints:
            if xarm_ros2_client_node.planJointTarget( targets[xarm_ros2_client_node.dof] ):
                xarm_ros2_client_node.execPlan()
            else:
                xarm_ros2_client_node.get_logger().info(f'Plan joint failed for: [{targets[xarm_ros2_client_node.dof]}]')

        # Restore to 0 
        if xarm_ros2_client_node.planJointTarget(xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof] ):
            xarm_ros2_client_node.execPlan()
        else:
            xarm_ros2_client_node.get_logger().info(f'Plan failed for: [{xarm_ros2_client_node.tar_joints[1][xarm_ros2_client_node.dof]}]')

        #task space motions
        for pose in xarm_ros2_client_node.target_poses:
            if xarm_ros2_client_node.planPoseTarget( pose ):
                xarm_ros2_client_node.execPlan()
            else:
                xarm_ros2_client_node.get_logger().info(f'Plan failed for: [{pose}]')

    xarm_ros2_client_node.get_logger().info('Shutting down XArmRos2ClientNode...')
    xarm_ros2_client_node.destroy_node()

if __name__ == '__main__':
    main()
