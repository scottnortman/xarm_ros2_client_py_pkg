#!/usr/bin/env python3
'''
File:   xarm_ros2_client_py.py
Desc:   Python implementation of the c++ version of the xarm cartesian path client
Auth:   scott@restfulrobotics.com
Date:   Aug 28 2021
Note:

See https://github.com/xArm-Developer/xarm_ros2

This is the c++ version that this is based on
https://github.com/xArm-Developer/xarm_ros2/blob/master/xarm_planner/test/test_xarm_planner_client_pose.cpp


'''

import rclpy
from rclpy.node import Node

from pyquaternion import Quaternion

from std_msgs.msg import Bool
from xarm_msgs.srv import PlanPose
from xarm_msgs.srv import PlanJoint # send float64[] target, get bool success
from xarm_msgs.srv import PlanExec  # send bool wait, get bool success
from xarm_msgs.srv import PlanSingleStraight

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

    
        

        pass

    def execPlan( self, wait=True ):
        self.get_logger().info(f'calling exec plan')
        self.exec_plan_req.wait = wait
        self.future = self.exec_plan_client.call_async(self.exec_plan_req)
        rclpy.spin_until_future_complete( self, self.future )
        self.get_logger().info(f'Service returned: [{self.future.result().success}]')



    def planJointTarget( self, joint_target):
        self.get_logger().info(f'Calling plan joint')
        self.plan_joint_req.target = joint_target
        self.future = self.plan_joint_client.call_async(self.plan_joint_req)
        rclpy.spin_until_future_complete( self, self.future )
        self.get_logger().info(f'Service returned: [{self.future.result().success}]')



    def planPoseTarget( self, pose_target ):
        pass
    
    '''
    def step( self ):
        if self.future is not None:
            if self.future.done():
                self.get_logger().info(f'Service returned: [{self.future.result().success}]')
    '''

def main(args=None):

    rclpy.init(args=args)

    xarm_ros2_client_node = XArmRos2ClientPy(args=args)

    xarm_ros2_client_node.get_logger().info('Created XArmRos2ClientNode...')



    #rclpy.spin(xarm_ros2_client_node)
    while(rclpy.ok()):

        for targets in xarm_ros2_client_node.tar_joints:
            xarm_ros2_client_node.planJointTarget( targets[xarm_ros2_client_node.dof] )
            xarm_ros2_client_node.execPlan()

    xarm_ros2_client_node.get_logger().info('Shutting down XArmRos2ClientNode...')

    xarm_ros2_client_node.destroy_node()




if __name__ == '__main__':
    main()
