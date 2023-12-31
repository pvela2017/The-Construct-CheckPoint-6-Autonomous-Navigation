#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import time
import rclpy

import math
from geometry_msgs.msg import Quaternion

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_apps.srv import GoToLoading
from std_msgs.msg import Empty


positions_sim = {
    "initial": [0.0, 0.0, 0.0],
    "loading": [5.40551, -0.315346, -1.57],
    "shipping": [0.985014, -2.9352, 1.57]}

positions = {
    "initial": [0.0, 0.0, 0.0],
    "loading": [4.45418, -1.11213, -1.57],
    "shipping": [0.236652, -2.92636, 1.57]}


def rpy_to_quaternion(yaw):
    # Set RPY angles
    quaternion = Quaternion()
    
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(0 * 0.5)
    sp = math.sin(0 * 0.5)
    cr = math.cos(0 * 0.5)
    sr = math.sin(0 * 0.5)

    quaternion.w = cr * cp * cy + sr * sp * sy
    quaternion.x = sr * cp * cy - cr * sp * sy
    quaternion.y = cr * sp * cy + sr * cp * sy
    quaternion.z = cr * cp * sy - sr * sp * cy
    return quaternion

def main():
    rclpy.init()
    node = rclpy.create_node('nav_node')
    
    #quaternion_result = rpy_to_quaternion(positions['shipping'][2])
    #node.get_logger().info(f"Quaternion result: {quaternion_result}")

    # Publisher to unload
    publisher = node.create_publisher(Empty, 'elevator_down', 10)
    publisher_cmd = node.create_publisher(Twist, '/robot/cmd_vel', 10)
    twist_msg = Twist()

    # Start the client
    client = node.create_client(GoToLoading, 'approach_shelf')

    # Wait for the service to be available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = positions['initial'][0]
    initial_pose.pose.position.y = positions['initial'][1]
    initial_pose.pose.orientation = rpy_to_quaternion(positions['initial'][2])
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Set loading pose
    loading_pose = PoseStamped()
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = navigator.get_clock().now().to_msg()
    loading_pose.pose.position.x = positions['loading'][0]
    loading_pose.pose.position.y = positions['loading'][1]
    loading_pose.pose.orientation = rpy_to_quaternion(positions['loading'][2])
    print('Going to loading pose')
    navigator.goToPose(loading_pose)

    
    while not navigator.isTaskComplete():
        pass
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Arrived to loading pose')
        # Call service
        # Create a request
        request = GoToLoading.Request()
        request.attach_to_shelf = True

        # Send the request and wait for a response
        future = client.call_async(request)

        # Wait for the response to be available
        rclpy.spin_until_future_complete(node, future)

        # Check if the service call was successful
        if future.result() is not None:
            response = future.result()
            node.get_logger().info('Result: %s' % response.complete)
            if not response.complete:
                exit(-1)
        else:
            node.get_logger().error('Service call failed!')
            exit(-1)

        # Go back and twist
        twist_msg.linear.x = -0.2 
        twist_msg.angular.z = 0.0  

        start_time = time.time()
        while time.time() - start_time < 6.0:  # Publish for 4 seconds
            publisher_cmd.publish(twist_msg)

        # Stop the robot after 4 seconds
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        publisher_cmd.publish(twist_msg)

        # Go back and twist
        twist_msg.linear.x = 0.0 
        twist_msg.angular.z = -0.2  

        start_time = time.time()
        while time.time() - start_time < 6.0:  # Publish for 5 seconds
            publisher_cmd.publish(twist_msg)

        # Stop the robot after 4 seconds
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        publisher_cmd.publish(twist_msg)

        # Move to shipping
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = positions['shipping'][0]
        shipping_destination.pose.position.y = positions['shipping'][1]
        shipping_destination.pose.orientation = rpy_to_quaternion(positions['shipping'][2])
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print('Going to loading was canceled. ')
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Going to loading failed!')
        exit(-1)


    while not navigator.isTaskComplete():
        pass
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print('Arrived to shipping pose')
        # Unload
        # Create an empty message
        msg = Empty()
        # Publish the message
        publisher.publish(msg)

        # Get out under the platform
        twist_msg.linear.x = 0.2 
        twist_msg.angular.z = 0.0  

        start_time = time.time()
        while time.time() - start_time < 4.0:  # Publish for 4 seconds
            publisher_cmd.publish(twist_msg)

        # Stop the robot after 4 seconds
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        publisher_cmd.publish(twist_msg)

        # Move to initial pose
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = positions['initial'][0]
        shipping_destination.pose.position.y = positions['initial'][1]
        shipping_destination.pose.orientation = rpy_to_quaternion(positions['initial'][2])
        navigator.goToPose(shipping_destination)

    elif result == TaskResult.CANCELED:
        print('Going to shipping was canceled. ')
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Going to shipping failed!')
        exit(-1)


    while not navigator.isTaskComplete():
        pass
    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Arrived to initial pose')

    elif result == TaskResult.CANCELED:
        print('Going to initial was canceled. ')
        exit(-1)

    elif result == TaskResult.FAILED:
        print('Going to initial failed!')
        exit(-1)

    exit(0)


if __name__ == '__main__':
    main()