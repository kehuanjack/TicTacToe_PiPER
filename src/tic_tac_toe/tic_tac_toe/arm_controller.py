#!/usr/bin/env python3
import os
import math
import time
import yaml
import rclpy
from enum import Enum
from rclpy.node import Node
from std_srvs.srv import Empty
from project.srv import DrawSymbol
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

package_share_dir = get_package_share_directory('tic_tac_toe')
config_path = os.path.join(package_share_dir, 'config', 'board_params.yaml')

class ArmState(Enum):
    IDLE = 0
    MOVING = 1
    DRAWING = 2
    CALIBRATING = 3

class CalibrationStep(Enum):
    START = 0
    TOP_LEFT = 1
    TOP_RIGHT = 2
    BOTTOM_RIGHT = 3
    BOTTOM_LEFT = 4
    CLEAR_BUTTON = 5
    COMPLETE = 6

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Load parameters
        self.load_parameters()
        
        # Initialize state
        self.state = ArmState.IDLE
        self.calibration_step = CalibrationStep.START
        self.current_pose = Pose()

        self.orientation = [0.0, 1.0, 0.0, 0.0]  # Default orientation, pointing downward
        
        # Publisher - joint trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )

        # Create a reentrant callback group to allow service callbacks to call other services without blocking
        self.callback_group = ReentrantCallbackGroup()        

        # Service clients
        self.compute_ik_cli = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.callback_group)
        self.compute_cartesian_path_cli = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.callback_group)
        
        while not self.compute_ik_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting again...')
        
        while not self.compute_cartesian_path_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Cartesian path service not available, waiting again...')
        
        self.get_logger().info('Arm controller initialized')
        
        # Create services
        self.draw_grid_srv = self.create_service(Empty, 'draw_grid', self.draw_grid_callback, callback_group=self.callback_group)
        self.draw_symbol_srv = self.create_service(DrawSymbol, 'draw_symbol', self.draw_symbol_callback, callback_group=self.callback_group)
        self.press_button_srv = self.create_service(Empty, 'press_button', self.press_button_callback, callback_group=self.callback_group)
        self.move_home_srv = self.create_service(Empty, 'move_home', self.move_home_callback, callback_group=self.callback_group)
        self.update_params_srv = self.create_service(Empty, 'update_parameters', self.update_parameters_callback, callback_group=self.callback_group)
        self.start_calibration_srv = self.create_service(Empty, 'start_calibration', self.start_calibration_callback, callback_group=self.callback_group)
        self.next_calibration_step_srv = self.create_service(Empty, 'next_calibration_step', self.next_calibration_step_callback, callback_group=self.callback_group)
        self.cancel_calibration_srv = self.create_service(Empty, 'cancel_calibration', self.cancel_calibration_callback, callback_group=self.callback_group)
        self.finish_calibration_srv = self.create_service(Empty, 'finish_calibration', self.finish_calibration_callback, callback_group=self.callback_group)
        
        # Move to initial position
        self.move_to_home()
    
    def load_parameters(self):
        # Load configuration from parameter file
        try:
            with open(config_path, 'r') as file:
                self.params = yaml.safe_load(file)
                self.get_logger().info('Parameters loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load parameters: {e}')
            exit(1)
    
    def save_parameters(self):
        # Save parameters to file
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w') as file:
                yaml.dump(self.params, file)
            self.get_logger().info('Parameters saved successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save parameters: {e}')
            return False
    
    def update_parameter(self, key, value):
        # Update parameter
        keys = key.split('.')
        param_dict = self.params
        for k in keys[:-1]:
            param_dict = param_dict[k]
        param_dict[keys[-1]] = value
        return self.save_parameters()
    
    def is_moving(self):
        return self.state != ArmState.IDLE
    
    def cancel_current_motion(self):
        # Publish empty trajectory to cancel current motion
        empty_trajectory = JointTrajectory()
        self.trajectory_pub.publish(empty_trajectory)
        self.get_logger().info('Current motion cancelled')
    
    def apply_tcp_offset(self, pose):
        """Apply TCP offset to target pose, without using TF2"""
        try:
            # Get TCP offset from parameters (in meters)
            tool_offset = (
                self.params['tcp_offset']['x'],
                self.params['tcp_offset']['y'],
                self.params['tcp_offset']['z']
            )
            
            # Convert Pose to Euler angles (radians)
            rotation = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            
            # Get Euler angles (XYZ order)
            euler_angles = rotation.as_euler('xyz')
            rx_rad, ry_rad, rz_rad = euler_angles
            
            # Pre-calculate trigonometric values
            cx, sx = math.cos(rx_rad), math.sin(rx_rad)
            cy, sy = math.cos(ry_rad), math.sin(ry_rad)
            cz, sz = math.cos(rz_rad), math.sin(rz_rad)
            
            # Calculate rotation matrix (XYZ order: R = Rz * Ry * Rx)
            r00 = cy * cz
            r01 = sx * sy * cz - cx * sz
            r02 = cx * sy * cz + sx * sz
            
            r10 = cy * sz
            r11 = sx * sy * sz + cx * cz
            r12 = cx * sy * sz - sx * cz
            
            r20 = -sy
            r21 = sx * cy
            r22 = cx * cy
            
            # Apply rotation matrix to tool offset
            offset_x = r00 * tool_offset[0] + r01 * tool_offset[1] + r02 * tool_offset[2]
            offset_y = r10 * tool_offset[0] + r11 * tool_offset[1] + r12 * tool_offset[2]
            offset_z = r20 * tool_offset[0] + r21 * tool_offset[1] + r22 * tool_offset[2]
            
            # Adjust target pose to compensate for TCP offset
            adjusted_pose = Pose()
            adjusted_pose.position.x = pose.position.x - offset_x
            adjusted_pose.position.y = pose.position.y - offset_y
            adjusted_pose.position.z = pose.position.z - offset_z
            adjusted_pose.orientation = pose.orientation
            
            return adjusted_pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to apply TCP offset: {e}')
            return pose
    
    def compute_ik(self, pose):
        # Apply TCP offset
        adjusted_pose = self.apply_tcp_offset(pose)
        
        # Calculate inverse kinematics
        req = GetPositionIK.Request()
        req.ik_request.group_name = "arm"
        req.ik_request.robot_state.joint_state.name = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.ik_link_name = "link6"
        req.ik_request.pose_stamped.pose = adjusted_pose
        req.ik_request.timeout.sec = 5
        
        future = self.compute_ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().solution.joint_state.position
        else:
            self.get_logger().error('IK computation failed')
            return None
    
    def compute_cartesian_path(self, waypoints):
        # Apply TCP offset to all waypoints
        adjusted_waypoints = [self.apply_tcp_offset(pose) for pose in waypoints]
        
        # Calculate Cartesian path
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "arm"
        req.link_name = "link6"
        req.start_state.joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]        
        req.waypoints = adjusted_waypoints.copy()
            
        req.max_step = 0.001
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        future = self.compute_cartesian_path_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().solution.joint_trajectory
        else:
            self.get_logger().error('Cartesian path computation failed')
            return None
    
    def execute_trajectory(self, trajectory: JointTrajectory):
        # Execute trajectory
        self.trajectory_pub.publish(trajectory)
        self.state = ArmState.MOVING
    
    def move_to_pose(self, position, orientation=None):
        # Move to specified pose
        if orientation is None:
            orientation = self.orientation
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        joint_positions = self.compute_ik(pose)
        if joint_positions is None:
            return False
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 1
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
        ]
        trajectory.points.append(point)
        
        # Execute trajectory
        self.execute_trajectory(trajectory)
        return True
    
    def move_to_home(self):
        # Move to initial position
        home_pos = self.params['arm_home_position']
        res = self.move_to_pose(home_pos)
        self.ArmState = ArmState.IDLE
        return res
    
    def generate_line_trajectory(self, waypoints:list, start, end):
        # Draw a line
        approach_height = self.params['approach_height']
        draw_height = self.params['draw_height']
        
        # Above start point
        pose1 = Pose()
        pose1.position.x = start[0]
        pose1.position.y = start[1]
        pose1.position.z = start[2] + approach_height
        pose1.orientation.x = self.orientation[0]
        pose1.orientation.y = self.orientation[1]
        pose1.orientation.z = self.orientation[2]
        pose1.orientation.w = self.orientation[3]
        waypoints.append(pose1)
        
        # Start point
        pose2 = Pose()
        pose2.position.x = start[0]
        pose2.position.y = start[1]
        pose2.position.z = start[2] + draw_height
        pose2.orientation.x = self.orientation[0]
        pose2.orientation.y = self.orientation[1]
        pose2.orientation.z = self.orientation[2]
        pose2.orientation.w = self.orientation[3]
        waypoints.append(pose2)
        
        # End point
        pose3 = Pose()
        pose3.position.x = end[0]
        pose3.position.y = end[1]
        pose3.position.z = end[2] + draw_height
        pose3.orientation.x = self.orientation[0]
        pose3.orientation.y = self.orientation[1]
        pose3.orientation.z = self.orientation[2]
        pose3.orientation.w = self.orientation[3]
        waypoints.append(pose3)
        
        # Above end point
        pose4 = Pose()
        pose4.position.x = end[0]
        pose4.position.y = end[1]
        pose4.position.z = end[2] + approach_height
        pose4.orientation.x = self.orientation[0]
        pose4.orientation.y = self.orientation[1]
        pose4.orientation.z = self.orientation[2]
        pose4.orientation.w = self.orientation[3]
        waypoints.append(pose4)
        
        return waypoints
    
    def draw_circle(self, center, radius):
        # Draw a circle
        approach_height = self.params['approach_height']
        draw_height = self.params['draw_height']
        
        # Waypoints: above center -> start drawing -> circular path -> above center
        waypoints = []
        
        # Above center
        pose1 = Pose()
        pose1.position.x = center[0]
        pose1.position.y = center[1]
        pose1.position.z = center[2] + approach_height
        pose1.orientation.x = self.orientation[0]
        pose1.orientation.y = self.orientation[1]
        pose1.orientation.z = self.orientation[2]
        pose1.orientation.w = self.orientation[3]
        waypoints.append(pose1)
        
        # Start drawing point
        pose2 = Pose()
        pose2.position.x = center[0] + radius
        pose2.position.y = center[1]
        pose2.position.z = center[2] + draw_height
        pose2.orientation.x = self.orientation[0]
        pose2.orientation.y = self.orientation[1]
        pose2.orientation.z = self.orientation[2]
        pose2.orientation.w = self.orientation[3]
        waypoints.append(pose2)
        
        # Circular path
        num_points = 100
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            pose = Pose()
            pose.position.x = center[0] + radius * math.cos(angle)
            pose.position.y = center[1] + radius * math.sin(angle)
            pose.position.z = center[2] + draw_height
            pose.orientation.x = self.orientation[0]
            pose.orientation.y = self.orientation[1]
            pose.orientation.z = self.orientation[2]
            pose.orientation.w = self.orientation[3]
            waypoints.append(pose)
        
        # Above center
        pose3 = Pose()
        pose3.position.x = center[0]
        pose3.position.y = center[1]
        pose3.position.z = center[2] + approach_height
        pose3.orientation.x = self.orientation[0]
        pose3.orientation.y = self.orientation[1]
        pose3.orientation.z = self.orientation[2]
        pose3.orientation.w = self.orientation[3]
        waypoints.append(pose3)

        # Return to start point
        home_pos = self.params['arm_home_position']
        pose = Pose()
        pose.position.x = home_pos[0]
        pose.position.y = home_pos[1]
        pose.position.z = home_pos[2]
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        waypoints.append(pose)
        
        # Calculate and execute trajectory
        trajectory = self.compute_cartesian_path(waypoints)
        if trajectory is not None:
            self.execute_trajectory(trajectory)
            return True
        return False
    
    def draw_cross(self, center, size):
        # Draw a cross
        half_size = size / 2

        waypoints = []
        
        # First diagonal
        start1 = [center[0] - half_size, center[1] - half_size, center[2]]
        end1 = [center[0] + half_size, center[1] + half_size, center[2]]
        self.generate_line_trajectory(waypoints, start1, end1)
        
        # Second diagonal
        start2 = [center[0] + half_size, center[1] - half_size, center[2]]
        end2 = [center[0] - half_size, center[1] + half_size, center[2]]
        self.generate_line_trajectory(waypoints, start2, end2)

        # Return to start point
        home_pos = self.params['arm_home_position']
        pose = Pose()
        pose.position.x = home_pos[0]
        pose.position.y = home_pos[1]
        pose.position.z = home_pos[2]
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        waypoints.append(pose)

        # Calculate and execute trajectory
        trajectory = self.compute_cartesian_path(waypoints)
        if trajectory is not None:
            self.execute_trajectory(trajectory)
            return True
        return False
    
    def press_button(self):
        # Press the clear button
        button_pos = self.params['clear_button_position']
        approach_height = self.params['approach_height']
        
        # Waypoints: above button -> press button -> above button
        waypoints = []
        
        # Above button
        pose1 = Pose()
        pose1.position.x = button_pos[0]
        pose1.position.y = button_pos[1]
        pose1.position.z = button_pos[2] + approach_height
        pose1.orientation.x = self.orientation[0]
        pose1.orientation.y = self.orientation[1]
        pose1.orientation.z = self.orientation[2]
        pose1.orientation.w = self.orientation[3]
        waypoints.append(pose1)
        
        # Press button
        pose2 = Pose()
        pose2.position.x = button_pos[0]
        pose2.position.y = button_pos[1]
        pose2.position.z = button_pos[2]
        pose2.orientation.x = self.orientation[0]
        pose2.orientation.y = self.orientation[1]
        pose2.orientation.z = self.orientation[2]
        pose2.orientation.w = self.orientation[3]
        waypoints.append(pose2)
        
        # Above button
        pose3 = Pose()
        pose3.position.x = button_pos[0]
        pose3.position.y = button_pos[1]
        pose3.position.z = button_pos[2] + approach_height
        pose3.orientation.x = self.orientation[0]
        pose3.orientation.y = self.orientation[1]
        pose3.orientation.z = self.orientation[2]
        pose3.orientation.w = self.orientation[3]
        waypoints.append(pose3)

        # Return to start point
        home_pos = self.params['arm_home_position']
        pose = Pose()
        pose.position.x = home_pos[0]
        pose.position.y = home_pos[1]
        pose.position.z = home_pos[2]
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        waypoints.append(pose)
        
        # Calculate and execute trajectory
        trajectory = self.compute_cartesian_path(waypoints)
        if trajectory is not None:
            self.execute_trajectory(trajectory)
            return True
        return False
    
    def draw_grid(self):
        # Draw tic-tac-toe grid
        corners = self.params['board_corners']
        tl = corners['top_left']
        tr = corners['top_right']
        br = corners['bottom_right']
        bl = corners['bottom_left']
        
        # Calculate grid line start and end points
        # Two vertical lines
        v1_start = [tl[0] + (tr[0] - tl[0]) / 3, tl[1] + (tr[1] - tl[1]) / 3, tl[2] + (tr[2] - tl[2]) / 3]
        v1_end = [bl[0] + (br[0] - bl[0]) / 3, bl[1] + (br[1] - bl[1]) / 3, bl[2] + (br[2] - bl[2]) / 3]
        
        v2_start = [bl[0] + 2 * (br[0] - bl[0]) / 3, bl[1] + 2 * (br[1] - bl[1]) / 3, bl[2] + 2 * (br[2] - bl[2]) / 3]
        v2_end = [tl[0] + 2 * (tr[0] - tl[0]) / 3, tl[1] + 2 * (tr[1] - tl[1]) / 3, tl[2] + 2 * (tr[2] - tl[2]) / 3]
        
        # Two horizontal lines
        h1_start = [tl[0] + (bl[0] - tl[0]) / 3, tl[1] + (bl[1] - tl[1]) / 3, tl[2] + (bl[2] - tl[2]) / 3]
        h1_end = [tr[0] + (br[0] - tr[0]) / 3, tr[1] + (br[1] - tr[1]) / 3, tr[2] + (br[2] - tr[2]) / 3]
        
        h2_start = [tr[0] + 2 * (br[0] - tr[0]) / 3, tr[1] + 2 * (br[1] - tr[1]) / 3, tr[2] + 2 * (br[2] - tr[2]) / 3]
        h2_end = [tl[0] + 2 * (bl[0] - tl[0]) / 3, tl[1] + 2 * (bl[1] - tl[1]) / 3, tl[2] + 2 * (bl[2] - tl[2]) / 3]
        
        # Generate waypoints for four lines
        waypoints = []
        self.generate_line_trajectory(waypoints, v1_start, v1_end)
        self.generate_line_trajectory(waypoints, v2_start, v2_end)
        self.generate_line_trajectory(waypoints, h1_start, h1_end)
        self.generate_line_trajectory(waypoints, h2_start, h2_end)

        # Return to start point
        home_pos = self.params['arm_home_position']
        pose = Pose()
        pose.position.x = home_pos[0]
        pose.position.y = home_pos[1]
        pose.position.z = home_pos[2]
        pose.orientation.x = self.orientation[0]
        pose.orientation.y = self.orientation[1]
        pose.orientation.z = self.orientation[2]
        pose.orientation.w = self.orientation[3]
        waypoints.append(pose)
        
        # Calculate and execute trajectory
        trajectory = self.compute_cartesian_path(waypoints)
        if trajectory is not None:
            self.execute_trajectory(trajectory)
            return True
        return False
    
    def get_cell_center(self, position):
        # Get center coordinates of specified position
        row = position // 3
        col = position % 3
        
        corners = self.params['board_corners']
        tl = corners['top_left']  # Top-left corner [x, y, z]
        tr = corners['top_right']  # Top-right corner [x, y, z]
        br = corners['bottom_right']  # Bottom-right corner [x, y, z]
        bl = corners['bottom_left']  # Bottom-left corner [x, y, z]
        
        # Calculate two vectors of the board plane
        vector_right = [tr[0] - tl[0], tr[1] - tl[1], tr[2] - tl[2]]  # Vector from top-left to top-right
        vector_down = [bl[0] - tl[0], bl[1] - tl[1], bl[2] - tl[2]]  # Vector from top-left to bottom-left
        
        # Calculate cell width and height (proportion in parameterized space)
        u = (col + 0.5) / 3.0  # Horizontal parameterized coordinate (0-1)
        v = (row + 0.5) / 3.0  # Vertical parameterized coordinate (0-1)
        
        # Calculate 3D coordinates of cell center
        # Use bilinear interpolation for all three dimensions
        x = tl[0] + u * vector_right[0] + v * vector_down[0]
        y = tl[1] + u * vector_right[1] + v * vector_down[1]
        z = tl[2] + u * vector_right[2] + v * vector_down[2]
        
        return [x, y, z]
    
    def move_to_calibration_point(self, point_name):
        """Move to calibration point"""
        if point_name == "top_left":
            position = self.params['board_corners']['top_left']
        elif point_name == "top_right":
            position = self.params['board_corners']['top_right']
        elif point_name == "bottom_right":
            position = self.params['board_corners']['bottom_right']
        elif point_name == "bottom_left":
            position = self.params['board_corners']['bottom_left']
        elif point_name == "clear_button":
            position = self.params['clear_button_position']
        else:
            self.get_logger().error(f'Unknown calibration point: {point_name}')
            return False
        
        # Add a slightly raised height to avoid collision
        approach_height = self.params['draw_height'] + 0.01
        position_with_height = [position[0], position[1], position[2] + approach_height]

        res = self.move_to_pose(position_with_height)
        self.state = ArmState.CALIBRATING
        return res
    
    # Service callback functions
    def draw_grid_callback(self, request, response):
        self.get_logger().info('Received request to draw grid')
        success = self.draw_grid()
        return Empty.Response()
    
    def draw_symbol_callback(self, request, response):
        position = request.position
        symbol = request.symbol
        self.get_logger().info(f'Received request to draw symbol {symbol} at position {position}')
        
        center = self.get_cell_center(position)
        
        if symbol == 1:  # X
            success = self.draw_cross(center, self.params['cross_size'])
        else:  # O
            success = self.draw_circle(center, self.params['circle_radius'])
        
        response.success = success
        return response
    
    def press_button_callback(self, request, response):
        self.get_logger().info('Received request to press button')
        success = self.press_button()
        return Empty.Response()
    
    def move_home_callback(self, request, response):
        self.get_logger().info('Received request to move home')
        success = self.move_to_home()
        return Empty.Response()
    
    def update_parameters_callback(self, request, response):
        self.get_logger().info('Received request to update parameters')
        
        # If arm is moving, stop motion and return to home position
        if self.is_moving():
            self.get_logger().warn('Arm is moving, stopping and moving to home...')
            
            # Cancel current motion
            self.cancel_current_motion()
            
            # Move to home position
            self.move_to_home()
            
            # Wait a while to ensure arm stops
            time.sleep(2)
        
        # Reload parameters
        self.load_parameters()
        return Empty.Response()
    
    def start_calibration_callback(self, request, response):
        self.get_logger().info('Received request to start calibration')
        self.state = ArmState.CALIBRATING
        self.calibration_step = CalibrationStep.START
        return Empty.Response()
    
    def next_calibration_step_callback(self, request, response):
        self.get_logger().info('Received request for next calibration step')
        
        if self.state != ArmState.CALIBRATING:
            self.get_logger().warn('Not in calibration mode')
            return Empty.Response()
        
        # Move to next calibration point
        if self.calibration_step == CalibrationStep.START:
            self.calibration_step = CalibrationStep.TOP_LEFT
            success = self.move_to_calibration_point("top_left")
        elif self.calibration_step == CalibrationStep.TOP_LEFT:
            self.calibration_step = CalibrationStep.TOP_RIGHT
            success = self.move_to_calibration_point("top_right")
        elif self.calibration_step == CalibrationStep.TOP_RIGHT:
            self.calibration_step = CalibrationStep.BOTTOM_RIGHT
            success = self.move_to_calibration_point("bottom_right")
        elif self.calibration_step == CalibrationStep.BOTTOM_RIGHT:
            self.calibration_step = CalibrationStep.BOTTOM_LEFT
            success = self.move_to_calibration_point("bottom_left")
        elif self.calibration_step == CalibrationStep.BOTTOM_LEFT:
            self.calibration_step = CalibrationStep.CLEAR_BUTTON
            success = self.move_to_calibration_point("clear_button")
        elif self.calibration_step == CalibrationStep.CLEAR_BUTTON:
            self.calibration_step = CalibrationStep.COMPLETE
            success = self.move_to_home()
        else:
            self.get_logger().warn('Calibration already complete')
            success = True
        
        return Empty.Response()
    
    def cancel_calibration_callback(self, request, response):
        self.get_logger().info('Received request to cancel calibration')
        self.state = ArmState.IDLE
        self.calibration_step = CalibrationStep.START
        return Empty.Response()
    
    def finish_calibration_callback(self, request, response):
        self.get_logger().info('Received request to finish calibration')
        self.state = ArmState.IDLE
        self.calibration_step = CalibrationStep.START
        return Empty.Response()

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    rclpy.spin(arm_controller)
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()