"""
Simple WidowX Environment for ROS 2

This is a simplified environment that directly uses the existing ROS 2 interbotix interface
instead of trying to adapt the complex ROS 1 widowx_envs code.
"""

import numpy as np
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.msg import JointSingleCommand
from cv_bridge import CvBridge
import threading


class SimpleWidowXEnv:
    """Simple WidowX environment that uses ROS 2 interbotix interface directly."""
    
    def __init__(self, cfg):
        self.cfg = cfg
        self.bridge = CvBridge()
        
        # Initialize ROS 2 if not already initialized
        try:
            if not rclpy.ok():
                print("DEBUG: Initializing ROS 2...")
                rclpy.init()
            else:
                print("DEBUG: ROS 2 already initialized")
        except Exception as e:
            print(f"DEBUG: ROS 2 initialization issue: {e}")
            # Try to initialize anyway
            try:
                rclpy.init()
            except:
                pass
        
        # Create a simple node for this environment
        print("DEBUG: Creating ROS 2 node...")
        self.node = rclpy.create_node('simple_widowx_env')
        print("DEBUG: ROS 2 node created successfully")
        
        # Publishers for robot control
        self.joint_pub = self.node.create_publisher(
            JointGroupCommand, 
            '/wx250s/commands/joint_group', 
            10
        )
        
        # Publisher for single joint commands (gripper)
        self.joint_single_pub = self.node.create_publisher(
            JointSingleCommand,
            '/wx250s/commands/joint_single',
            10
        )
        
        # Subscribers for robot state
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/wx250s/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Camera subscriber
        self.image_sub = self.node.create_subscription(
            Image,
            '/camera/camera_stream/color/image_raw',  # Fixed topic name
            self.image_callback,
            10
        )
        
        # State variables
        self.current_joint_states = None
        self.current_image = None
        self.joint_lock = threading.Lock()
        self.image_lock = threading.Lock()
        
        # Start ROS 2 spinning in a separate thread
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
        # Wait for initial data
        print("Waiting for robot state and camera data...")
        self.wait_for_data()
        print("Robot ready!")
        
    def spin_ros(self):
        """Spin ROS 2 in a separate thread."""
        rclpy.spin(self.node)
        
    def joint_state_callback(self, msg):
        """Callback for joint state updates."""
        with self.joint_lock:
            self.current_joint_states = msg
            
    def image_callback(self, msg):
        """Callback for camera image updates."""
        with self.image_lock:
            try:
                # Convert ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.current_image = cv_image
            except Exception as e:
                print(f"Error converting image: {e}")
                
    def wait_for_data(self):
        """Wait for initial robot state and camera data."""
        timeout = 10.0  # 10 second timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.joint_lock:
                joint_ready = self.current_joint_states is not None
            with self.image_lock:
                image_ready = self.current_image is not None
                
            if joint_ready and image_ready:
                return
                
            time.sleep(0.1)
            
        if self.current_joint_states is None:
            print("Warning: No joint state data received")
        if self.current_image is None:
            print("Warning: No camera data received")
            
    def get_observation(self):
        """Get current observation from robot."""
        print("DEBUG: Starting get_observation()")
        obs = {}
        
        # Get current image
        print("DEBUG: Getting current image...")
        with self.image_lock:
            if self.current_image is not None:
                print("DEBUG: Image available, copying...")
                obs["full_image"] = self.current_image.copy()
                obs["image_primary"] = self.current_image.copy()
                print(f"DEBUG: Image shape: {self.current_image.shape}")
            else:
                print("DEBUG: No image available, using dummy...")
                # Dummy image if no camera data
                obs["full_image"] = np.zeros((480, 640, 3), dtype=np.uint8)
                obs["image_primary"] = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Get current joint states (proprioception)
        print("DEBUG: Getting joint states...")
        with self.joint_lock:
            if self.current_joint_states is not None:
                print("DEBUG: Joint states available...")
                # Extract arm joint positions (first 6 joints typically)
                arm_positions = list(self.current_joint_states.position[:6])
                # Add gripper state (assume last joint or default)
                if len(self.current_joint_states.position) > 6:
                    gripper_pos = self.current_joint_states.position[-1]
                else:
                    gripper_pos = 0.0
                arm_positions.append(gripper_pos)
                obs["proprio"] = np.array(arm_positions)
                print(f"DEBUG: Joint positions: {arm_positions}")
            else:
                print("DEBUG: No joint states available, using default...")
                # Default proprioception if no joint data
                obs["proprio"] = np.zeros(7)
        
        print("DEBUG: get_observation() completed successfully")
        return obs
        
    def reset(self):
        """Reset the environment."""
        print("Resetting robot to neutral position...")
        
        # Move to neutral position
        neutral_positions = [0.0, -0.8, 0.4, 0.0, 1.8, 0.0]  # Typical neutral pose
        self.move_to_joint_positions(neutral_positions)
        
        # Wait a bit for movement to complete
        time.sleep(2.0)
        
        # Return initial observation
        obs = self.get_observation()
        return obs, {}
        
    def step(self, action):
        """Execute an action using end-effector pose control."""
        print(f"Executing action: {action}")
        
        # OpenVLA action format: [dx, dy, dz, droll, dpitch, dyaw, gripper]
        # Convert to end-effector pose change
        
        # Get current end-effector pose (this would need to be implemented)
        # For now, let's use a simplified approach with joint control but better scaling
        
        # Get current joint positions
        with self.joint_lock:
            if self.current_joint_states is not None:
                current_positions = list(self.current_joint_states.position[:6])
            else:
                print("Warning: No joint states available for action execution")
                current_positions = [0.0, -0.8, 0.4, 0.0, 1.8, 0.0]
        
        # Convert action to joint position increments with better scaling
        # Scale down actions significantly for safety
        action_scale = 0.02  # Much smaller scale for safety
        
        new_positions = []
        for i in range(6):
            if i < len(action) - 1:  # Exclude gripper (last element)
                increment = action[i] * action_scale
                new_pos = current_positions[i] + increment
                # Apply joint limits (simplified)
                new_pos = max(-3.14, min(3.14, new_pos))
                new_positions.append(new_pos)
            else:
                new_positions.append(current_positions[i])
        
        print(f"Current positions: {current_positions}")
        print(f"New positions: {new_positions}")
        print(f"Increments: {[new_positions[i] - current_positions[i] for i in range(6)]}")
        
        # Send joint command
        self.move_to_joint_positions(new_positions)
        
        # Handle gripper command (last element of action)
        if len(action) > 6:
            gripper_cmd = action[6]
            print(f"Gripper command: {gripper_cmd}")
            # gripper_cmd > 0.5 means open, < 0.5 means close
            if gripper_cmd > 0.5:
                self.open_gripper()
            else:
                self.close_gripper()
        
        # Wait for movement to start
        time.sleep(0.3)
        
        obs = self.get_observation()
        reward = 0.0
        done = False
        truncated = False
        info = {}
        
        return obs, reward, done, truncated, info
        
    def move_to_joint_positions(self, positions):
        """Move robot to specified joint positions."""
        cmd = JointGroupCommand()
        cmd.name = "arm"
        cmd.cmd = positions
        
        self.joint_pub.publish(cmd)
        
    def open_gripper(self):
        """Open the gripper."""
        cmd = JointSingleCommand()
        cmd.name = "gripper"
        cmd.cmd = 0.037  # Open position for interbotix gripper
        
        self.joint_single_pub.publish(cmd)
        
    def close_gripper(self):
        """Close the gripper."""
        cmd = JointSingleCommand()
        cmd.name = "gripper"
        cmd.cmd = -0.037  # Close position for interbotix gripper
        
        self.joint_single_pub.publish(cmd)
        
    def close(self):
        """Clean up the environment."""
        if hasattr(self, 'node'):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown() 