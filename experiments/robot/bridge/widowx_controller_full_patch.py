"""
Comprehensive patch for widowx_controller to work with ROS 2.
This replaces the entire widowx_controller module.
"""

import sys
import numpy as np
import time
from threading import Lock
import logging
import os

# Import interbotix modules
from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface

# Mock the controller base
class RobotControllerBase:
    pass

class GripperControllerBase:
    pass

# Constants from the original file
ABS_MAX_JOINT_EFFORTS = np.array([800, 1000, 600.0, 600.0, 600.0, 700.0]) * 1.7
NEUTRAL_JOINT_STATE = np.array([-0.13192235, -0.76238847,  0.44485444,
                                -0.01994175,  1.7564081,  -0.15953401])
DEFAULT_ROTATION = np.array([[0 , 0, 1.0],
                             [0, 1.0,  0],
                             [-1.0,  0, 0]])

# Mock GripperController
class GripperController:
    """Mock GripperController that simulates gripper behavior without ROS 1."""
    
    def __init__(self, robot_name, create_node=False, upper_limit=0.035, lower_limit=0.010, des_pos_max=1, des_pos_min=0):
        self.robot_name = robot_name
        self.des_pos_max = des_pos_max
        self.des_pos_min = des_pos_min
        self._upper_limit = upper_limit
        self._lower_limit = lower_limit
        assert self._upper_limit > self._lower_limit
        
        self._joint_lock = Lock()
        self._des_pos_lock = Lock()
        
        # Simulated gripper state
        self._angles = {'left_finger': upper_limit, 'right_finger': upper_limit, 'gripper': upper_limit}
        self._velocities = {'left_finger': 0.0, 'right_finger': 0.0, 'gripper': 0.0}
        
        self._moving = False
        self._time_movement_started = None
        self._grace_period_until_can_be_marked_as_stopped = 0.1
        self._des_pos = None
        self.des_pos = self._upper_limit
        
        print(f"Created mock GripperController for {robot_name}")
    
    @property
    def des_pos(self):
        return self._des_pos
    
    @des_pos.setter
    def des_pos(self, value):
        if value != self._des_pos:
            with self._des_pos_lock:
                self._moving = True
                self._time_movement_started = time.time()
                self._des_pos = value
                # Simulate gripper movement
                self._angles['left_finger'] = value
                self._angles['right_finger'] = value
                self._angles['gripper'] = value
    
    def get_gripper_pos(self):
        with self._joint_lock:
            return self._angles.get('left_finger', 0.0)
    
    def open(self):
        self.des_pos = self._upper_limit
    
    def close(self):
        self.des_pos = self._lower_limit
    
    def set_continuous_position(self, target):
        target_clipped = np.clip(target, self.des_pos_min, self.des_pos_max)
        if target != target_clipped:
            print('Warning target gripper pos outside of range', target)
        self.des_pos = self.denormalize(target_clipped)
    
    def get_continuous_position(self):
        gripper_pos = self.get_gripper_pos()
        return self.normalize(gripper_pos)
    
    def normalize(self, x):
        return (self.des_pos_max - self.des_pos_min) * (x - self._lower_limit) / (self._upper_limit - self._lower_limit) + self.des_pos_min
    
    def denormalize(self, x):
        return (x - self.des_pos_min) * (self._upper_limit - self._lower_limit) / (self.des_pos_max - self.des_pos_min) + self._lower_limit
    
    def is_moving(self):
        return self._moving
    
    def get_gripper_target_position(self):
        des_pos_normed = self.normalize(self.des_pos)
        assert des_pos_normed <= self.des_pos_max and des_pos_normed >= self.des_pos_min
        return des_pos_normed
    
    def update_gripper_pwm(self, event):
        # Mock implementation - no actual PWM control
        pass
    
    def get_gripper_pwm(self, pressure):
        # Mock implementation
        return 0


# Simplified WidowX_Controller that uses the actual interbotix ROS 2 interface
class WidowX_Controller(RobotControllerBase):
    def __init__(self, robot_name, print_debug, gripper_params,
                 enable_rotation='6dof',
                 gripper_attached='custom',
                 normal_base_angle=0):
        """
        Simplified WidowX controller that uses the interbotix ROS 2 interface directly.
        """
        print('Setting up simplified WidowX controller for ROS 2...')
        
        # Use the interbotix ROS 2 interface directly
        # The InterbotixRobotXSCore will handle all ROS 2 communication
        self.core = InterbotixRobotXSCore(robot_model=robot_name, robot_name=robot_name)
        self.arm = InterbotixArmXSInterface(self.core, robot_model=robot_name, group_name="arm")
        
        # Initialize with reasonable defaults, but let environment override
        self.current_moving_time = 2.0  # Default from interbotix
        self.current_accel_time = 0.3   # Default from interbotix
        
        if gripper_attached != 'default':
            # Use real interbotix gripper interface for custom gripper
            self.gripper = InterbotixGripperXSInterface(self.core, gripper_name="gripper")
            self._gripper = GripperController(robot_name=robot_name, 
                                            des_pos_max=gripper_params.get('des_pos_max', 1),
                                            des_pos_min=gripper_params.get('des_pos_min', 0))
            self.custom_gripper_controller = True
        else:
            # Use interbotix gripper interface
            self.gripper = InterbotixGripperXSInterface(self.core, gripper_name="gripper")
            self.custom_gripper_controller = False
            self.des_gripper_state = np.array([1])
        
        self._robot_name = robot_name
        self.gripper_params = gripper_params or {}
        
        # Initialize joint state tracking
        self._joint_lock = Lock()
        self._angles = {}
        self._velocities = {}
        self._effort = {}
        
        # Get joint limits and info from the arm interface
        self._upper_joint_limits = np.array(self.arm.group_info.joint_upper_limits)
        self._lower_joint_limits = np.array(self.arm.group_info.joint_lower_limits)
        self._qn = self.arm.group_info.num_joints
        self.joint_names = self.arm.group_info.joint_names
        
        # Set up default rotation
        from widowx_envs.utils import transformation_utils as tr
        self.default_rot = np.dot(tr.eulerAnglesToRotationMatrix([0, 0, normal_base_angle]), DEFAULT_ROTATION)
        self.neutral_joint_angles = NEUTRAL_JOINT_STATE
        self.enable_rotation = enable_rotation
        
        # Subscribe to joint states using the core's joint state
        # The interbotix core already subscribes to joint states internally
        self._update_joint_states()
        
        # Initialize gripper to open position
        print("Initializing gripper...")
        self.open_gripper(wait=True)
        
        print('WidowX controller setup complete!')
    
    def _update_joint_states(self):
        """Update joint states from the interbotix core."""
        if hasattr(self.core, 'joint_states') and self.core.joint_states is not None:
            with self._joint_lock:
                for i, name in enumerate(self.core.joint_states.name):
                    if i < len(self.core.joint_states.position):
                        self._angles[name] = self.core.joint_states.position[i]
                    if i < len(self.core.joint_states.velocity):
                        self._velocities[name] = self.core.joint_states.velocity[i]
                    if i < len(self.core.joint_states.effort):
                        self._effort[name] = self.core.joint_states.effort[i]
    
    def get_joint_angles(self):
        """Returns current joint angles."""
        self._update_joint_states()
        with self._joint_lock:
            try:
                return np.array([self._angles[k] for k in self.joint_names])
            except KeyError:
                # Return zeros if joint states not yet available
                return np.zeros(self._qn)
    
    def get_joint_effort(self):
        """Returns current joint efforts."""
        self._update_joint_states()
        with self._joint_lock:
            try:
                return np.array([self._effort[k] for k in self.joint_names])
            except KeyError:
                return np.zeros(self._qn)
    
    def get_joint_angles_velocity(self):
        """Returns velocities for joints."""
        self._update_joint_states()
        with self._joint_lock:
            try:
                return np.array([self._velocities[k] for k in self.joint_names])
            except KeyError:
                return np.zeros(self._qn)
    
    def get_state(self):
        """Get a tuple of (joint_angles, joint_angles_velocity, cartesian_pose)"""
        return self.get_joint_angles(), self.get_joint_angles_velocity(), self.get_cartesian_pose()
    
    def move_to_neutral(self, duration=4):
        """Move to neutral position."""
        print('Moving to neutral position...')
        self.arm.set_joint_positions(self.neutral_joint_angles, moving_time=duration)
        time.sleep(duration + 0.5)  # Wait for movement to complete
    
    def move_to_state(self, target_xyz, target_zangle, duration=1.5):
        """Move to a specific state with smooth motion."""
        print(f"Moving to state: xyz={target_xyz}, zangle={target_zangle}, duration={duration:.2f}s")
        
        # Set trajectory time for smooth motion
        self.set_moving_time(duration)
        
        # Convert to pose matrix and move
        from widowx_envs.utils import transformation_utils as tr
        target_pose = np.eye(4)
        target_pose[:3, 3] = target_xyz
        
        # Apply rotation
        rot_matrix = tr.eulerAnglesToRotationMatrix([0, 0, target_zangle])
        target_pose[:3, :3] = np.dot(rot_matrix, self.default_rot)
        
        return self.move_to_eep(target_pose, duration=duration, blocking=True)
    
    def move_to_eep(self, target_pose, duration=None, blocking=True, check_effort=True, step=True):
        """Move end effector to target pose with smooth motion."""
        if duration is not None:
            # Use the specific duration provided by the environment
            self.set_moving_time(duration)
            actual_duration = duration
        else:
            # Use current moving time
            actual_duration = self.current_moving_time
        
        print(f"Moving to end effector pose with duration={actual_duration:.2f}s, blocking={blocking}")
        
        try:
            success = self.arm.set_ee_pose_matrix(target_pose, moving_time=actual_duration, blocking=blocking)
            if not success:
                print("Warning: IK solver could not find valid solution for target pose")
                # Return False to indicate failure, but don't crash
                return False
            return True
        except Exception as e:
            print(f"Error in move_to_eep: {e}")
            return False
    
    def set_moving_time(self, moving_time):
        """Set the moving time for trajectories."""
        # Use better acceleration timing for smooth motion
        accel_time = moving_time * 0.5  # 50% of time for acceleration (smoother for longer movements)
        
        print(f"Setting trajectory time: moving_time={moving_time:.2f}s, accel_time={accel_time:.2f}s")
        self.arm.set_trajectory_time(moving_time=moving_time, accel_time=accel_time)
        
        # Store for future use
        self.current_moving_time = moving_time
        self.current_accel_time = accel_time
    
    def get_cartesian_pose(self, matrix=False):
        """Get current cartesian pose."""
        # Get from the arm interface
        if matrix:
            return self.arm.get_ee_pose()
        else:
            pose_matrix = self.arm.get_ee_pose()
            # Convert to position + quaternion
            from pyquaternion import Quaternion
            pos = pose_matrix[:3, 3]
            quat = Quaternion(matrix=pose_matrix[:3, :3])
            return np.concatenate([pos, quat.elements])
    
    # Gripper methods
    def open_gripper(self, wait=False):
        print("Opening gripper...")
        if self.custom_gripper_controller:
            self._gripper.open()
            # Also use real gripper
            delay = 1.0 if wait else 0.1
            self.gripper.release(delay=delay)
        else:
            delay = 1.0 if wait else 0.1
            self.gripper.release(delay=delay)
            self.des_gripper_state = np.array([1])
    
    def close_gripper(self, wait=False):
        print("Closing gripper...")
        if self.custom_gripper_controller:
            self._gripper.close()
            # Also use real gripper
            delay = 1.0 if wait else 0.1
            self.gripper.grasp(delay=delay)
        else:
            delay = 1.0 if wait else 0.1
            self.gripper.grasp(delay=delay)
            self.des_gripper_state = np.array([0])
    
    def get_gripper_position(self):
        if self.custom_gripper_controller:
            # Get from real gripper, fallback to mock
            try:
                return self.gripper.get_gripper_position()
            except:
                return self._gripper.get_continuous_position()
        else:
            # Get from real gripper
            try:
                return self.gripper.get_gripper_position()
            except:
                # Fallback to joint states
                self._update_joint_states()
                return self._angles.get('gripper', 0.0)
    
    def get_continuous_gripper_position(self):
        return self.get_gripper_position()
    
    def set_continuous_gripper_position(self, target):
        # Extract scalar value if target is a numpy array
        target_value = float(target[0]) if hasattr(target, '__len__') else float(target)
        print(f"Setting gripper position to: {target_value:.3f}")
        if self.custom_gripper_controller:
            self._gripper.set_continuous_position(target_value)
            # Also use real gripper - map target to gripper position
            if target_value > 0.5:
                self.gripper.release(delay=0.1)
            else:
                self.gripper.grasp(delay=0.1)
        else:
            # Map to gripper commands
            if target_value > 0.5:
                self.open_gripper()
            else:
                self.close_gripper()
    
    def get_gripper_desired_position(self):
        if self.custom_gripper_controller:
            return self._gripper.get_gripper_target_position()
        else:
            return self.des_gripper_state[0]
    
    def wait_until_gripper_position_reached(self):
        """Wait until gripper reaches target position."""
        time.sleep(0.5)  # Simple wait
    
    # Stub methods that were in original but not critical
    def clean_shutdown(self):
        pass
    
    def check_motor_status_and_reboot(self):
        pass
    
    def reboot_motor(self, joint_name):
        pass


# Mock the publish_transform function
def publish_transform(transform, name, parent_name='wx250s/base_link'):
    """Mock transform publisher."""
    pass


# Create the modules
widowx_controller_module = type('Module', (), {
    'WidowX_Controller': WidowX_Controller,
    'RobotControllerBase': RobotControllerBase,
    'publish_transform': publish_transform,
})()

custom_gripper_module = type('Module', (), {
    'GripperController': GripperController,
    'GripperControllerBase': GripperControllerBase,
})()

controller_base_module = type('Module', (), {
    'RobotControllerBase': RobotControllerBase,
    'GripperControllerBase': GripperControllerBase,
})()

# Inject the modules
sys.modules['widowx_controller.widowx_controller'] = widowx_controller_module
sys.modules['widowx_controller.custom_gripper_controller'] = custom_gripper_module
sys.modules['widowx_controller.controller_base'] = controller_base_module

print("Patched widowx_controller modules for ROS 2 compatibility") 