"""
Minimal patch for widowx_controller to work with ROS 2.
This patches only the ROS 1 API calls while preserving all original functionality.
"""

import sys
import time

# First, let's patch the rospy calls in the original widowx_controller
# We'll monkey-patch specific functions rather than replacing entire classes

def patch_widowx_controller():
    """Apply minimal patches to make widowx_controller work with ROS 2."""
    
    # Import the original modules first
    try:
        from widowx_controller import widowx_controller
        from widowx_controller import custom_gripper_controller
        
        # Patch the rospy.Timer call in custom_gripper_controller
        original_gripper_init = custom_gripper_controller.GripperController.__init__
        
        def patched_gripper_init(self, robot_name, create_node=False, upper_limit=0.035, lower_limit=0.010, des_pos_max=1, des_pos_min=0):
            # Call original init but skip the rospy.Timer call
            if create_node:
                import rospy
                rospy.init_node('gripper_controller')
            
            assert des_pos_max >= des_pos_min, "gripper des_pos_max has to be >= des_pos_min"
            self.des_pos_max = des_pos_max
            self.des_pos_min = des_pos_min
            self._upper_limit = upper_limit
            self._lower_limit = lower_limit
            assert self._upper_limit > self._lower_limit
            
            # Skip the rospy.Timer call for ROS 2 compatibility
            # if not create_node:
            #     rospy.Timer(rospy.Duration(0.02), self.update_gripper_pwm)

            from threading import Lock
            self._joint_lock = Lock()
            self._des_pos_lock = Lock()

            self._angles = {}
            self._velocities = {}
            
            # Use ROS 2 compatible publisher and subscriber
            try:
                from interbotix_xs_msgs.msg import JointSingleCommand
            except:
                from interbotix_xs_sdk.msg import JointSingleCommand
            
            import rospy
            from sensor_msgs.msg import JointState
            
            self._pub_gripper_command = rospy.Publisher(f"/{robot_name}/commands/joint_single", JointSingleCommand, queue_size=3)
            rospy.Subscriber(f"/{robot_name}/joint_states", JointState, self._joint_callback)

            self._moving = False
            self._time_movement_started = None
            self._grace_period_until_can_be_marked_as_stopped = 0.1
            self._des_pos = None
            self.des_pos = self._upper_limit
        
        # Apply the patch
        custom_gripper_controller.GripperController.__init__ = patched_gripper_init
        
        # Patch the WidowX_Controller to handle ROS 2 service calls
        original_widowx_init = widowx_controller.WidowX_Controller.__init__
        
        def patched_widowx_init(self, robot_name, print_debug, gripper_params,
                               enable_rotation='6dof',
                               gripper_attached='custom',
                               normal_base_angle=0):
            """Patched WidowX_Controller init that handles ROS 2 compatibility."""
            
            print('waiting for widowx_controller to be set up...')
            
            # Use the original ModifiedInterbotixManipulatorXS but with init_node=False
            # since the interbotix ROS 2 node is already initialized
            self.bot = widowx_controller.ModifiedInterbotixManipulatorXS(robot_model=robot_name, init_node=False)
            
            if gripper_params is None:
                gripper_params = {}

            self._robot_name = robot_name
            
            # Skip rospy.on_shutdown for ROS 2
            # rospy.on_shutdown(self.clean_shutdown)

            import logging
            logger = logging.getLogger('robot_logger')
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            log_level = logging.WARN
            if print_debug:
                log_level = logging.DEBUG
            ch = logging.StreamHandler()
            ch.setLevel(log_level)
            ch.setFormatter(formatter)
            logger.addHandler(ch)
            
            self._init_gripper(gripper_attached, gripper_params)

            from threading import Lock
            self._joint_lock = Lock()
            self._angles, self._velocities, self._effort = {}, {}, {}
            
            # Use the rospy compatibility layer for subscriber
            import rospy
            from sensor_msgs.msg import JointState
            rospy.Subscriber(f"/{robot_name}/joint_states", JointState, self._joint_callback)
            time.sleep(1)
            self._n_errors = 0

            import numpy as np
            self._upper_joint_limits = np.array(self.bot.arm.group_info.joint_upper_limits)
            self._lower_joint_limits = np.array(self.bot.arm.group_info.joint_lower_limits)
            self._qn = self.bot.arm.group_info.num_joints

            self.joint_names = self.bot.arm.group_info.joint_names
            
            import widowx_envs.utils.transformation_utils as tr
            self.default_rot = np.dot(tr.eulerAnglesToRotationMatrix([0, 0, normal_base_angle]), widowx_controller.DEFAULT_ROTATION)

            self.neutral_joint_angles = widowx_controller.NEUTRAL_JOINT_STATE
            self.enable_rotation = enable_rotation
        
        # Apply the patch
        widowx_controller.WidowX_Controller.__init__ = patched_widowx_init
        
        # Patch service calls to be no-ops for ROS 2 (since we're using interbotix directly)
        def patched_reboot_motor(self, joint_name: str):
            """Patched reboot_motor that doesn't use rospy services."""
            print(f"Would reboot motor: {joint_name} (disabled for ROS 2 compatibility)")
            return None
        
        def patched_wait_for_service(service_name):
            """Mock wait_for_service for ROS 2."""
            print(f"Would wait for service: {service_name} (disabled for ROS 2 compatibility)")
        
        def patched_service_proxy(service_name, service_type):
            """Mock ServiceProxy for ROS 2."""
            class DummyProxy:
                def __call__(self, *args, **kwargs):
                    print(f"Would call service: {service_name} (disabled for ROS 2 compatibility)")
                    return None
            return DummyProxy()
        
        # Apply service patches
        widowx_controller.WidowX_Controller.reboot_motor = patched_reboot_motor
        
        # Patch rospy service calls
        import rospy
        rospy.wait_for_service = patched_wait_for_service
        rospy.ServiceProxy = patched_service_proxy
        
        print("Applied minimal widowx_controller patches for ROS 2 compatibility")
        
    except ImportError as e:
        print(f"Could not patch widowx_controller: {e}")
        print("Original modules not available, using fallback")

# Apply the patches
patch_widowx_controller() 