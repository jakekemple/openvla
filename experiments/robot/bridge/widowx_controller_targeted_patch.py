"""
Targeted patch for widowx_controller to work with ROS 2.
This preserves all original functionality while only fixing ROS 1/2 compatibility.
"""

import sys

def apply_targeted_patches():
    """Apply targeted patches to preserve original behavior while fixing ROS compatibility."""
    
    # Patch the ModifiedInterbotixManipulatorXS to not create a duplicate node
    try:
        # Import the original widowx_controller module
        from widowx_controller import widowx_controller
        
        # Store the original class
        OriginalModifiedInterbotixManipulatorXS = widowx_controller.ModifiedInterbotixManipulatorXS
        
        class PatchedModifiedInterbotixManipulatorXS(OriginalModifiedInterbotixManipulatorXS):
            def __init__(self, robot_model, group_name="arm", gripper_name="gripper", robot_name=None, moving_time=2.0, accel_time=0.3, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
                # Always set init_node=False to avoid duplicate node creation
                # The interbotix ROS 2 node is already running
                super().__init__(robot_model, group_name, gripper_name, robot_name, moving_time, accel_time, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit, init_node=False)
        
        # Replace the class in the module
        widowx_controller.ModifiedInterbotixManipulatorXS = PatchedModifiedInterbotixManipulatorXS
        
        # Patch the custom gripper controller to skip the rospy.Timer
        from widowx_controller import custom_gripper_controller
        
        # Store original init
        original_gripper_init = custom_gripper_controller.GripperController.__init__
        
        def patched_gripper_init(self, robot_name, create_node=False, upper_limit=0.035, lower_limit=0.010, des_pos_max=1, des_pos_min=0):
            # Call most of the original init logic but skip problematic parts
            if create_node:
                import rospy
                rospy.init_node('gripper_controller')
            assert des_pos_max >= des_pos_min, "gripper des_pos_max has to be >= des_pos_min"
            self.des_pos_max = des_pos_max
            self.des_pos_min = des_pos_min
            self._upper_limit = upper_limit
            self._lower_limit = lower_limit
            assert self._upper_limit > self._lower_limit
            
            # Skip the rospy.Timer call that causes issues
            # if not create_node:
            #     rospy.Timer(rospy.Duration(0.02), self.update_gripper_pwm)

            from threading import Lock
            self._joint_lock = Lock()
            self._des_pos_lock = Lock()

            self._angles = {}
            self._velocities = {}
            
            # Keep the original publisher and subscriber logic
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
        
        # Apply the gripper patch
        custom_gripper_controller.GripperController.__init__ = patched_gripper_init
        
        print("Applied targeted widowx_controller patches for ROS 2 compatibility")
        
    except ImportError as e:
        print(f"Could not apply targeted patches: {e}")
        # Fall back to the comprehensive patch if needed
        try:
            import widowx_controller_full_patch
            print("Fell back to comprehensive patch")
        except ImportError:
            print("No patches available")

# Apply the patches
apply_targeted_patches() 