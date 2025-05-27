"""
Minimal node creation patch for widowx_controller.
This only prevents duplicate ROS node creation while preserving all original behavior.
"""

import sys

def patch_node_creation():
    """Patch only the node creation to prevent conflicts with existing ROS 2 node."""
    
    # The key insight: the original widowx_controller works fine with ROS 2 
    # through our rospy compatibility layer, we just need to prevent it from
    # trying to create its own InterbotixRobotXSCore when one already exists
    
    try:
        # Import after our rospy compatibility is set up
        from widowx_controller import widowx_controller
        
        # Store the original ModifiedInterbotixManipulatorXS
        OriginalModifiedInterbotixManipulatorXS = widowx_controller.ModifiedInterbotixManipulatorXS
        
        class NodeCompatibleModifiedInterbotixManipulatorXS:
            """Wrapper that uses the existing interbotix node instead of creating a new one."""
            
            def __init__(self, robot_model, group_name="arm", gripper_name="gripper", robot_name=None, moving_time=2.0, accel_time=0.3, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
                # Import the interbotix modules
                from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
                from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
                from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
                
                # Use the existing ROS 2 node instead of creating a new one
                self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node=False)
                self.arm = widowx_controller.ModifiedInterbotixArmXSInterface(self.dxl, robot_model, group_name, moving_time, accel_time)
                if gripper_name is not None:
                    self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit)
        
        # Replace the class
        widowx_controller.ModifiedInterbotixManipulatorXS = NodeCompatibleModifiedInterbotixManipulatorXS
        
        print("Applied minimal node creation patch for widowx_controller")
        
    except ImportError as e:
        print(f"Could not apply node creation patch: {e}")
        return False
    
    return True

# Apply the patch
if not patch_node_creation():
    print("Node creation patch failed, falling back to comprehensive patch")
    try:
        import widowx_controller_full_patch
        print("Using comprehensive patch as fallback")
    except ImportError:
        print("No fallback available") 