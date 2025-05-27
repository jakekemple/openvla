"""Utils for evaluating policies in real-world BridgeData V2 environments."""

import os
import sys
import time
import threading
from queue import Queue

# Apply ROS 2 patches before importing any widowx modules
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import widowx_controller_node_patch
    print("Applied minimal node creation patch for widowx_controller")
except ImportError as e:
    print(f"Warning: Could not apply widowx_controller patches: {e}")

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as ROS2Image, JointState as ROS2JointState
    from geometry_msgs.msg import TransformStamped as ROS2TransformStamped
    from builtin_interfaces.msg import Time as ROS2Time
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS 2 not available, using mock implementations")

# Add bridge_data_robot to path FIRST
bridge_data_path = "/mnt/nvme/cyberlimb-bridge/bridge_data_robot"
if bridge_data_path not in sys.path:
    sys.path.insert(0, bridge_data_path)

# Add widowx_controller source path
widowx_controller_path = "/mnt/nvme/cyberlimb-bridge/bridge_data_robot/widowx_envs/widowx_controller/src"
if widowx_controller_path not in sys.path:
    sys.path.insert(0, widowx_controller_path)

# Add widowx_envs path
widowx_envs_path = "/mnt/nvme/cyberlimb-bridge/bridge_data_robot/widowx_envs"
if widowx_envs_path not in sys.path:
    sys.path.insert(0, widowx_envs_path)

# Global ROS 2 node for sharing across modules
_global_ros2_node = None
_ros2_executor = None
_ros2_thread = None

def get_ros2_node():
    """Get or create the global ROS 2 node."""
    global _global_ros2_node, _ros2_executor, _ros2_thread
    
    if _global_ros2_node is None and ROS2_AVAILABLE:
        if not rclpy.ok():
            rclpy.init()
        
        _global_ros2_node = Node('widowx_bridge_node')
        
        # Create executor and run in thread
        _ros2_executor = rclpy.executors.SingleThreadedExecutor()
        _ros2_executor.add_node(_global_ros2_node)
        
        def spin():
            while rclpy.ok():
                _ros2_executor.spin_once(timeout_sec=0.01)
        
        _ros2_thread = threading.Thread(target=spin, daemon=True)
        _ros2_thread.start()
        
        print("Created ROS 2 node: widowx_bridge_node")
    
    return _global_ros2_node

# Setup interbotix compatibility FIRST
try:
    # Add interbotix to path if not already there
    interbotix_path = "/home/jake/interbotix_ws/install/interbotix_xs_modules/lib/python3.10/site-packages"
    if interbotix_path not in sys.path:
        sys.path.insert(0, interbotix_path)
        
    from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
    from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore  
    from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
    
    # Create the compatibility module for interbotix_xs_modules.arm
    interbotix_arm_compat = type('Module', (), {
        'InterbotixArmXSInterface': InterbotixArmXSInterface,
        'InterbotixRobotXSCore': InterbotixRobotXSCore,
        'InterbotixGripperXSInterface': InterbotixGripperXSInterface
    })()
    
    sys.modules['interbotix_xs_modules.arm'] = interbotix_arm_compat
    print("Successfully set up interbotix compatibility")
    
except ImportError as e:
    print(f"Warning: Could not import interbotix modules: {e}")
    print("Make sure you have sourced your ROS 2 workspace")

# Enhanced rospy compatibility with ROS 2 support
class RospyCompat:
    @staticmethod
    def init_node(name, **kwargs):
        """Initialize ROS node (no-op for ROS 2 as we use global node)"""
        print(f"[INFO] rospy.init_node called with name: {name}")
        get_ros2_node()  # Ensure node is created
        
    @staticmethod
    def get_time():
        """Get current time as float"""
        if ROS2_AVAILABLE:
            node = get_ros2_node()
            if node:
                now = node.get_clock().now()
                return now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] / 1e9
        return time.time()
    
    @staticmethod
    def sleep(duration):
        """Sleep for specified duration"""
        time.sleep(duration)
        
    @staticmethod
    def loginfo(msg):
        """Log info message"""
        print(f"[INFO] {msg}")
        
    @staticmethod
    def on_shutdown(callback):
        """Register shutdown callback (not implemented for ROS 2)"""
        pass
        
    @staticmethod
    def wait_for_service(service_name):
        """Wait for service (simplified for ROS 2)"""
        print(f"[INFO] Would wait for service: {service_name}")
        
    @staticmethod
    def ServiceProxy(service_name, service_type):
        """Create service proxy (mock for now)"""
        class DummyProxy:
            def __call__(self, *args, **kwargs):
                print(f"[INFO] Service call to {service_name}")
                return None
        return DummyProxy()
        
    @staticmethod
    def Subscriber(topic, msg_type, callback):
        """Create ROS 2 compatible subscriber"""
        if ROS2_AVAILABLE:
            node = get_ros2_node()
            if node:
                # Map ROS 1 message types to ROS 2
                if msg_type.__name__ == 'JointState':
                    ros2_msg_type = ROS2JointState
                elif msg_type.__name__ == 'Image':
                    ros2_msg_type = ROS2Image
                else:
                    # For other types, try to use as-is
                    ros2_msg_type = msg_type
                
                return node.create_subscription(ros2_msg_type, topic, callback, 10)
        
        print(f"[INFO] Would subscribe to {topic}")
        return None
        
    class Time:
        @staticmethod
        def now():
            """Get current ROS time"""
            current_time = RospyCompat.get_time()
            
            class TimeStamp:
                def __init__(self, t):
                    if isinstance(t, (int, float)):
                        self.secs = int(t)
                        self.nsecs = int((t % 1) * 1e9)
                    else:
                        # Handle ROS 2 Time objects
                        self.secs = t.sec if hasattr(t, 'sec') else int(t)
                        self.nsecs = t.nanosec if hasattr(t, 'nanosec') else int((t % 1) * 1e9)
                    
                def to_sec(self):
                    return self.secs + self.nsecs / 1e9
                
                def __sub__(self, other):
                    """Enable subtraction between timestamps"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(self.to_sec() - other.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp(self.to_sec() - (other.secs + other.nsecs / 1e9))
                    else:
                        return TimeStamp(self.to_sec() - float(other))
                
                def __rsub__(self, other):
                    """Enable reverse subtraction"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(other.to_sec() - self.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp((other.secs + other.nsecs / 1e9) - self.to_sec())
                    else:
                        return TimeStamp(float(other) - self.to_sec())
                    
            return TimeStamp(current_time)
    
    @staticmethod
    def is_shutdown():
        """Check if ROS is shutdown"""
        if ROS2_AVAILABLE:
            return not rclpy.ok()
        return False
            
    class ServiceException(Exception):
        pass
        
    # Add service submodule
    service = type('Module', (), {'ServiceException': ServiceException})()

sys.modules['rospy'] = RospyCompat()
sys.modules['rospy.service'] = RospyCompat.service

# Setup other compatibility modules
import numpy as np

# Mock tf2_ros for ROS 2
class TransformBroadcaster:
    def __init__(self):
        if ROS2_AVAILABLE:
            from tf2_ros import TransformBroadcaster as ROS2TransformBroadcaster
            node = get_ros2_node()
            if node:
                self._broadcaster = ROS2TransformBroadcaster(node)
            else:
                self._broadcaster = None
        else:
            self._broadcaster = None
    
    def sendTransform(self, transform):
        if self._broadcaster:
            self._broadcaster.sendTransform(transform)

tf2_ros_module = type('Module', (), {'TransformBroadcaster': TransformBroadcaster})()
sys.modules['tf2_ros'] = tf2_ros_module

# Mock transformations module (doesn't exist in ROS 2)
def quaternion_from_matrix(matrix):
    """Convert rotation matrix to quaternion."""
    # Simple implementation
    trace = np.trace(matrix[:3, :3])
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
    else:
        if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            w = (matrix[2, 1] - matrix[1, 2]) / s
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
        elif matrix[1, 1] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            w = (matrix[0, 2] - matrix[2, 0]) / s
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            w = (matrix[1, 0] - matrix[0, 1]) / s
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s
    return np.array([w, x, y, z])
    
transformations = type('Module', (), {
    'quaternion_from_matrix': quaternion_from_matrix
})()
sys.modules['transformations'] = transformations

# Enhanced multicam_server mock with ROS 2 support
class IMTopic:
    def __init__(self, name, width=640, height=480, top=0, bot=0, right=0, left=0, dtype="bgr8", flip=False, info_name=None):
        self.name = name
        self.width = width
        self.height = height
        self.top = top
        self.bot = bot
        self.right = right
        self.left = left
        self.dtype = dtype
        self.flip = flip
        self.info_name = info_name
    
    def process_image(self, img):
        import cv2
        if (self.height, self.width) != img.shape[:2]:
            return cv2.resize(img, (self.width, self.height), interpolation=cv2.INTER_AREA)
        return img

    @classmethod
    def from_dict(cls, data):
        return cls(**data)

class CameraRecorder:
    def __init__(self, topic_data, opencv_tracking=False, save_videos=False):
        # Handle both dict and IMTopic objects
        if isinstance(topic_data, dict):
            # Convert dict to IMTopic object
            self.topic_data = IMTopic(**topic_data)
        else:
            self.topic_data = topic_data
            
        self._cam_width = 640
        self._cam_height = 480
        self.camera_info = None
        self._latest_image = None
        self._latest_timestamp = None
        self._image_queue = Queue(maxsize=10)
        
        # Subscribe to camera topic if ROS 2 is available
        if ROS2_AVAILABLE:
            node = get_ros2_node()
            if node:
                print(f"Subscribing to camera topic: {self.topic_data.name}")
                self._sub = node.create_subscription(
                    ROS2Image,
                    self.topic_data.name,
                    self._image_callback,
                    10
                )
    
    def _image_callback(self, msg):
        """ROS 2 image callback."""
        try:
            # Convert ROS 2 image to numpy array
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                img = img[:, :, ::-1]  # Convert BGR to RGB
            else:
                print(f"Unsupported image encoding: {msg.encoding}")
                return
            
            # Process image according to topic settings
            if self.topic_data.flip:
                img = np.fliplr(img)
            
            img = self.topic_data.process_image(img)
            
            # Store latest image
            self._latest_image = img
            self._latest_timestamp = msg.header.stamp
            
            # Try to add to queue (drop old images if full)
            try:
                self._image_queue.put_nowait((self._latest_timestamp, img))
            except:
                # Queue is full, drop oldest
                try:
                    self._image_queue.get_nowait()
                    self._image_queue.put_nowait((self._latest_timestamp, img))
                except:
                    pass
                    
        except Exception as e:
            print(f"Error in image callback: {e}")
    
    def get_image(self):
        """Get the latest image."""
        if self._latest_image is not None:
            # Convert ROS 2 timestamp to compatible format
            stamp = self._latest_timestamp
            
            class TimeStamp:
                def __init__(self, ros2_stamp):
                    if hasattr(ros2_stamp, 'sec'):
                        self.secs = ros2_stamp.sec
                        self.nsecs = ros2_stamp.nanosec
                    else:
                        # Handle float timestamps
                        self.secs = int(ros2_stamp)
                        self.nsecs = int((ros2_stamp % 1) * 1e9)
                    
                def to_sec(self):
                    return self.secs + self.nsecs / 1e9
                
                def __sub__(self, other):
                    """Enable subtraction between timestamps"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(self.to_sec() - other.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp(self.to_sec() - (other.secs + other.nsecs / 1e9))
                    else:
                        return TimeStamp(self.to_sec() - float(other))
                
                def __rsub__(self, other):
                    """Enable reverse subtraction"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(other.to_sec() - self.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp((other.secs + other.nsecs / 1e9) - self.to_sec())
                    else:
                        return TimeStamp(float(other) - self.to_sec())
            
            return TimeStamp(stamp), self._latest_image
        else:
            # Return dummy image if no real image available
            current_time = time.time()
            
            class TimeStamp:
                def __init__(self, t):
                    if isinstance(t, (int, float)):
                        self.secs = int(t)
                        self.nsecs = int((t % 1) * 1e9)
                    else:
                        self.secs = t.sec if hasattr(t, 'sec') else int(t)
                        self.nsecs = t.nanosec if hasattr(t, 'nanosec') else int((t % 1) * 1e9)
                    
                def to_sec(self):
                    return self.secs + self.nsecs / 1e9
                
                def __sub__(self, other):
                    """Enable subtraction between timestamps"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(self.to_sec() - other.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp(self.to_sec() - (other.secs + other.nsecs / 1e9))
                    else:
                        return TimeStamp(self.to_sec() - float(other))
                
                def __rsub__(self, other):
                    """Enable reverse subtraction"""
                    if hasattr(other, 'to_sec'):
                        return TimeStamp(other.to_sec() - self.to_sec())
                    elif hasattr(other, 'secs') and hasattr(other, 'nsecs'):
                        return TimeStamp((other.secs + other.nsecs / 1e9) - self.to_sec())
                    else:
                        return TimeStamp(float(other) - self.to_sec())
            
            dummy_image = np.random.randint(0, 255, (self._cam_height, self._cam_width, 3), dtype=np.uint8)
            return TimeStamp(current_time), dummy_image
    
    @property
    def img_width(self):
        return self._cam_width
    
    @property
    def img_height(self):
        return self._cam_height

topic_utils = type('Module', (), {'IMTopic': IMTopic})()
camera_recorder = type('Module', (), {'CameraRecorder': CameraRecorder})()
multicam_server = type('Module', (), {
    'topic_utils': topic_utils,
    'camera_recorder': camera_recorder
})()

sys.modules['multicam_server'] = multicam_server
sys.modules['multicam_server.topic_utils'] = topic_utils
sys.modules['multicam_server.camera_recorder'] = camera_recorder

# Mock widowx_controller services
class MockService:
    Request = type('Request', (), {})()
    Response = type('Response', (), {})()
    
widowx_srv = type('Module', (), {
    'OpenGripper': MockService,
    'OpenGripperResponse': MockService.Response,
    'GetGripperDesiredState': MockService,
    'GetGripperDesiredStateResponse': MockService.Response,
    'SetGripperPosition': MockService,
    'SetGripperPositionResponse': MockService.Response,
})()
sys.modules['widowx_controller.srv'] = widowx_srv

print("ROS 1 to ROS 2 compatibility layer initialized")

# Now import the rest
import imageio
import tensorflow as tf
import torch
from widowx_envs.widowx_env_service import WidowXConfigs

sys.path.append(".")

# Initialize important constants and pretty-printing mode in NumPy.
ACTION_DIM = 7
BRIDGE_PROPRIO_DIM = 7
DATE_TIME = time.strftime("%Y_%m_%d-%H_%M_%S")
DEVICE = torch.device("cuda:0") if torch.cuda.is_available() else torch.device("cpu")
np.set_printoptions(formatter={"float": lambda x: f"{x:0.2f}"})


def get_widowx_env_params(cfg):
    """Gets (mostly default) environment parameters for the WidowX environment."""
    env_params = WidowXConfigs.DefaultEnvParams.copy()
    env_params["override_workspace_boundaries"] = cfg.bounds
    env_params["camera_topics"] = cfg.camera_topics
    env_params["return_full_image"] = True
    
    # Override motion parameters for much smoother movement
    env_params["move_duration"] = 1.5  # Much slower: 1.5 seconds instead of 0.2
    
    return env_params


def get_widowx_env(cfg, model=None):
    """Get WidowX control environment - Uses the actual widowx_envs code with ROS 2 compatibility."""
    print("Initializing WidowX environment...")
    
    try:
        print("Attempting to use actual widowx_envs...")
        # Import the actual widowx_envs
        from widowx_envs.widowx_env import FinetuningBridgeDataWidowX
        
        # Get environment parameters
        env_params = get_widowx_env_params(cfg)
        
        # Create the actual environment used during training
        env = FinetuningBridgeDataWidowX(env_params)
        print("Successfully created FinetuningBridgeDataWidowX environment")
        return env
        
    except Exception as e:
        print(f"Failed to create actual widowx_envs environment: {e}")
        import traceback
        traceback.print_exc()
        return None


def get_next_task_label(task_label):
    """Prompt the user to input the next task."""
    if task_label == "":
        user_input = ""
        while user_input == "":
            user_input = input("Enter the task name: ")
        task_label = user_input
    else:
        user_input = input("Enter the task name (or leave blank to repeat the previous task): ")
        if user_input == "":
            pass  # Do nothing -> Let task_label be the same
        else:
            task_label = user_input
    print(f"Task: {task_label}")
    return task_label


def save_rollout_video(rollout_images, idx):
    """Saves an MP4 replay of an episode."""
    os.makedirs("./rollouts", exist_ok=True)
    mp4_path = f"./rollouts/rollout-{DATE_TIME}-{idx+1}.mp4"
    video_writer = imageio.get_writer(mp4_path, fps=5)
    
    for img in rollout_images:
        # Ensure img is a numpy array
        if not isinstance(img, np.ndarray):
            img = np.array(img)
        
        # Handle different image shapes
        if len(img.shape) == 4:
            # Remove batch dimension if present
            img = img[0]
        elif len(img.shape) == 2:
            # Convert grayscale to RGB
            img = np.stack([img, img, img], axis=-1)
        
        # Ensure image is 3D (H, W, C) with 3 channels
        if len(img.shape) != 3 or img.shape[2] not in [1, 3, 4]:
            print(f"Warning: Skipping image with invalid shape {img.shape}")
            continue
        
        # Ensure image is uint8
        if img.dtype != np.uint8:
            if img.max() <= 1.0:
                img = (img * 255).astype(np.uint8)
            else:
                img = img.astype(np.uint8)
        
        # Convert single channel to RGB if needed
        if img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)
        
        video_writer.append_data(img)
    
    video_writer.close()
    print(f"Saved rollout MP4 at path {mp4_path}")


def save_rollout_data(rollout_orig_images, rollout_images, rollout_states, rollout_actions, idx):
    """
    Saves rollout data from an episode.

    Args:
        rollout_orig_images (list): Original rollout images (before preprocessing).
        rollout_images (list): Preprocessed images.
        rollout_states (list): Proprioceptive states.
        rollout_actions (list): Predicted actions.
        idx (int): Episode index.
    """
    os.makedirs("./rollouts", exist_ok=True)
    path = f"./rollouts/rollout-{DATE_TIME}-{idx+1}.npz"
    # Convert lists to numpy arrays
    orig_images_array = np.array(rollout_orig_images)
    images_array = np.array(rollout_images)
    states_array = np.array(rollout_states)
    actions_array = np.array(rollout_actions)
    # Save to a single .npz file
    np.savez(path, orig_images=orig_images_array, images=images_array, states=states_array, actions=actions_array)
    print(f"Saved rollout data at path {path}")


def resize_image(img, resize_size):
    """
    Takes numpy array corresponding to a single image and returns resized image as numpy array.

    NOTE (Moo Jin): To make input images in distribution with respect to the inputs seen at training time, we follow
                    the same resizing scheme used in the Octo dataloader, which OpenVLA uses for training.
    """
    assert isinstance(resize_size, tuple)
    
    # Ensure img is a numpy array
    if not isinstance(img, np.ndarray):
        img = np.array(img)
    
    # Handle different image shapes
    if len(img.shape) == 4:
        # Remove batch dimension if present
        img = img[0]
    elif len(img.shape) == 2:
        # Convert grayscale to RGB
        img = np.stack([img, img, img], axis=-1)
    
    # Ensure image is 3D (H, W, C)
    if len(img.shape) != 3:
        raise ValueError(f"Image must be 3D after preprocessing, got shape {img.shape}")
    
    # Ensure image is uint8
    if img.dtype != np.uint8:
        if img.max() <= 1.0:
            img = (img * 255).astype(np.uint8)
        else:
            img = img.astype(np.uint8)
    
    # Resize to image size expected by model
    img_tensor = tf.constant(img)
    img = tf.image.encode_jpeg(img_tensor)  # Encode as JPEG, as done in RLDS dataset builder
    img = tf.io.decode_image(img, expand_animations=False, dtype=tf.uint8)  # Immediately decode back
    img = tf.image.resize(img, resize_size, method="lanczos3", antialias=True)
    img = tf.cast(tf.clip_by_value(tf.round(img), 0, 255), tf.uint8)
    img = img.numpy()
    return img


def get_preprocessed_image(obs, resize_size):
    """Extracts image from observations and preprocesses it."""
    assert isinstance(resize_size, int) or isinstance(resize_size, tuple)
    if isinstance(resize_size, int):
        resize_size = (resize_size, resize_size)
    obs["full_image"] = resize_image(obs["full_image"], resize_size)
    return obs["full_image"]


def refresh_obs(obs, env):
    """Fetches new observations from the environment and updates the current observations."""
    # Try different method names based on the environment
    if hasattr(env, 'get_obs'):
        new_obs = env.get_obs()
    elif hasattr(env, 'get_observation'):
        new_obs = env.get_observation()
    elif hasattr(env, 'current_obs'):
        new_obs = env.current_obs()
    else:
        raise AttributeError(f"Environment {type(env)} doesn't have get_obs, get_observation, or current_obs method")
    
    # Update observations based on what's available in new_obs
    if "full_image" in new_obs:
        obs["full_image"] = new_obs["full_image"]
    elif "full_image_observation" in new_obs:
        obs["full_image"] = new_obs["full_image_observation"]
        
    if "image_primary" in new_obs:
        obs["image_primary"] = new_obs["image_primary"]
    elif "image_observation" in new_obs:
        obs["image_primary"] = new_obs["image_observation"]
        
    if "proprio" in new_obs:
        obs["proprio"] = new_obs["proprio"]
    elif "state" in new_obs:
        obs["proprio"] = new_obs["state"]
        
    return obs 