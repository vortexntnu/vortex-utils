from python_utils import Pose, Twist, quat_to_euler
from geometry_msgs.msg import Pose as PoseMsg
from geometry_msgs.msg import Twist as TwistMsg

def pose_from_ros(pose_msg: PoseMsg) -> Pose:
    quat = pose_msg.orientation
    euler_angles = quat_to_euler(quat.x, quat.y, quat.z, quat.w)
    return Pose(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z, *euler_angles)

def twist_from_ros(twist_msg: TwistMsg) -> Twist:
    return Twist(twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z, twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z)