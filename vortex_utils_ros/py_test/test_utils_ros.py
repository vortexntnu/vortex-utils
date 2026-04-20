import pytest
from geometry_msgs.msg import Pose, Twist
from vortex_utils.python_utils import quat_to_euler
from vortex_utils_ros.ros_converter import pose_from_ros, twist_from_ros


def test_pose_from_ros():
    pose_msg = Pose()
    pose_msg.position.x = 1.0
    pose_msg.position.y = 2.0
    pose_msg.position.z = 3.0
    pose_msg.orientation.x = 0.1
    pose_msg.orientation.y = 0.2
    pose_msg.orientation.z = 0.3
    pose_msg.orientation.w = 0.4
    euler = quat_to_euler(0.1, 0.2, 0.3, 0.4)
    pose = pose_from_ros(pose_msg)
    assert pose.x == 1.0
    assert pose.y == 2.0
    assert pose.z == 3.0
    assert pose.roll == pytest.approx(euler[0], abs=0.01)
    assert pose.pitch == pytest.approx(euler[1], abs=0.01)
    assert pose.yaw == pytest.approx(euler[2], abs=0.01)


def test_twist_from_ros():
    twist_msg = Twist()
    twist_msg.linear.x = 1.0
    twist_msg.linear.y = 2.0
    twist_msg.linear.z = 3.0
    twist_msg.angular.x = 0.1
    twist_msg.angular.y = 0.2
    twist_msg.angular.z = 0.3
    twist = twist_from_ros(twist_msg)
    assert twist.linear_x == 1.0
    assert twist.linear_y == 2.0
    assert twist.linear_z == 3.0
    assert twist.angular_x == 0.1
    assert twist.angular_y == 0.2
    assert twist.angular_z == 0.3
