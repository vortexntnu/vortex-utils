import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation

def ssa(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi

def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Converts euler angles to quaternion (x, y, z, w)
    """
    rotation_matrix = Rotation.from_euler("XYZ", [roll, pitch, yaw]).as_matrix()
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
    return quaternion

def quat_to_euler(x: float, y: float, z: float, w: float) -> np.ndarray:
    """
    Converts quaternion (x, y, z, w) to euler angles
    """
    rotation_matrix = Rotation.from_quat([x, y, z, w]).as_matrix()
    euler_angles = Rotation.from_matrix(rotation_matrix).as_euler("XYZ")
    return euler_angles

@dataclass(slots=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def __add__(self, other: "Pose") -> "Pose":
        return Pose(**{field.name: getattr(self, field.name) + getattr(other, field.name) for field in self.__dataclass_fields__.values()})

    def __sub__(self, other: "Pose") -> "Pose":
        return Pose(**{field.name: getattr(self, field.name) - getattr(other, field.name) for field in self.__dataclass_fields__.values()})
    
    def __mul__(self, other: float) -> "Pose":
        if isinstance(other, Pose):
            return Pose(**{field.name: getattr(self, field.name) * getattr(other, field.name) for field in self.__dataclass_fields__.values()})
        return Pose(**{field.name: getattr(self, field.name) * other for field in self.__dataclass_fields__.values()})
    
    def as_rotation_matrix(self) -> np.ndarray:
        euler_angles = [self.roll, self.pitch, self.yaw]
        quat = euler_to_quat(*euler_angles)
        rotation_matrix = Rotation.from_quat(quat).as_matrix()
        return rotation_matrix

@dataclass(slots=True)
class Twist:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0
    
    def __add__(self, other: "Twist") -> "Twist":
        return Twist(**{field.name: getattr(self, field.name) + getattr(other, field.name) for field in self.__dataclass_fields__.values()})

    def __sub__(self, other: "Twist") -> "Twist":
        return Twist(**{field.name: getattr(self, field.name) - getattr(other, field.name) for field in self.__dataclass_fields__.values()})
    
    def __mul__(self, other: float) -> "Twist":
        if isinstance(other, Twist): 
            return Twist(**{field.name: getattr(self, field.name) * getattr(other, field.name) for field in self.__dataclass_fields__.values()})
        return Twist(**{field.name: getattr(self, field.name) * other for field in self.__dataclass_fields__.values()})
    
@dataclass(slots=True)
class State:
    pose: Pose
    twist: Twist

    def __add__(self, other: "State") -> "State":
        return State(pose=self.pose + other.pose, twist=self.twist + other.twist)

    def __sub__(self, other: "State") -> "State":
        return State(pose=self.pose - other.pose, twist=self.twist - other.twist)
