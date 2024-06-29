from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    BIDIRECTIONAL = 0
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2


DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: '@',
    BridgeDirection.GZ_TO_ROS: '[',
    BridgeDirection.ROS_TO_GZ: ']',
}

@dataclass
class Bridge:
    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        out = f'{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}'
        return out

    def remapping(self):
        return (self.gz_topic, self.ros_topic)
    

def image():
    return Bridge(
        gz_topic=f'/camera',
        ros_topic=f'/camera',
        gz_type='ignition.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)

def camera_info():
    return Bridge(
        gz_topic=f'/camera_info',
        ros_topic=f'/camera_info',
        gz_type='ignition.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS)