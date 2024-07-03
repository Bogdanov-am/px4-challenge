import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from filterpy.kalman import KalmanFilter

class TargetFollower(Node):

    def __init__(self):
        super().__init__('target_follower')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Подписка на состояние автопилота
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        # Паблишер оповещения автопилота о наличии offboard управления
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # Паблишер установки целевой траектории (в данном примере только положения)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.aruco_sub = self.create_subscription(
            PoseArray,
            '/aruco_poses',
            self.aruco_pose_callback,
            10)
        self.target = None
        
        self.declare_parameter('radius', 4.0)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('altitude', 8.0)
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
        self.new_altitude = float(self.altitude)
        
        timer_period = 0.02  # seconds
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        
        self.filter = KalmanFilter(6, 3)
        self.filter.x = np.array([0., 0., 0., 0., 0., 0.])
        self.filter.F = np.asarray(
            [
                [1., 0., 0., 1., 0., 0.],
                [0., 1., 0., 0., 1., 0.],
                [0., 0., 1., 0., 0., 1.],
                [0., 0., 0., 1., 0., 0.],
                [0., 0., 0., 0., 1., 0.],
                [0., 0., 0., 0., 0., 1.]
            ]
        )
        self.filter.H = np.array([
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 1., 0., 0., 0.]
        ])
        self.filter.P *= 1000
        self.filter.R *= 500
        from filterpy.common import Q_discrete_white_noise
        self.filter.Q = Q_discrete_white_noise(2, dt=0.05, var=5., block_size=3)
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self._logger.debug(f"NAV_STATUS: {msg.nav_state}")
        self._logger.debug(f"  - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}")
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def aruco_pose_callback(self, msg: PoseArray):
        if len(msg.poses):
            try:
                t1 = self.tf_buffer.lookup_transform('vehicle', 'x500_mono_cam_0/mono_cam/base_link/imager', rclpy.time.Time())
                target = tf2_geometry_msgs.do_transform_pose(msg.poses[0], t1)
                t2 = self.tf_buffer.lookup_transform('map', 'vehicle', rclpy.time.Time())
                target = tf2_geometry_msgs.do_transform_pose(target, t2)
               
                self.filter.predict()
                self.filter.update(np.array([target.position.x, target.position.y, target.position.z]))
                self.target = self.filter.x
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform')
        else:
            self.target = None
        
    def send_offboard_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
    
    
    def cmdloop_callback(self):
        self.send_offboard_mode()
        if self.target is not None:
            self.get_logger().debug(f'Target coordinate: {self.target[0]} {self.target[1]} {self.target[2]}')
        else:
            return    
        
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            # pass
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.target[0] + self.radius * np.cos(self.theta) * (1 - np.cos(self.theta / 10)) / 2
            trajectory_msg.position[1] = - self.target[1] + self.radius * np.sin(self.theta) * (1 - np.cos(self.theta / 10)) /2
            trajectory_msg.position[2] = - self.target[2] + -self.new_altitude
            self.publisher_trajectory.publish(trajectory_msg)

            self.new_altitude = self.new_altitude
            self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)

    target_follower = TargetFollower()

    rclpy.spin(target_follower)

    target_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
