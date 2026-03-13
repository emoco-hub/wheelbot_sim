import math
import random
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Imu

# If no Joy message arrives within this window, zero the velocity.
CMD_TIMEOUT_SEC = 0.5


class WheelbotSim(Node):

    def __init__(self):
        super().__init__('wheelbot_sim')

        self.declare_parameter('max_linear_vel', 1.0, ParameterDescriptor(
            description='Maximum linear velocity (m/s)',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)]
        ))
        self.declare_parameter('max_angular_vel', 2.0, ParameterDescriptor(
            description='Maximum angular velocity (rad/s)',
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)]
        ))
        self.declare_parameter('sim_rate', 10.0, ParameterDescriptor(
            description='Simulation update rate (Hz)',
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=100.0, step=1.0)]
        ))

        # Differential-drive state
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_theta = 0.0
        self.sim_vx = 0.0
        self.sim_wz = 0.0
        self.latest_joy = None
        self.last_joy_time = 0.0  # monotonic timestamp of last Joy msg

        # Subscription
        self.create_subscription(Joy, 'cmd_joy', self.joy_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'sim/odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'sim/imu', 10)

        # Simulation timer
        sim_rate = self.get_parameter('sim_rate').value
        self.sim_dt = 1.0 / sim_rate
        self.sim_timer = self.create_timer(self.sim_dt, self.sim_step)

        self.get_logger().info(
            f'Wheelbot sim started: rate={sim_rate} Hz, '
            f'max_v={self.get_parameter("max_linear_vel").value} m/s, '
            f'max_w={self.get_parameter("max_angular_vel").value} rad/s'
        )

    def joy_callback(self, msg):
        self.latest_joy = msg
        self.last_joy_time = time.monotonic()

    @staticmethod
    def yaw_to_quaternion(yaw):
        return Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )

    def sim_step(self):
        max_linear_vel = self.get_parameter('max_linear_vel').value
        max_angular_vel = self.get_parameter('max_angular_vel').value

        # Read joy input: axes[1] = forward, axes[3] or axes[0] = turn
        # Watchdog: zero velocity if no Joy message received recently
        cmd_stale = (time.monotonic() - self.last_joy_time) > CMD_TIMEOUT_SEC

        if not cmd_stale and self.latest_joy and len(self.latest_joy.axes) >= 2:
            self.sim_vx = self.latest_joy.axes[1] * max_linear_vel
            # Negate: positive stick X = right, but positive ω = CCW in ROS
            if len(self.latest_joy.axes) >= 4:
                self.sim_wz = -self.latest_joy.axes[3] * max_angular_vel
            else:
                self.sim_wz = -self.latest_joy.axes[0] * max_angular_vel
        else:
            self.sim_vx = 0.0
            self.sim_wz = 0.0

        # Integrate unicycle kinematics
        dt = self.sim_dt
        self.sim_theta += self.sim_wz * dt
        self.sim_x += self.sim_vx * math.cos(self.sim_theta) * dt
        self.sim_y += self.sim_vx * math.sin(self.sim_theta) * dt

        now = self.get_clock().now().to_msg()
        q = self.yaw_to_quaternion(self.sim_theta)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.sim_x
        odom.pose.pose.position.y = self.sim_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.sim_vx
        odom.twist.twist.angular.z = self.sim_wz
        self.odom_pub.publish(odom)

        # Publish IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'base_link'
        imu.orientation = q
        imu.angular_velocity = Vector3(x=0.0, y=0.0, z=self.sim_wz)
        centripetal = self.sim_vx * self.sim_wz
        noise = random.gauss(0, 0.01)
        imu.linear_acceleration = Vector3(
            x=noise,
            y=centripetal + noise,
            z=9.81 + noise,
        )
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = WheelbotSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
