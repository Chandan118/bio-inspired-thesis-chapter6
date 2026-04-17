#!/usr/bin/env python3
import glob
import os
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String, Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import time
import math


def _arduino_candidate_ports(preferred: str) -> list:
    """Same discovery order as arduino/arduino_motor_control.py (CH341 USB1 before USB0)."""
    preferred = (preferred or '').strip()
    if preferred:
        return [preferred]
    env = os.environ.get('ARDUINO_PORT', '').strip()
    if env:
        return [env]
    fixed = [
        '/dev/ttyCH341USB1',
        '/dev/ttyCH341USB0',
        '/dev/ttyACM0',
        '/dev/ttyACM1',
        '/dev/ttyUSB1',
        '/dev/ttyUSB0',
    ]
    extra = sorted(glob.glob('/dev/ttyCH341USB*'))
    seen = set()
    out = []
    for p in fixed + extra:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out

class ArduinoBaseController(Node):
    def __init__(self):
        super().__init__('arduino_base_controller')
        
        # Parameters: port '' → scan CH341USB1 before USB0 (matches arduino_motor_control.py)
        self.declare_parameter('port', '')
        self.declare_parameter('baud_rate', 115200)
        # Scale AVR 10-bit line ADC (0–1023) to thesis-style 0–4095 for /line_sensors
        self.declare_parameter('line_adc_input_max', 1023.0)
        self.declare_parameter('line_adc_scale_to', 4095.0)
        # Below discrete F/L/R thresholds; keep below ~0.25*typical cmd_vel so filtered twist clears deadband quickly.
        self.declare_parameter('linear_deadband', 0.03)
        self.declare_parameter('angular_deadband', 0.08)
        self.declare_parameter('subscribe_cmd_vel_nav', True)

        self.port = str(self.get_parameter('port').value).strip()
        self.baud_rate = self.get_parameter('baud_rate').value
        self._line_in_max = float(self.get_parameter('line_adc_input_max').value)
        self._line_scale_to = float(self.get_parameter('line_adc_scale_to').value)
        self._linear_deadband = float(self.get_parameter('linear_deadband').value)
        self._angular_deadband = float(self.get_parameter('angular_deadband').value)
        self._sub_nav = bool(self.get_parameter('subscribe_cmd_vel_nav').value)

        # Serial connection
        self.arduino = None
        self._connected_port = ''
        self._serial_lock = threading.Lock()
        self.connect_to_arduino()
        
        # Dummy Odometry integration state
        self.dummy_x = 0.0
        self.dummy_y = 0.0
        self.dummy_yaw = 0.0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.last_integration_time = time.time()
        self.first_cmd_vel_received = False
        self._raw_log_count = 0
        self._filtered_linear = 0.0
        self._filtered_angular = 0.0
        self._last_cmd_char = 'S'
        self._last_cmd_sent_time = 0.0
        self._have_real_line_sensors = False  # True once we parse 4 TCRT values from firmware

        # Use a ReentrantCallbackGroup to allow parallel execution of callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribers and Publishers
        # Match navigation-style cmd_vel QoS (often best-effort); int-depth QoS can fail to match.
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self._cmd_nav_sub = None
        if self._sub_nav:
            self._cmd_nav_sub = self.create_subscription(
                Twist,
                'cmd_vel_nav',
                self.cmd_vel_callback,
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
        self.distance_pub = self.create_publisher(Float32, 'sensor/distance', 10)
        self.gas_pub = self.create_publisher(Float32, '/gas_sensor', 10)

        # Add required topics for experiments
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.line_pub = self.create_publisher(Float32MultiArray, '/line_sensors', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for reading sensors (isolated in the callback group)
        self.sensor_timer = self.create_timer(
            0.1, self.read_sensor_loop,
            callback_group=self.callback_group)
        
        # Dedicated fast timer for dummy ROS topics so they never block
        self.dummy_pub_timer = self.create_timer(
            0.05, self.publish_dummies_loop,
            callback_group=self.callback_group)
        
        self.get_logger().info('Arduino Base Controller started.')

    def _scale_line_adc(self, raw_vals: list) -> list:
        if self._line_in_max <= 0:
            return raw_vals
        scale = self._line_scale_to / self._line_in_max
        return [float(v) * scale for v in raw_vals]

    def connect_to_arduino(self):
        ports = _arduino_candidate_ports(self.port)
        last_err = None
        for p in ports:
            if not os.path.exists(p):
                continue
            try:
                self.arduino = serial.Serial(p, self.baud_rate, timeout=1)
                time.sleep(2)
                self._connected_port = p
                self.get_logger().info(f'Connected to Arduino on {p}')
                return
            except Exception as e:
                last_err = e
                self.arduino = None
        self.get_logger().error(
            f'Could not open any Arduino port (tried {ports}). '
            f'Set param port:=/dev/tty... or env ARDUINO_PORT. Last error: {last_err}'
        )

    def send_command(self, cmd):
        if not (self.arduino and self.arduino.is_open):
            return
        try:
            with self._serial_lock:
                # Firmware (arduino/firmware + arduino_motor_ultrasonic_combined): one char per command, B = backward.
                self.arduino.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Error writing to Arduino: {e}')

    def cmd_vel_callback(self, msg: Twist):
        if not self.first_cmd_vel_received:
            self.get_logger().info('>>> Received first cmd_vel! Movement integration active.')
            self.first_cmd_vel_received = True
            
        # Low-pass filter cmd_vel to reduce abrupt command flipping.
        alpha = 0.25
        self._filtered_linear = (1.0 - alpha) * self._filtered_linear + alpha * msg.linear.x
        self._filtered_angular = (1.0 - alpha) * self._filtered_angular + alpha * msg.angular.z

        # Store filtered values for odometry integration.
        self.last_linear_vel = self._filtered_linear
        self.last_angular_vel = self._filtered_angular

        linear = self._filtered_linear
        angular = self._filtered_angular
        abs_linear = abs(linear)
        abs_angular = abs(angular)
        lb = self._linear_deadband
        ab = self._angular_deadband

        # Prefer forward/backward motion unless turn demand is dominant.
        if abs_linear < lb and abs_angular < ab:
            cmd = 'S'
        elif abs_angular > 0.45 and abs_linear < 0.08:
            cmd = 'L' if angular > 0.0 else 'R'
        elif linear >= lb:
            cmd = 'F'
        elif linear <= -lb:
            cmd = 'B'
        else:
            cmd = 'L' if angular > 0.0 else 'R'

        # Avoid saturating serial with repeated or too-frequent commands.
        now = time.time()
        min_interval_s = 0.12
        if cmd != self._last_cmd_char or (now - self._last_cmd_sent_time) >= min_interval_s:
            self.send_command(cmd)
            self._last_cmd_char = cmd
            self._last_cmd_sent_time = now

    def read_sensor_loop(self):
        if not self.arduino or not self.arduino.is_open:
            return

        line = ''
        try:
            with self._serial_lock:
                self.arduino.reset_input_buffer()
                self.arduino.write(b'V')
                time.sleep(0.1)
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            return

        if not line or line.startswith('#'):
            return

        if self._raw_log_count < 10:
            self.get_logger().info(f'RAW SERIAL FROM ARDUINO: "{line}"')
            self._raw_log_count += 1

        parts = [p.strip() for p in line.split(',')]

        try:
            s_prefix = bool(parts and parts[0].lower() == 's')
            if parts and not parts[0].replace('.', '', 1).lstrip('-').isdigit():
                parts = parts[1:]

            if not parts:
                return

            # Minimal firmware: V -> Serial.println(distance) only (no CSV).
            if len(parts) == 1:
                dist_cm = float(parts[0])
                d = Float32()
                d.data = dist_cm
                self.distance_pub.publish(d)
                return

            if s_prefix and len(parts) >= 5:
                mq2 = float(parts[0])
                mq135 = float(parts[1])
                dist_cm = float(parts[3])
                g = Float32()
                g.data = mq135
                self.gas_pub.publish(g)
                d = Float32()
                d.data = dist_cm
                self.distance_pub.publish(d)
                if len(parts) >= 8:
                    line_msg = Float32MultiArray()
                    raw4 = [float(parts[i]) for i in range(4, 8)]
                    line_msg.data = self._scale_line_adc(raw4)
                    self.line_pub.publish(line_msg)
                    self._have_real_line_sensors = True
            elif len(parts) >= 5:
                dist_cm = float(parts[0])
                d = Float32()
                d.data = dist_cm
                self.distance_pub.publish(d)
                line_msg = Float32MultiArray()
                raw4 = [float(parts[i]) for i in range(1, 5)]
                line_msg.data = self._scale_line_adc(raw4)
                self.line_pub.publish(line_msg)
                self._have_real_line_sensors = True
            else:
                return

        except (ValueError, IndexError) as e:
            if self._raw_log_count < 20:
                self.get_logger().warn(f'Parse error on line "{line}": {e}')
                self._raw_log_count += 1

    def publish_dummies_loop(self):
        # Integrate position based on last received cmd_vel (simulating encoders)
        now_time = time.time()
        dt = now_time - self.last_integration_time
        self.last_integration_time = now_time
        
        # Very simple unicycle model
        self.dummy_yaw += self.last_angular_vel * dt
        dist = self.last_linear_vel * dt
        self.dummy_x += dist * math.cos(self.dummy_yaw)
        self.dummy_y += dist * math.sin(self.dummy_yaw)
        
        # Publish dummy odometry to prevent experiments blocking
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.dummy_x
        odom.pose.pose.position.y = self.dummy_y
        
        # Convert yaw to quaternion
        qz = math.sin(self.dummy_yaw / 2.0)
        qw = math.cos(self.dummy_yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # Publish odom -> base_link TF for Nav2/local_costmap transforms
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.dummy_x
        tf_msg.transform.translation.y = self.dummy_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # Placeholder line data only before any real TCRT parse (mq+firmware without TCRT publishes none)
        if not self._have_real_line_sensors:
            line_msg = Float32MultiArray()
            line_msg.data = [0.0, 0.0, 0.0, 0.0]
            self.line_pub.publish(line_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBaseController()
    
    # Use MultiThreadedExecutor so timer callbacks don't block each other
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.send_command('S')
        if getattr(node, 'arduino', None):
            node.arduino.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
