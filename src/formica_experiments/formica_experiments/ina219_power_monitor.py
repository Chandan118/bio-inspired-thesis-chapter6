#!/usr/bin/env python3
"""
Power monitor publisher for Experiment 2.

Publishes /power_monitor as Float32MultiArray:
  data[0] = bus voltage (V)
  data[1] = current (A)
  data[2] = power (W)

On Jetson, address 0x40 is commonly managed by kernel ina3221; this node reads
sysfs hwmon first, then falls back to direct INA219 SMBus access.
"""

import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from smbus2 import SMBus
except ImportError:  # pragma: no cover - depends on target device
    SMBus = None


REG_BUS_VOLTAGE = 0x02
REG_SHUNT_VOLTAGE = 0x01


class Ina219PowerMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ina219_power_monitor")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("shunt_ohms", 0.1)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("sysfs_i2c_device", "1-0040")

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_addr = int(self.get_parameter("i2c_address").value)
        self.shunt_ohms = float(self.get_parameter("shunt_ohms").value)
        publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self.sysfs_i2c_device = str(self.get_parameter("sysfs_i2c_device").value)

        self._bus = None
        self._connected = False
        self._sysfs_ready = False
        self._sysfs_in_v = None
        self._sysfs_curr_a = None
        self._pub = self.create_publisher(Float32MultiArray, "/power_monitor", 10)
        self.create_timer(1.0 / publish_hz, self._tick)

        if SMBus is None:
            self.get_logger().error(
                "smbus2 not installed. Install with: pip3 install smbus2"
            )
        self.get_logger().info(
            f"Power monitor started (bus={self.i2c_bus}, addr=0x{self.i2c_addr:02X})"
        )
        self._init_sysfs()

    def _init_sysfs(self) -> None:
        base = Path("/sys/bus/i2c/devices") / self.sysfs_i2c_device / "hwmon"
        if not base.exists():
            return
        hwmons = sorted(base.glob("hwmon*"))
        if not hwmons:
            return
        # Choose first hwmon entry; for Jetson INA3221 this provides in1/curr1.
        h = hwmons[0]
        v = h / "in1_input"
        c = h / "curr1_input"
        if v.exists() and c.exists():
            self._sysfs_in_v = v
            self._sysfs_curr_a = c
            self._sysfs_ready = True
            self.get_logger().info(f"Using sysfs power source: {h}")

    def _read_sysfs(self):
        # in*_input on this platform is mV, curr*_input is mA.
        v_mv = float(self._sysfs_in_v.read_text().strip())
        c_ma = float(self._sysfs_curr_a.read_text().strip())
        v = v_mv / 1000.0
        a = c_ma / 1000.0
        return v, a, v * a

    def _connect_if_needed(self) -> bool:
        if self._connected and self._bus is not None:
            return True
        if SMBus is None:
            return False
        try:
            self._bus = SMBus(self.i2c_bus)
            self._connected = True
            self.get_logger().info("Connected to INA219.")
        except Exception as exc:  # pragma: no cover - hardware-dependent
            self._connected = False
            self._bus = None
            self.get_logger().warn(f"INA219 connect failed: {exc}")
        return self._connected

    def _read_u16_be(self, reg: int) -> int:
        data = self._bus.read_i2c_block_data(self.i2c_addr, reg, 2)
        return (data[0] << 8) | data[1]

    @staticmethod
    def _to_i16(v: int) -> int:
        return v - 65536 if v & 0x8000 else v

    def _tick(self) -> None:
        if self._sysfs_ready:
            try:
                bus_v, current_a, power_w = self._read_sysfs()
                if math.isfinite(bus_v) and math.isfinite(current_a) and math.isfinite(power_w):
                    msg = Float32MultiArray()
                    msg.data = [float(bus_v), float(current_a), float(power_w)]
                    self._pub.publish(msg)
                return
            except Exception as exc:
                self.get_logger().warn(f"Sysfs read failed, fallback to SMBus: {exc}")
                self._sysfs_ready = False

        if not self._connect_if_needed():
            return
        try:
            raw_bus = self._read_u16_be(REG_BUS_VOLTAGE)
            raw_shunt = self._to_i16(self._read_u16_be(REG_SHUNT_VOLTAGE))

            # INA219 conversions:
            # bus voltage register uses bits [15:3], LSB = 4 mV
            bus_v = ((raw_bus >> 3) * 0.004)
            # shunt voltage register LSB = 10 uV
            shunt_v = raw_shunt * 10e-6
            current_a = shunt_v / max(self.shunt_ohms, 1e-6)
            power_w = bus_v * current_a

            if not (math.isfinite(bus_v) and math.isfinite(current_a) and math.isfinite(power_w)):
                return

            msg = Float32MultiArray()
            msg.data = [float(bus_v), float(current_a), float(power_w)]
            self._pub.publish(msg)
        except Exception as exc:  # pragma: no cover - hardware-dependent
            self.get_logger().warn(f"INA219 read failed: {exc}")
            self._connected = False
            if self._bus is not None:
                try:
                    self._bus.close()
                except Exception:
                    pass
                self._bus = None


def main(args=None):
    rclpy.init(args=args)
    node = Ina219PowerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

