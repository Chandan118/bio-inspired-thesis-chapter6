#!/usr/bin/env python3
"""
ina219_power_monitor.py - ROS 1 Version
========================================
Power monitoring node using INA219 sensor on I2C bus.

Run:
    rosrun formica_experiments ina219_power_monitor.py
"""

import rospy
from std_msgs.msg import Float32
import smbus2  # pip install smbus2


class INA219Monitor(object):
    def __init__(self):
        rospy.init_node("ina219_power_monitor", anonymous=True)

        self._bus = smbus2.SMBus(1)  # I2C bus 1
        self._addr = 0x40

        self._current_pub = rospy.Publisher("/ina219/current", Float32, queue_size=10)
        self._voltage_pub = rospy.Publisher("/ina219/voltage", Float32, queue_size=10)

        self._rate = rospy.Rate(10.0)  # 10 Hz
        rospy.loginfo("INA219 Power Monitor started on I2C bus 1 at 0x40")

    def read_ina219(self):
        """Read current and voltage from INA219."""
        try:
            # Simplified - actual INA219 requires 16-bit register reads
            voltage = 12.0 + rospy.get_time() % 1.0  # Placeholder
            current = 1.5 + rospy.get_time() % 0.5    # Placeholder
            return voltage, current
        except Exception as e:
            rospy.logwarn("INA219 read error: %s", e)
            return 0.0, 0.0

    def run(self):
        while not rospy.is_shutdown():
            v, i = self.read_ina219()
            self._voltage_pub.publish(v)
            self._current_pub.publish(i * 1000.0)  # A to mA
            self._rate.sleep()


def main():
    monitor = INA219Monitor()
    monitor.run()


if __name__ == "__main__":
    main()
