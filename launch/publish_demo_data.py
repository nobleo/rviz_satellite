#!/usr/bin/python3
"""
Periodically publish gps positions in a circle around a given origin as NavSatFix messages.
"""
import rclpy
import math
import numpy as np
from argparse import ArgumentParser
from rclpy.time import CONVERSION_CONSTANT
from sensor_msgs.msg import NavSatFix, NavSatStatus


class CircularTranslate():
    """Translate fix periodically in a circle"""

    def update(self, t, fix):
        r = 0.001
        circle_period = 10.0
        t_sec = t.nanoseconds / CONVERSION_CONSTANT
        phi = math.tau * (t_sec / circle_period)
        fix.latitude += math.cos(phi) * r
        fix.longitude += math.sin(phi) * r


class ApplyNoise():
    """Translate fix position using gaussian noise"""

    def update(self, t, fix):
        stddev = 0.01
        noise = np.random.normal(0, stddev, 2)
        fix.latitude += noise[0]
        fix.longitude += noise[1]


class LooseFix():
    """Set NavSatFix status to NO_FIX periodically"""

    def update(self, t, fix):
        no_fix_in_phase = 0.2
        no_fix_period = 5.0
        t_sec = t.nanoseconds / CONVERSION_CONSTANT
        no_fix_current_phase = math.fmod(t_sec, no_fix_period) / no_fix_period
        if no_fix_current_phase < no_fix_in_phase:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX


def main():
    parser = ArgumentParser('Periodically create and publish NavSatFix')
    parser.add_argument('latitude', type=float)
    parser.add_argument('longitude', type=float)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('publish_demo_data')
    publisher = node.create_publisher(NavSatFix, '/fix', 2)

    origin = NavSatFix()
    origin.latitude = args.latitude
    origin.longitude = args.longitude

    updaters = [CircularTranslate(), LooseFix()]

    def timer_callback():
        fix = NavSatFix()
        t = node.get_clock().now()
        fix.header.stamp = t.to_msg()
        fix.header.frame_id = "gps_sensor"
        fix.latitude = origin.latitude
        fix.longitude = origin.longitude
        for u in updaters:
            u.update(t, fix)
        node.get_logger().info(
            f'Publishing NavSatFix at ({fix.latitude:.5f}, {fix.longitude:.5f})')
        publisher.publish(fix)

    timer = node.create_timer(0.1, timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
