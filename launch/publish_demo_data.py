#!/usr/bin/python3
"""
Periodically publish gps positions in a circle around a given origin as NavSatFix messages.
"""
import rclpy
import math
from rclpy.time import CONVERSION_CONSTANT
from sensor_msgs.msg import NavSatFix, NavSatStatus


def main():
    rclpy.init()

    node = rclpy.create_node('publish_demo_data')
    publisher = node.create_publisher(NavSatFix, '/fix', 2)

    origin = NavSatFix()
    origin.latitude = 48.211486027247936
    origin.longitude = 16.383982692712074

    r = 0.001
    phi = 0.0
    timer_period = 0.1
    circle_period = 10.0

    # publish status NO_FIX in [0, 0.1] of the phase
    no_fix_in_phase = 0.05
    no_fix_period = 20.0

    def timer_callback():
        nonlocal phi
        fix = NavSatFix()
        t = node.get_clock().now()
        fix.header.stamp = t.to_msg()
        fix.header.frame_id = "gps_sensor"
        fix.latitude = origin.latitude + math.cos(phi) * r
        fix.longitude = origin.longitude + math.sin(phi) * r
        node.get_logger().info(
            f'Publishing NavSatFix at ({fix.latitude:.5f}, {fix.longitude:.5f})')
        phi += math.tau * (timer_period / circle_period)
        t_sec = t.nanoseconds / CONVERSION_CONSTANT
        no_fix_current_phase = math.fmod(t_sec, no_fix_period) / no_fix_period
        if no_fix_current_phase < no_fix_in_phase:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX
        publisher.publish(fix)

    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
