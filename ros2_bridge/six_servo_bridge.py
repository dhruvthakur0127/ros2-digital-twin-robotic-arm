#!/usr/bin/env python3
"""
6-DOF Servo Bridge - FINAL VERSION
Proportional mapping with custom PCA channel assignment.

Home state calibration:
  URDF: [0.0, 0.0, -1.536, 1.449, 0.0, 0.0] radians
  Servo: [90, 100, 50, 50, 120, 120] degrees
  PCA:   [CH0, CH1, CH2, CH3, CH5, CH4]

CUSTOMIZE: Edit SERVO_CONFIG to match your robot.
See README.md Customization Guide for details.

Manual control while running:
  ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: '0 90'}"
  ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: 'h'}"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import math
import time


class SixServoBridge(Node):
    def __init__(self):
        super().__init__('six_servo_bridge')

        self.JOINT_NAMES = [
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_link6',
            'link6_to_link6_flange'
        ]

        # ══════════════════════════════════════════════════════
        # CUSTOMIZE THIS SECTION FOR YOUR ROBOT
        # ══════════════════════════════════════════════════════
        self.SERVO_CONFIG = [
            {   # CH0 - Base (link1_to_link2)
                'urdf_home': 0.0, 'servo_home': 90,
                'urdf_min': -2.879793, 'urdf_max': 2.879793,
                'servo_min': 10, 'servo_max': 170,
                'direction': 1,
                'pca_channel': 0
            },
            {   # CH1 - Shoulder (link2_to_link3)
                'urdf_home': 0.0, 'servo_home': 100,
                'urdf_min': -2.879793, 'urdf_max': 2.879793,
                'servo_min': 55, 'servo_max': 150,
                'direction': 1,
                'pca_channel': 1
            },
            {   # CH2 - Elbow (link3_to_link4)
                'urdf_home': -1.536, 'servo_home': 50,
                'urdf_min': -2.879793, 'urdf_max': 2.879793,
                'servo_min': 55, 'servo_max': 180,
                'direction': 1,
                'pca_channel': 2
            },
            {   # CH3 - Wrist Pitch (link4_to_link5)
                'urdf_home': 1.449, 'servo_home': 50,
                'urdf_min': -2.879793, 'urdf_max': 2.879793,
                'servo_min': 80, 'servo_max': 160,
                'direction': 1,
                'pca_channel': 3
            },
            {   # Wrist Roll (link5_to_link6) -> PCA channel 5
                'urdf_home': 0.0, 'servo_home': 120,
                'urdf_min': -2.879793, 'urdf_max': 2.879793,
                'servo_min': 100, 'servo_max': 140,
                'direction': 1,
                'pca_channel': 5
            },
            {   # Gripper (link6_to_link6_flange) -> PCA channel 4
                'urdf_home': 0.0, 'servo_home': 120,
                'urdf_min': -3.05, 'urdf_max': 3.05,
                'servo_min': 10, 'servo_max': 170,
                'direction': 1,
                'pca_channel': 4
            },
        ]
        # ══════════════════════════════════════════════════════

        self.JOINT_LABELS = ['Base', 'Shoulder', 'Elbow', 'Wrist Pitch', 'Wrist Roll', 'Gripper']

        # CUSTOMIZE: Change this if your Arduino is on a different port
        self.SERIAL_PORT = '/dev/ttyUSB0'
        self.BAUD_RATE = 115200
        self.is_ready = False

        self.last_sent = [0] * 6
        for cfg in self.SERVO_CONFIG:
            self.last_sent[cfg['pca_channel']] = cfg['servo_home']

        try:
            self.ser = serial.Serial(self.SERIAL_PORT, self.BAUD_RATE, timeout=1)
            time.sleep(2)
            startup = self.ser.readline().decode().strip()
            self.get_logger().info(f'Arduino says: {startup}')
            if self.ser.in_waiting > 0:
                home_msg = self.ser.readline().decode().strip()
                self.get_logger().info(f'Arduino: {home_msg}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
            self.get_logger().error('Run: killserial && sudo chmod 666 /dev/ttyUSB0')
            return

        self.get_logger().info('')
        self.get_logger().info('==================================================')
        self.get_logger().info('  SAFETY: Moving all servos to HOME one by one')
        self.get_logger().info('  Hold the arm gently during homing!')
        self.get_logger().info('==================================================')

        home_command = [0] * 6
        for cfg in self.SERVO_CONFIG:
            home_command[cfg['pca_channel']] = cfg['servo_home']

        current_command = home_command.copy()

        for i in range(len(self.SERVO_CONFIG)):
            pca_ch = self.SERVO_CONFIG[i]['pca_channel']
            current_command[pca_ch] = self.SERVO_CONFIG[i]['servo_home']
            cmd = ' '.join(str(a) for a in current_command) + '\n'
            self.ser.write(cmd.encode())
            self.get_logger().info(
                f'  Homing CH{pca_ch} ({self.JOINT_LABELS[i]}) -> '
                f'{self.SERVO_CONFIG[i]["servo_home"]} deg  '
                f'[range: {self.SERVO_CONFIG[i]["servo_min"]}-{self.SERVO_CONFIG[i]["servo_max"]}]'
            )
            time.sleep(1.0)
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                self.get_logger().info(f'  Arduino: {response}')

        self.last_sent = home_command.copy()

        self.get_logger().info('  All servos at HOME')
        self.get_logger().info('==================================================')
        self.get_logger().info('')
        self.get_logger().info('  MAPPING TABLE:')
        for i in range(len(self.SERVO_CONFIG)):
            cfg = self.SERVO_CONFIG[i]
            self.get_logger().info(
                f'    {self.JOINT_LABELS[i]}: '
                f'URDF home={math.degrees(cfg["urdf_home"]):.1f} deg -> '
                f'Servo home={cfg["servo_home"]} deg (PCA CH{cfg["pca_channel"]})  '
                f'[{cfg["servo_min"]}-{cfg["servo_max"]}]  '
                f'{"NORMAL" if cfg["direction"] == 1 else "REVERSED"}'
            )
        self.get_logger().info('')
        self.get_logger().info('  Waiting 3 seconds...')
        time.sleep(3)

        self.is_ready = True

        self.subscription = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10
        )
        self.adjust_sub = self.create_subscription(
            String, '/servo_adjust',
            self.adjust_callback, 10
        )
        self.timer = self.create_timer(0.1, self.read_arduino)

        self.get_logger().info('==================================================')
        self.get_logger().info('  6-DOF Servo Bridge READY')
        self.get_logger().info('  Manual: ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: \'0 90\'}"')
        self.get_logger().info('  Home:   ros2 topic pub --once /servo_adjust std_msgs/msg/String "{data: \'h\'}"')
        self.get_logger().info('==================================================')

    def radians_to_servo(self, radians, config):
        delta_rad = radians - config['urdf_home']
        delta_rad = delta_rad * config['direction']

        if delta_rad >= 0:
            urdf_range = config['urdf_max'] - config['urdf_home']
            servo_range = config['servo_max'] - config['servo_home']
            if urdf_range != 0:
                fraction = delta_rad / urdf_range
                fraction = min(fraction, 1.0)
            else:
                fraction = 0
            servo_angle = config['servo_home'] + (fraction * servo_range)
        else:
            urdf_range = config['urdf_home'] - config['urdf_min']
            servo_range = config['servo_home'] - config['servo_min']
            if urdf_range != 0:
                fraction = abs(delta_rad) / urdf_range
                fraction = min(fraction, 1.0)
            else:
                fraction = 0
            servo_angle = config['servo_home'] - (fraction * servo_range)

        servo_angle = int(max(config['servo_min'], min(config['servo_max'], servo_angle)))
        return servo_angle

    def joint_state_callback(self, msg):
        if not self.is_ready:
            return

        servo_angles = [0] * 6

        for i, joint_name in enumerate(self.JOINT_NAMES):
            try:
                index = msg.name.index(joint_name)
                radians = msg.position[index]
                servo_angle = self.radians_to_servo(radians, self.SERVO_CONFIG[i])
                pca_ch = self.SERVO_CONFIG[i]['pca_channel']
                servo_angles[pca_ch] = servo_angle
            except ValueError:
                pca_ch = self.SERVO_CONFIG[i]['pca_channel']
                servo_angles[pca_ch] = self.last_sent[pca_ch]

        if servo_angles != self.last_sent:
            cmd = ' '.join(str(a) for a in servo_angles) + '\n'
            self.ser.write(cmd.encode())
            self.last_sent = servo_angles.copy()

            log_parts = []
            for i in range(len(self.SERVO_CONFIG)):
                pca_ch = self.SERVO_CONFIG[i]['pca_channel']
                log_parts.append(f'CH{pca_ch}({self.JOINT_LABELS[i]}):{servo_angles[pca_ch]}')
            self.get_logger().info(' | '.join(log_parts))

    def adjust_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f'Manual adjust: {command}')
        self.ser.write((command + '\n').encode())

    def read_arduino(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()
            if line:
                self.get_logger().info(f'Arduino: {line}')


def main():
    rclpy.init()
    node = SixServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info('Shutting down - homing servos...')
        if hasattr(node, 'ser') and node.ser.is_open:
            current_command = node.last_sent.copy()
            for i in range(len(node.SERVO_CONFIG)):
                pca_ch = node.SERVO_CONFIG[i]['pca_channel']
                current_command[pca_ch] = node.SERVO_CONFIG[i]['servo_home']
                cmd = ' '.join(str(a) for a in current_command) + '\n'
                node.ser.write(cmd.encode())
                node.get_logger().info(f'  Homing CH{pca_ch} ({node.JOINT_LABELS[i]}) -> {node.SERVO_CONFIG[i]["servo_home"]}')
                time.sleep(1.0)
            node.ser.close()
        node.get_logger().info('Safe to power off.')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
