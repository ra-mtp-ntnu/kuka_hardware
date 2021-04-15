# Copyright 2019 Norwegian University of Science and Technology
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import socket
import numpy as np
import time
import errno
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

node_name = 'kuka_rsi_simulator'


class KukaRsiSimulator(Node):

    def __init__(self):
        super().__init__(node_name)
        self._publisher = self.create_publisher(String, 'rsi_sim_topic', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.settimeout(1)
        self._host = '127.0.0.1'
        self._port = 49152

        self.act_joint_pos = np.array([0.0, -90.0, 90.0, 0.0, 90.0, 0.0])
        self.initial_joint_pos = self.act_joint_pos.copy()
        self.des_joint_correction = np.zeros(6)
        self.timeout_count = 0
        self.ipoc = 0

    def timer_callback(self):
        try:
            msg = self.create_rsi_xml_rob(
                self.act_joint_pos, self.timeout_count, self.ipoc)
            print("msg",msg)
            self._socket.sendto(msg, (self._host, self._port))
            recv_msg, addr = self._socket.recvfrom(1024)
            self.des_joint_correction, ipoc_recv = self.parse_rsi_xml_sen(
                recv_msg)
            print("self.des_joint_correction",self.des_joint_correction)
            self.act_joint_pos = self.des_joint_correction + self.initial_joint_pos
            print("self.act_joint_pos", self.act_joint_pos)
            self.ipoc += 1
        except socket.timeout:
            self.get_logger().warn('{}: Socket timed out'.format(node_name))
            self.timeout_count += 1
        # except socket.error, e:
        #     if e.errno != errno.EINTR:
        #         raise

        pubmsg = String()
        pubmsg.data = str(msg)[1:]
        self._publisher.publish(pubmsg)
        pubmsg = String()
        pubmsg.data = str(recv_msg)[1:]
        self._publisher.publish(pubmsg)
        self.i += 1

    def create_rsi_xml_rob(self, act_joint_pos, timeout_count, ipoc):
        q = act_joint_pos
        root = ET.Element('Rob', {'TYPE': 'KUKA'})
        ET.SubElement(root, 'RIst', {'X': '0.0', 'Y': '0.0', 'Z': '0.0',
                                     'A': '0.0', 'B': '0.0', 'C': '0.0'})
        ET.SubElement(root, 'RSol', {'X': '0.0', 'Y': '0.0', 'Z': '0.0',
                                     'A': '0.0', 'B': '0.0', 'C': '0.0'})
        ET.SubElement(root, 'AIPos', {'A1': str(q[0]), 'A2': str(q[1]), 'A3': str(q[2]),
                                      'A4': str(q[3]), 'A5': str(q[4]), 'A6': str(q[5])})
        ET.SubElement(root, 'ASPos', {'A1': '0.0', 'A2': '-90.0', 'A3': '90.0',
                                      'A4': '0.0', 'A5': '90.0', 'A6': '0.0'})
        ET.SubElement(root, 'Delay', {'D': str(timeout_count)})
        ET.SubElement(root, 'IPOC').text = str(ipoc)
        return ET.tostring(root)

    def parse_rsi_xml_sen(self, data):
        root = ET.fromstring(data)
        AK = root.find('AK').attrib
        desired_joint_correction = np.array([AK['A1'], AK['A2'], AK['A3'],
                                             AK['A4'], AK['A5'], AK['A6']]).astype(np.float64)
        IPOC = root.find('IPOC').text
        return desired_joint_correction, int(IPOC)


def main(args=None):
    rclpy.init(args=args)

    kuka_rsi_simulator = KukaRsiSimulator()

    rclpy.spin(kuka_rsi_simulator)

    kuka_rsi_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
