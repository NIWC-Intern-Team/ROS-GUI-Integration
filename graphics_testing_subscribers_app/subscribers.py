import sys
import time
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import uic

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class floatSub():
    def connect_ros_float(self, state):
        if (Qt.Checked == state):
            try:
                # ROS2 init
                rclpy.init(args=None)
                self.node = Node('Qt_view_node')

                self.pub = self.node.create_subscription(Float64, self.float_topic_name, self.sub_float_callback, 10)


                # spin once, timeout_sec 5[s]
                timeout_sec_rclpy = 5
                timeout_init = time.time()
                rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
                timeout_end = time.time()
                ros_connect_time = timeout_end - timeout_init

                # Error Handle for rclpy timeout
                if ros_connect_time >= timeout_sec_rclpy:
                    self.label_ros2_state_float.setText("Couldn't Connect")
                    self.label_ros2_state_float.setStyleSheet(
                        "color: rgb(255,255,255);"
                        "background-color: rgb(255,0,51);"
                        "border-radius:5px;"
                    )
                else:
                    self.label_ros2_state_float.setText("Connected")
                    self.label_ros2_state_float.setStyleSheet(
                        "color: rgb(255,255,255);"
                        "background-color: rgb(18,230,95);"
                        "border-radius:5px;"
                    )
            except:
                pass
        else:
            self.node.destroy_node()
            rclpy.shutdown()
    
    def timer_float_update(self):
        rclpy.spin_once(self.node)
        self.update_float_data_label()
        self.show()
        self.timer.start(10)