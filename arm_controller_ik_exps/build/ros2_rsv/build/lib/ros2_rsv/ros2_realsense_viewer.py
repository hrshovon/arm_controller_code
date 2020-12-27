# coding=utf-8

import rclpy
import rclpy.node as node
import cv2
import numpy as np
import sensor_msgs.msg as msg
import std_msgs.msg as std_msg
import rclpy.qos as qos

#import third_party.ros.ros as ros

class TestDisplayNode(node.Node):
    def __init__(self):
        super().__init__('IProc_TestDisplayNode')
        self.__window_name = "img"

        #profile = qos.QoSProfile(history=qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        profile = qos.QoSProfile()

        self.sub = self.create_subscription(msg.Image, '/color/image_raw', self.msg_callback, qos_profile=profile)

   
    def msg_callback(self, m : msg.Image):
        #self.print_header_info(m.header)
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        self.display(np_img)

    def display(self, img : np.ndarray):
        cv2.imshow(self.__window_name, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        cv2.waitKey(10)

def main():
    #ros_core = ros.IProc_Ros2Core()
    rclpy.init()
    node = TestDisplayNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
