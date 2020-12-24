import cv2
import numpy as np 
from math import *
import rclpy
from rclpy.node import Node
import time 
from std_msgs.msg import String, Float32MultiArray
import copy
angle_data=[0,0,0,0]
prev_angle_data=[0,0,0,0]

def draw_position(img,base_control_circle_center, arm_base_pos, angle_data):
    #draw base position indicator hand
    base_line_x2 = int(base_control_circle_center[0]+(120*cos(radians(angle_data[0]-30))))
    base_line_y2 = int(base_control_circle_center[1]-(120*sin(radians(angle_data[0]-30))))
    
    cv2.line(img,base_control_circle_center,(base_line_x2,base_line_y2),(140,140,140),2)

    #draw the arm 
    arm_tip_x = int(arm_base_pos[0]-(200*cos(radians(angle_data[1]))))
    arm_tip_y = int(arm_base_pos[1]-(200*sin(radians(angle_data[1]))))
    cv2.line(img,arm_base_pos,(arm_tip_x,arm_tip_y),(255,0,0),2)
    #draw the elbow
    elbow_tip_x = int(arm_tip_x-(200*cos(radians(angle_data[1]+angle_data[2]-90))))
    elbow_tip_y = int(arm_tip_y-(200*sin(radians(angle_data[1]+angle_data[2]-90))))
    cv2.line(img,(arm_tip_x,arm_tip_y),(elbow_tip_x,elbow_tip_y),(0,255,0),2)
    
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('robot_ui')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_sensor_data',
            self.listener_callback)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        global angle_data,h,w,prev_angle_data
        tm=time.time()
        angle_data=msg.data
        arm_base_pos = (w-50,h-450)

        base_control_bbox_coords = ((w-310,10),(w-10,310)) 
        base_control_circle_center = ((base_control_bbox_coords[0][0]+base_control_bbox_coords[1][0])//2,(base_control_bbox_coords[0][1]+base_control_bbox_coords[1][1])//2)

        img_blank = np.zeros((h,w,3),dtype=np.uint8)
        cv2.rectangle(img_blank,base_control_bbox_coords[0],base_control_bbox_coords[1],(0,255,0),2)
        cv2.circle(img_blank,base_control_circle_center,140,(0,0,255),2)
        try:
            draw_position(img_blank,base_control_circle_center,arm_base_pos,angle_data)
            prev_angle_data=copy.copy(angle_data)
        except:
            draw_position(img_blank,base_control_circle_center,arm_base_pos,prev_angle_data)
        cv2.imshow("controller_window",img_blank)
        cv2.waitKey(1) # & 0xFF == ord('q'):
        print(time.time()-tm)
            #break 
        

h,w=0,0    
def main(args=None):
    global angle_data,h,w
    h, w = 1200,800
    
    cv2.namedWindow("controller_window")
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

   

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
