import cv2
import numpy as np 
from math import *
import rclpy
from rclpy.node import Node
import time 
from std_msgs.msg import String, Float32MultiArray
import copy


class point:
    def __init__(self,x,y):
        self.x=x
        self.y=y
    def check_if_within_rectangle(self,rec,inclusive=False):
        assert isinstance(rec,rectangle)
        p_left = rec.p1 
        p_right = rec.p2
        if inclusive==True:
            return (p_left.x<=self.x<=p_right.x) and (p_left.y<=self.y<=p_right.y)
        else:
            return (p_left.x<self.x<p_right.x) and (p_left.y<self.y<p_right.y)
    
class rectangle:
    def __init__(self,p1,p2):
        assert isinstance(p1,point)
        assert isinstance(p2,point)
        self.p1=p1
        self.p2=p2    
    def get_center(self):
        center_point = point((self.p1.x+self.p2.x)//2,
                              (self.p1.y+self.p2.y)//2)
        return center_point


angle_data=[0,0,0,0]
prev_angle_data=[0,0,0,0]
mouse_coords=point(0,0)
set_angle=False
h, w = 1200,800
    
arm_base_pos = point(w-50,h-450)
base_length = 120
arm_length = 200
elbow_length = 200
point_ok = False
base_control_bbox_coords =  rectangle(point(w-310,10),
                                      point(w-10,310)) #((w-310,10),(w-10,310)) 
base_control_circle_center = base_control_bbox_coords.get_center() 
#((base_control_bbox_coords[0][0]+base_control_bbox_coords[1][0])//2,
# (base_control_bbox_coords[0][1]+base_control_bbox_coords[1][1])//2)

def calcdist(p1,p2):
    return sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2)

#def get_target_angle(mouse_p,is_base=False):
    '''
    we have mouse_p which is our target point
    we have length of each arm
    if its base then we do calculation for base
    if its arm we dp calculation for arm
    '''
    #base_length = 
    #if is_base==False:
    #    arm_angle = 

def handle_mouse_events(event, x, y, flags, param):
    global mouse_coords,point_ok
    if event==cv2.EVENT_LBUTTONDOWN:
        point_ok=False
        mouse_coords=point(x,y)
        #check if within base control
        if mouse_coords.check_if_within_rectangle(base_control_bbox_coords)==True:
            print("base control")
        else:
            print("arm control")
            if calcdist(mouse_coords,arm_base_pos) <= arm_length+elbow_length:
                
                point_ok=True
            
def draw_position(img,base_control_circle_center, arm_base_pos, angle_data):
    #draw base position indicator hand
    global point_ok
    base_line_x2 = int(base_control_circle_center.x+(base_length*cos(radians(angle_data[0]-30))))
    base_line_y2 = int(base_control_circle_center.y-(base_length*sin(radians(angle_data[0]-30))))
    
    base_2 = point(base_line_x2,base_line_y2)

    cv2.line(img,
             (base_control_circle_center.x,base_control_circle_center.y),
             (base_2.x,base_2.y),(140,140,140),
             2)

    #draw the arm 
    arm_tip_x = int(arm_base_pos.x-(arm_length*cos(radians(angle_data[1]))))
    arm_tip_y = int(arm_base_pos.y-(arm_length*sin(radians(angle_data[1]))))
    
    arm_tip = point(arm_tip_x,arm_tip_y)
    cv2.line(img,
             (arm_base_pos.x,arm_base_pos.y),
             (arm_tip.x,arm_tip.y),(255,0,0),
             2)
    #draw the elbow
    elbow_tip_x = int(arm_tip.x-(elbow_length*cos(radians(angle_data[1]+angle_data[2]-90))))
    elbow_tip_y = int(arm_tip.y-(elbow_length*sin(radians(angle_data[1]+angle_data[2]-90))))
    
    elbow_tip=point(elbow_tip_x,elbow_tip_y)
    cv2.line(img,
             (arm_tip.x,arm_tip.y),
             (elbow_tip.x,elbow_tip.y),
             (0,255,0),
             2)
    
    #draw target_triangle
    if point_ok==True:
        cv2.line(img,
                  (mouse_coords.x,mouse_coords.y),
                  (arm_base_pos.x,arm_base_pos.y),
                  (255,255,255),
                  2)

class botSubscriber(Node):

    def __init__(self):
        super().__init__('robot_ui')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_sensor_data',
            self.listener_callback)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        global angle_data,h,w,prev_angle_data, arm_base_pos, base_control_bbox_coords, base_control_circle_center
        angle_data=msg.data
        
        img_blank = np.zeros((h,w,3),dtype=np.uint8)
        cv2.rectangle(img_blank,
                      (base_control_bbox_coords.p1.x,base_control_bbox_coords.p1.y),
                      (base_control_bbox_coords.p2.x,base_control_bbox_coords.p2.y),
                      (0,255,0),
                      2)
        cv2.circle(img_blank,
                   (base_control_circle_center.x,base_control_circle_center.y),
                   140,
                   (0,0,255),
                   2)
        try:
            if len(angle_data)==4:
                draw_position(img_blank,base_control_circle_center,arm_base_pos,angle_data)
                prev_angle_data=copy.copy(angle_data)
            else:
                draw_position(img_blank,base_control_circle_center,arm_base_pos,prev_angle_data)    
        except:
            draw_position(img_blank,base_control_circle_center,arm_base_pos,prev_angle_data)
        cv2.imshow("controller_window",img_blank)
        cv2.waitKey(10) # & 0xFF == ord('q'):
            #break 
        


def main(args=None):
    
    cv2.namedWindow("controller_window")
    cv2.setMouseCallback("controller_window",handle_mouse_events,param=12)
    rclpy.init(args=args)

    bot_subscriber = botSubscriber()

   

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    while rclpy.ok():
        rclpy.spin_once(bot_subscriber)
    bot_subscriber.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
