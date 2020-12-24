import cv2
import numpy as np 
from math import *

def draw_position(img,base_control_circle_center, arm_base_pos, angle_data):
    #draw base position indicator hand
    base_line_x2 = int(base_control_circle_center[0]+(120*cos(radians(angle_data[0]-30))))
    base_line_y2 = int(base_control_circle_center[1]-(120*sin(radians(angle_data[0]-30))))
    
    cv2.line(img,base_control_circle_center,(base_line_x2,base_line_y2),(140,140,140),2)

h, w = 800,800
img_blank = np.zeros((h,w,3),dtype=np.uint8)

base_control_bbox_coords = ((w-310,10),(w-10,310)) 
base_control_circle_center = ((base_control_bbox_coords[0][0]+base_control_bbox_coords[1][0])//2,(base_control_bbox_coords[0][1]+base_control_bbox_coords[1][1])//2)

cv2.namedWindow("controller_window")

while True:
    cv2.rectangle(img_blank,base_control_bbox_coords[0],base_control_bbox_coords[1],(0,255,0),2)
    cv2.circle(img_blank,base_control_circle_center,140,(0,0,255),2)
    draw_position(img_blank,base_control_circle_center,0,[0])
    cv2.imshow("controller_window",img_blank)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break 

cv2.destroyAllWindows()