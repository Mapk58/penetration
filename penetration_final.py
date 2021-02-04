import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
from PIL import Image as IMG
import os
from clover.srv import SetLEDEffect
import sys


bridge = CvBridge()
rospy.init_node('final')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  

#funkzia ozhidania prileta do tochki
def wait_arrival(tolerance=0.1):
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


#funkzia posadki
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)



def number_detection(img):
    patterns_scaled = patterns
    x= img.shape[1]
    y = img.shape[0]
    for i in range(4):
        patterns_scaled[i] = cv2.resize(patterns_scaled[i],(x,y))
        
    checking =np.sum(img == 255)
    percents = []
    for i in range(4):
        res = cv2.bitwise_and(img,patterns_scaled[i])
        #cv2.imshow(str(i)+"result_.png", res)
        summa =np.sum(res == 255)
        percents.append(float(summa)/float(checking) *100)
       # print(str(i)+" mask, percentage ="+str(percentage))
    #print(percents)
    print(percents.index(max(percents)))
    return(percents.index(max(percents)))




def pattern_creation(img):
    global patterns
    x_max =0
    
    x_min = img.shape[1]
    y_max = 0
    
    y_min = img.shape[0]
    white = np.argwhere(img == 255)
    for i in range(len(white)):
        if white[i][0] > x_max:
            x_max = white[i][0]
        if white[i][0] < x_min:
            x_min = white[i][0]
        if white[i][1] > y_max:
            y_max = white[i][1]
        if white[i][1] < y_min:
            y_min = white[i][1]
    cutted = img[x_min:x_max, y_min:y_max]

    #print(x_min, x_max, y_min, y_max)
    return(cutted)


def number_detection(img):
    patterns_scaled = patterns
    x= img.shape[1]
    y = img.shape[0]
    for i in range(4):
        patterns_scaled[i] = cv2.resize(patterns_scaled[i],(x,y))
        
    checking =np.sum(img == 255)
    percents = []
    for i in range(4):
        res = cv2.bitwise_and(img,patterns_scaled[i])
        #cv2.imshow(str(i)+"result_.png", res)
        summa =np.sum(res == 255)
        percents.append(float(summa)/float(checking) *100)
       # print(str(i)+" mask, percentage ="+str(percentage))
    #print(percents)
    print(percents.index(max(percents)))
    return(percents.index(max(percents)))

#T0 = 0 #yellow - products
#T1 = 0 #green - clothes
#T2 = 0 #blue - fragile packaging
#T3 = 0 #red - correspondence
#sozdanie masok
def colour_detect(img):
    global mask_red, mask_green, mask_yellow, mask_blue, mask,mask_green_number
    #img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imwrite("img.png", img)

    #cv2.imwrite("img.png", img)

    mask_yellow = cv2.inRange(hsv, (0, 37, 189), (37, 192, 255))
    mask_green = cv2.inRange(hsv, (60, 50, 110), (90, 130, 210))
    mask_blue = cv2.inRange(hsv, (80, 122, 122), (157, 255, 255))
    mask_red = cv2.inRange(hsv, (158, 26, 174), (202, 187, 255))
    #cv2.imwrite("result_yellow.png", mask_yellow)
    mask_green_number = cv2.inRange(hsv, (0, 0, 0), (255, 150, 150))

    #cv2.imwrite("result_blue.png", mask_blue)
    mask =[]
    mask.append(mask_yellow)
    mask.append(mask_green)
    mask.append(mask_blue)
    mask.append(mask_red)


def dumping_goods(i):
    navigate(x=0.0, y=0.0, z=-0.3, speed=0.5, frame_id='body', auto_arm=True) #lift_up
    wait_arrival()
    land_wait()
    if(i == 0):
        set_effect(r=255, g=255, b=0)  # fill strip with yellow color
    
    elif(i==1):
        set_effect(r=0, g=100, b=0)  # fill strip with green color

    elif(i==2):
        set_effect( r=0, g=0, b=255)  # fade to blue color

    else:
        set_effect(r=255, g=0, b=0)  # fill strip with red color
    
    rospy.sleep(5)
    set_effect(r=255, g=255, b=255)
    print("D"+str(i)+" delvered"+goods[i])
    navigate(x=0.0, y=0.0, z=0.5, speed=0.5, frame_id='body', auto_arm=True)
    wait_arrival()
    


def count_contours(img, Ti):
    
    image, contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(cam_img, contours, -1, (0,0,0), 2)
    #print("number of contours is "+str(len(contours)))
    
    #print("areas are:"+str(cv2.contourArea(contours[0]))+" "+str(cv2.contourArea(contours[1]))) ### min area = 400
    for i in range(len(contours)):
        if(cv2.contourArea(contours[i]) > 250):
            Ti +=1
            i = i+1
    return(Ti)
            
   

def count_goods():
    global T,cam_img
    cam_img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cam_img=cam_img[img_height-height_cutted:img_height+height_cutted, img_width-width_cutted:img_width+width_cutted]
    cam_img = color_correction(cam_img)
    colour_detect(cam_img)
    for i in range(len(T)):
        T[i]=count_contours(mask[i],T[i])


def find_out_max_contour(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # red color boundaries [B, G, R] (72, 68, 74), (180,255,166)(58, 58, 93), (84, 255, 255)
    lower = [58, 58, 93]
    upper = [84, 255, 255]

    # create NumPy arrays from the boundaries mask_green = cv2.inRange(hsv, (40,55,70), (110,255,150))
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask_ = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask_)
    #cv2.imshow("Mesk green", mask_)

    ret, thresh = cv2.threshold(mask_, 40, 255, 0)
    if (int(cv2.__version__[0]) > 3):
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    else:
        contours, hierarchy, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    mask_result = np.zeros(image.shape[:2], dtype=np.uint8)
    if len(contours) != 0:
        try:
            # draw in blue the contours that were founded
            cv2.drawContours(output, contours, -1, 255, 3)

            # find the biggest countour (c) by the area
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            # draw the biggest contour (c) in green
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv.drawContours(mask_result, [c], 0, (255), -1)
            #cv2.imshow("Result3", mask_result)
            # cv2.waitKey(0)
        except Exception as e:
            pass
    #cv2.imshow("Result", image)
    #cv2.imshow("mask_result", mask_result)
    return mask_result


def color_correction(image):
    filterFactor = 1
    h, w, _ = image.shape
    w -= 1
    h -= 1
    N = w * h

    b, g, r = [0 for i in range(256)], [0 for i in range(256)], [0 for i in range(256)]

    for i in range(h):
        for j in range(w):
            intensity = image[i][j]

            b[intensity[0]] = b[intensity[0]] + 1
            g[intensity[1]] = g[intensity[1]] + 1
            r[intensity[2]] = r[intensity[2]] + 1

    for i in range(1, 256):
        b[i] = b[i] + filterFactor * b[i - 1]
        g[i] = g[i] + filterFactor * g[i - 1]
        r[i] = r[i] + filterFactor * r[i - 1]

    vmin_b = 0
    vmin_g = 0
    vmin_r = 0

    vmax_b = 255 - 1
    vmax_g = 255 - 1
    vmax_r = 255 - 1

    s1 = 3
    s2 = 3

    def find_min(b, vmin_b, N=N,s1=s1):
        # global N, s1
        while (b[vmin_b + 1] <= N * s1 / 100):
            vmin_b = vmin_b + 1
        return vmin_b

    def find_max(b, vmax_b,N=N, s2=s2):
        # global N, s2
        while b[vmax_b - 1] > (N - ((N / 100) * s2)):
            vmax_b = vmax_b - 1

        if vmax_b < 255 - 1:
            vmax_b = vmax_b + 1
        return vmax_b

    vmin_b = find_min(b, vmin_b)
    vmin_g = find_min(g, vmin_g)
    vmin_r = find_min(r, vmin_r)

    vmax_b = find_max(b, vmax_b)
    vmax_g = find_max(g, vmax_g)
    vmax_r = find_max(r, vmax_r)

    for i in range(h):
        for j in range(w):
            intensity = image[i][j]

            if intensity[0] < vmin_b:
                intensity[0] = vmin_b
            if intensity[1] < vmin_g:
                intensity[1] = vmin_g
            if intensity[2] < vmin_r:
                intensity[2] = vmin_r

            if intensity[0] > vmax_b:
                intensity[0] = vmax_b
            if intensity[1] > vmax_g:
                intensity[1] = vmax_g
            if intensity[2] > vmax_r:
                intensity[2] = vmax_r
            image[i][j] = intensity

    for i in range(h):
        for j in range(w):
            intensity = image[i][j]

            intensity[0] = (intensity[0] - vmin_b) * 255 / (vmax_b - vmin_b)
            intensity[1] = (intensity[1] - vmin_g) * 255 / (vmax_g - vmin_g)
            intensity[2] = (intensity[2] - vmin_r) * 255 / (vmax_r - vmin_r)

            image[i][j] = intensity
    return image


goods =["products", "clothes","fragile packaging","correspondence"]

T = [0,0,0,0]

#T0 = 0 #yellow - products
#T1 = 0 #green - clothes
#T2 = 0 #blue - fragile packaging
#T3 = 0 #red - correspondence

#standart 320*240
height_cutted = 150/2
width_cutted = 320/2
cam_img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

img_width = int(cam_img.shape[1])
img_height = int(cam_img.shape[0])
cam_img=cam_img[img_height-height_cutted:img_height+height_cutted, img_width-width_cutted:img_width+width_cutted]
img_width = img_width/2
img_height = img_height/2





#count_goods()



#print(T)

patterns = []              #creating patterns
for i in range(4):
    img = cv2.imread("number_" +str(i)+ ".jpg")
    #width = int(img.shape[1] * 100 / 100)
    #height = int(img.shape[0] * 100/ 100)
    #dsize = (width, height)
    #output = cv2.resize(img, dsize)
    #colour_detect(img)
    #pattern = pattern_creation(mask_green_number)
    pattern = img
    patterns.append(pattern)
    
    #cv2.imshow("pattern"+str(i)+ "_.png", patterns[i])



navigate(x=0.1, y=0.1, z=1.0, speed=0.5, frame_id='body', auto_arm=True) #lift_up
wait_arrival()
navigate(x=0.1, y=0.1, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
wait_arrival() 

navigate(x=0.5, y=3.6, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True) #lift_up
wait_arrival() 
navigate(x=0.5, y=3.6, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True) #lift_up
wait_arrival()

navigate(x=0.5, y=5.0, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
wait_arrival()

for i in range(0,4):
    navigate(x=0.45+3.6/4*i, y=5.0, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
    wait_arrival()
    count_goods()
    #print(T)
    #cv2.imwrite("result"+str(i)+".png", cam_img)
    
print("Balance: "+str(sum(T))+" cargo")
for i in range(len(T)):
   
    print("Type "+str(i)+": "+str(T[i])+" cargo")


navigate(x=1.8, y=3., z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
wait_arrival()

droppoints = []
founded_dp =0
for j in range(4):
    for i in range(5):
        navigate(x= 0.9*i, y=0.9*j, z=2.4, speed=0.5, frame_id='aruco_map', auto_arm=True)
        wait_arrival()
        img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
        #img = cv2.resize(img, dsize)
        img = img[40:200, 100:220]
        img = color_correction(img)
        img_corrected= find_out_max_contour(img)
        #img = cv2.blur(img,(5,5))

        #colour_detect(img_corrected)
        if(np.sum(mask_green == 255)>100) and founded_dp<2:
            img_cropped = pattern_creation(img_corrected)
            k = number_detection(img_cropped)
            print(k)
            founded_dp = founded_dp +1
            droppoints.append(0.9*i)
            droppoints.append(0.9*j)
            droppoints.append(k)
            print(droppoints)

drop_len = (len(droppoints))/2
for i in range(drop_len):
    x_n = droppoints[0+i*3]
    y_n = droppoints[1+i*3]
    navigate(x = x_n, y=y_n, z=2.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
    dumping_goods(int(droppoints[2+i*3]))



navigate(x=0.0, y=0.0, z=1.8, speed=0.5, frame_id='aruco_map', auto_arm=True)
wait_arrival()
navigate(x=0.0, y=0.0, z=1.0, speed=0.5, frame_id='aruco_map', auto_arm=True)
wait_arrival()
navigate(x=0.0, y=0.0, z=-0.5, speed=0.5, frame_id='body', auto_arm=True)
wait_arrival()

land_wait()
for i in range(len(droppoints)/2):
    print("D"+str(droppoints[2+i*3])+"_delivered to "+T[droppoints[2+i*3]]+" cargo")
    T[droppoints[2+i*3]]=0



print("Balance: "+str(sum(T))+" cargo")
