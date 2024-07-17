import cv2
import numpy
from hobot_dnn import pyeasy_dnn as dnn
import time

def main():
            
    # image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') #compressed_imgmsg_to_cv2
    image = cv2.imread('/root/dev_ws/src/line_follow_resnet/example.jpg')
    time_0 = time.time()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   
    lower = numpy.array([ 0,  0, 0])
    upper = numpy.array([180, 255, 46])
    mask = cv2.inRange(hsv, lower, upper)

    h, w, d = image.shape

    search_top = int(h/2)
    search_bot = int(h/2 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)

    if M['m00'] > 0:
        cx = int(M['m10']/M['m00']) 
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    time_1 = time.time()    
    print(time_1 - time_0)
    
if __name__ == "__main__":
    main()