import numpy as np
from rclpy import init, shutdown
import re
import cv2
import os
from pyzbar import pyzbar
# os.environ["QT_QPA_PLATFORM"] = "xcb"
img = cv2.imread("image/QR2.png",cv2.IMREAD_GRAYSCALE)
img = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)

ret, th = cv2.threshold(img, 0, 255, cv2.THRESH_OTSU)
res = str(pyzbar.decode(th))
match = re.search(r"data=b'([^']+)'", res)
if match:
    data_value = match.group(1)
    print(data_value)
cv2.imshow("11", th)
cv2.waitKey(0)
cv2.destroyAllWindows()
