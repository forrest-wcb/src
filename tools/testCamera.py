import cv2

cap = cv2.VideoCapture("/dev/video8")

while cap.isOpened():
    ret, frame = cap.read()
    if ret is True:
        cv2.imshow("video", frame)
        
        # cv2.waitKey(int(1000 / cap.get(cv2.CAP_PROP_FPS)))

        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    
    else:
        print("load failed")
        break