
import cv2
import time


cap = cv2.VideoCapture(0)
prevt = time.time()

wp = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
hp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

try:
    while(True):
        ret, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        cv2.imshow('asdf', frame)
        if cv2.waitKey(1) == ord('q'):
            break

        print(time.time() - prevt)
        prevt = time.time()

except KeyboardInterrupt:
    pass
