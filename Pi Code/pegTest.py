import numpy as np
import cv2

idealRatio = 0.39  # 20/51.25
ratioTolerance = 0.075

lower_bound = np.array([89, 0, 60]) #ADJUST THISSSSS
upper_bound = np.array([92, 255, 255])

cap = cv2.VideoCapture(0)

_, frame = cap.read()

frameWidth = int(cap.get(3))
print frameWidth
frameHeight = int(cap.get(4))
print frameHeight



while (1):
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    _,contours,_ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #|CHANGED

    result = np.array([])
    if len(contours) > 0:
        for c in contours:
            hull = cv2.convexHull(c)
            hullArea = cv2.contourArea(hull)
            insideArea = cv2.contourArea(c)
            ratio = insideArea/(hullArea+.1)

            if ratio > 0.9 and ratio < 1.1:
                if len(result) == 0:
                    result = c
                else:
                    result = np.concatenate((result, c), axis = 0)
        if len(result) > 0:
            resultContour = cv2.convexHull(result)
            for x in range(0, len(resultContour)-1):
                print tuple(resultContour[x][0])
                cv2.line(frame, tuple(resultContour[x][0]), tuple(resultContour[x+1][0]), (0, 255, 0), 2)
            cv2.imshow("this", frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()