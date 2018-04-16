import numpy as np
import cv2
import cv2.aruco as aruco




cap = cv2.VideoCapture(0)
cap.set(3, 1920)
cap.set(4, 1080)
ret, gray = cap.read()
bbox = cv2.selectROI(gray, False)

cv2.destroyAllWindows()

count = 0
corner_anterior = None
gray_ant = None
while(True):
    # Capture frame-by-frame
    ret, gray = cap.read()
    gray = gray[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    #gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    if len(corners) == 0:
        count += 1
        cv2.imwrite('lost'+str(count)+'.jpg', gray)
        #cv2.imwrite('anterior'+str(count)+'.jpg', gray_ant)


    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    gray = aruco.drawDetectedMarkers(gray, corners)

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

    corner_anterior = corners
    gray_ant = gray

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()