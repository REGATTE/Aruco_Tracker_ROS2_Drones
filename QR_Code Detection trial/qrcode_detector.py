import cv2
import numpy as np
from pyzbar.pyzbar import decode


#img = cv2.imread('sample.png')   #for reading a sample image
cap = cv2.VideoCapture(0)   #for accessing the camera
cap.set(3, 640)
cap.set(4, 480)

with open('DataFile.txt') as f:
   myDataList = f.read().splitlines()
print(myDataList)


#for one qrcode to detect
#code = decode(img)
#print(code)

while True:

    ret, frame = cap.read()
    for qrcode in decode(frame):
        #print(qrcode.data)  #for printing only the data of the code. we can also extract .rect
        myData = qrcode.data.decode('utf-8')  #Decoding the data from the qrcode
        print(myData)

        #to check whether our decoded data is the one we require. In this case we are comparing the data from a test file
        #if myData in myDataList:
            #print('Authorized')
        #else:
            #print('un-authorized')

        if myData in myDataList:
            Output = 'Authorized'
            color = (255, 0, 0) #blue
        else:
            Output = 'un-authorized'
            color =  (0, 0, 255) #red  

        pts = np.array([qrcode.polygon], np.int32)  #for creaitng polygon taking the coordinates of qrcode on the frame
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], True, color, 5)  #cv2.polylines() method is used to draw a polygon on any image.
        pts2 = qrcode.rect

        #to print the data on screen
        #cv2.putText(frame, myData, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)  #putting the decoded data on screen along with the bounding boxes

        #to print whether it is authorized or not along with color
        cv2.putText(frame, Output, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)



    cv2.imshow('Result', frame)
    cv2.waitKey(1)
