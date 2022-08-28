import cv2
import numpy as np
from pyzbar.pyzbar import decode


img = cv2.imread('sample.jpg')   #for reading a sample image
height, width = img.shape[:2]


with open('DataFile.txt') as f:
   myDataList = f.read().splitlines()
print(myDataList)


#qrcode = decode((img[:, :, 0].astype('uint8').tobytes(), width, height))  #pyzbar decodes images through a tuple (pixels, width, height), where the image data is eight bits-per-pixel.
#print(qrcode)

for qrcode in decode(img):
    print(qrcode.data)
    myData = qrcode.data.decode('utf-8')  #Decoding the data from the qrcode
    print(myData)

    if myData in myDataList:
        Output = 'Authorized'
        color = (255, 0, 0) #blue
    else:
        Output = 'un-authorized'
        color =  (0, 0, 255) #red  

    pts = np.array([qrcode.polygon], np.int32)  #for creaitng polygon taking the coordinates of qrcode on the frame
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(img, [pts], True, color, 5)  #cv2.polylines() method is used to draw a polygon on any image.
    pts2 = qrcode.rect

    cv2.putText(img, Output, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

cv2.imshow('Result', img)
cv2.waitKey(1)
