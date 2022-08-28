## QRCode Detection 

this section of the repo includes codes for detecting and decoding the contents of a qrcode in real time. 
An additional Functionality is added to compare the contents of the qrcode with a list to check validity of our required data. For this example, it's a text file containing a list of numbers. The text file is compared with the decoded content to check whether the same number is displayed on the decoded list or not. If yes, it is displayed as authorized on the screen. Bounding box and displayed text for the authorized and unauthorized codes will be in blue and red colors respectively
Two main objectives of this code:
1. to create bounding boxes over the detected qrcodes using OpenCV. Polylines were used to draw bounding box with decoded frames as their coordinates
2. After detection, decode and read the contents for comparison with the text file. The contents of the decoded message will be displayed on the screen after successfully reading them.



## Authors

- [@Anshumaan](https://github.com/Anshumaan031)


## Documentations for libraries used

[Pyzbar](https://pypi.org/project/pyzbar/)

[OpenCV](https://pypi.org/project/opencv-python/)

[NumPy](https://numpy.org/)

## Installation

Install pyzbar for python

The zbar DLLs are included with the Windows Python wheels. On other operating systems, you will need to install the zbar shared library.

Mac OS X:

```bash
brew install zbar
```
Linux:

```bash
sudo apt-get install libzbar0
```

Install this Python wrapper; use the second form to install dependencies of the command-line scripts:

```bash
pip install pyzbar
pip install pyzbar[scripts]  
```  
Example usage:

The decode function accepts instances of PIL.Image.


```bash
>>> from pyzbar.pyzbar import decode
>>> from PIL import Image
>>> decode(Image.open('pyzbar/tests/code128.png'))

[
    Decoded(
        data=b'Foramenifera', type='CODE128',
        rect=Rect(left=37, top=550, width=324, height=76),
        polygon=[
            Point(x=37, y=551), Point(x=37, y=625), Point(x=361, y=626),
            Point(x=361, y=550)
        ]
    )
    Decoded(
        data=b'Rana temporaria', type='CODE128',
        rect=Rect(left=4, top=0, width=390, height=76),
        polygon=[
            Point(x=4, y=1), Point(x=4, y=75), Point(x=394, y=76),
            Point(x=394, y=0)
        ]
    )
]
```  
For reading the text file as a list for comparison:

```bash
with open('DataFile.txt') as f:
   myDataList = f.read().splitlines()
print(myDataList)
``` 

sample image:</br>

![sample2](https://user-images.githubusercontent.com/67821036/150372857-e3bb75c9-c256-49c3-ba9d-e3e308ac976e.jpg)
