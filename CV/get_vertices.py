# Python code to find the co-ordinates of
# the contours detected in an image.
import numpy as np
import cv2
import matplotlib.pyplot as plt
import argparse
import glob

def auto_canny(image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)
        # return the edged image
        return edged
  
# define font
font = cv2.FONT_HERSHEY_SIMPLEX

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser(description='Process images or video')
ap.add_argument("-i", "--images", action="store",
        help="path to input dataset of images in [IMAGES].jpg")
ap.add_argument("-v", "--video", action="store_true",
        help="to start videp capture")
args = ap.parse_args()

# loop over the images
if args.images:
    #print(args.images)
    #for imagePath in glob.glob(args["images"] + "*.jpg"):
    imagePath = args.images + ".jpg"
    #print(imagePath)
    # load the image
    img = cv2.imread(imagePath)
elif args.video:
    capture = cv2.VideoCapture(0)

    print("ESC key to capture image")

    while(True):
   	ret, frame = capture.read()
    	grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    	cv2.imshow('video gray', grayFrame)
    	#cv2.imshow('video original', frame)


    	if cv2.waitKey(1) == 27:
            print("Capturing Image")
  	    imagePath = 'test_gray.jpg'
            cv2.imwrite(imagePath, grayFrame)
            break
  
    capture.release()
    cv2.destroyAllWindows()

    # load the captured image
    img = cv2.imread(imagePath)
else: 
    exit()

# Reading image and converting to gray scale.
#img = cv2.imread('house.jpg')
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(img_gray, (5, 5), 0)

# apply Canny edge detection using a wide threshold, tight
# threshold, and automatically determined threshold
edges = cv2.Canny(blurred, 10, 200)
#edges = cv2.Canny(blurred, 225, 250)
#edges = auto_canny(blurred)


plt.figure("Original")
plt.imshow(img)
plt.show()

#plt.figure("Canny Edges")
#plt.imshow(edges)
#plt.show()

# Converting image to a binary image
# ( black and white only image).
ret,thresh = cv2.threshold(edges, 110, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
#plt.figure("BW Image")
#plt.imshow(thresh)
#plt.show()
  
# Detecting contours in image.
image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

#count = 1

#Take first contour detected
cnt = contours[0] #Take first contour detected

# Going through every contours found in the image.
#for cnt in contours :
approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
  
# draws boundary of contours.
offset = cnt.min(axis=0)
cnt = cnt - cnt.min(axis=0)
max_xy = cnt.max(axis=0) + 1
w,h = max_xy[0][0], max_xy[0][1]

# draw on blank canvass
name = "Contour " #+ str(count) 
canvass = np.zeros((h+100,w+100,3), np.uint8)
canvass.fill(255)
cv2.drawContours(canvass, [approx], 0, (0, 0, 0), 5)

  
# Used to flatted the array containing
# the co-ordinates of the vertices.
n = approx.ravel() 
i = 0
  
for j in n :
	if(i % 2 == 0):
            x = n[i]
            y = n[i + 1]
  
            # String containing the co-ordinates.
            string = str(x) + ", " + str(y) 
  
            if(i == 0):
                # text on topmost co-ordinate.
		print(string)
		cv2.circle(canvass, (x,y), 10, (0,0,255))
                cv2.putText(canvass, string, (x+10, y-20 ), 
                          font, 0.5, (255, 0, 0)) 
            else:
                # text on remaining co-ordinates.
		cv2.circle(canvass, (x,y), 10, (0,0,255))
                cv2.putText(canvass, string, (x+10, y-20), 
                          font, 0.5, (255, 0, 0)) 
		print(string)
	
        i = i + 1
    
# Showing the contour images
plt.figure(name)
plt.imshow(canvass)
plt.show()
#count = count + 1
