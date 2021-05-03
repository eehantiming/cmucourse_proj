
# Python code to find the co-ordinates of
# the contours detected in an image.
import numpy as np
import cv2
# from pyrobot.utils.util import try_cv2_import
# cv2 = try_cv2_import()
import matplotlib.pyplot as plt

def auto_canny(image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)
        # return the edged image
        return edged

def get_vertices(imgFile):  
        # define font
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Reading image and converting to gray scale.
        img = cv2.imread(imgFile)
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
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #count = 1

        #Take first contour detected
        cnt = contours[0] #Take first contour detected

        # Going through every contours found in the image.
        #for cnt in contours :
        approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
          
        # Find boundary of contours.
        offset = cnt.min(axis=0)
        cnt = cnt - cnt.min(axis=0)
        max_xy = cnt.max(axis=0) + 1
        w,h = max_xy[0][0], max_xy[0][1]

        print("Width : ", w)
        print("Height :", h)

        # Create and draw contour on blank canvass
        name = "Contour " #+ str(count) 
        canvass = np.zeros((h+100,w+100,3), np.uint8)
        canvass.fill(255)
        cv2.drawContours(canvass, [approx], 0, (0, 0, 0), 5)

        # X-axis and Y-axis min max boundary
        lx = 0.2
        rx = -0.2
        ty = 0.4
        by = 0.22

        # create scale and shift factors
        dx = rx - lx
        dy = ty - by
        scaleX = w / dx
        scaleY = h / dy
          
        # Used to flatted the array containing
        # the co-ordinates of the vertices.
        n = approx.ravel() 
        i = 0
          
        pts = []

        for j in n :
                if(i % 2 == 0):
                    x = n[i]
                    y = n[i + 1]
          
                    # Calculate the shift and scaled x and y
                    ix = x -(w/2)
                    iy = h - y
                    fx = round(ix/scaleX, 2)
                    fy = round((by + iy/scaleY), 2)

                    # String containing the co-ordinates.
                    string = str(x) + ", " + str(y) 
          
                    if(i == 0):
                        # text on topmost co-ordinate.
                        cv2.circle(canvass, (x,y), 10, (0,0,255))
                        cv2.putText(canvass, string, (x+10, y-20 ), 
                                  font, 0.5, (255, 0, 0)) 
                        # pts.append([fx, fy])
                        pts.append([fy, fx])
                        x0 = fx
                        y0 = fy
                    else:
                        # text on remaining co-ordinates.
                        cv2.circle(canvass, (x,y), 10, (0,0,255))
                        cv2.putText(canvass, string, (x+10, y-20), 
                                  font, 0.5, (255, 0, 0)) 
                        print(string)
                        # pts.append([fx, fy])
                        pts.append([fy, fx])
                
                i = i + 1
            
        # Showing the contour images
        plt.figure(name)
        plt.imshow(canvass)
        plt.show()

        px = []
        py = []

        plt.figure("Scaled")
        for pt in pts:
          px.append(pt[0])
          py.append(pt[1])

        # zip joins x and y coordinates in pairs
        for x1,y1 in zip(px,py):
            label = "(" + str(x1) + ", " + str(y1) + ")" 
            plt.annotate(label, (x1,y1), 
                        textcoords="offset points", xytext=(0,10), 
                        ha='center')

        #Draw plots from scaled contour points
        plt.scatter(px, py)
        print(pts)

        #Append first point as last point to close the plot
        px.append(x0)
        py.append(y0)
        plt.plot(px, py)
        plt.show()

        #pts.append([x0, y0])
        return pts

if __name__ == "__main__":
    points = get_vertices('house.jpg')
