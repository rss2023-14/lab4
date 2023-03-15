import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

	min_orange = np.array([9,100,185])  #hsv
	max_orange = np.array([30,255,255]) #hsv 
	# min_orange = np.array([0,0,185]) #rgb
	# max_orange = np.array([100,255,255]) #rgb
	
	mask = cv2.inRange(hsv_img,min_orange,max_orange)  # hsv
	
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cone_contour = max(contours, key=cv2.contourArea)
	x,y,w,h = cv2.boundingRect(cone_contour)

	boundingbox = ((x,y),(x+w,y+h))

	

	return boundingbox

