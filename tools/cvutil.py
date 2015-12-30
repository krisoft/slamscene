import cv2

def wait():
	while True:
		c = cv2.waitKey(20)
		if c!=-1:
			return c