import cv2
camera = cv2.VideoCapture(0)
grab,frame=camera.read()
cv2.namedWindow('robots')
cv2.imshow('robots',frame)
cv2.waitKey(0)
