import cv2
from cv2 import aruco

# get dictionary and parameters
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Create ArucoDetector object
detector = aruco.ArucoDetector(dictionary, parameters)

# read from image
input_file = "markers_0_to_5.png"
output_file = "markers_0_to_5_drawing.png"
input_img = cv2.imread(input_file)

# detect and draw marker's information
corners, ids, rejectedCandidates = detector.detectMarkers(input_img)
print(ids)
ar_image = aruco.drawDetectedMarkers(input_img, corners, ids)

cv2.imwrite(output_file, ar_image)
