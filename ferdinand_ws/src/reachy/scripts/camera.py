#!/usr/bin/env python3
import rospy

# Message Object Format to send face detection datas
from ferdinand_msgs.msg import FaceDetectionData

# OpenCv for computer vision
import cv2

# For face detection
import mediapipe as mp

# To detect a face in a frame
mp_face_detection = mp.solutions.face_detection

# To draw around the face
mp_drawing = mp.solutions.drawing_utils

# Publisher to send face detection data to the topic : "/face_detection_data"
face_detection_data_publisher = rospy.Publisher("/face_detection_data", FaceDetectionData, queue_size=1)

# Function that uses the publisher to send face detection data dimension 
def publish_face_detection_data(height, width, xmin, ymin, xmax, ymax):
	msg = FaceDetectionData()
	msg.height = height
	msg.width = width
	msg.xmin = xmin
	msg.ymin = ymin
	msg.xmax = xmax 
	msg.ymax = ymax
	face_detection_data_publisher.publish(msg)


if __name__ == '__main__':
    
    # Node initialization
	rospy.init_node('camera')
 
 	# Node initialization message printing
	rospy.loginfo("Camera Node has been started")

	# Rate fixation for the spin threads
	rate = rospy.Rate(10)
	
	# Reading video from the first source with OpenCv
	cap = cv2.VideoCapture(0)

	# As long as the node is running, I get the frames from the camera
	while not rospy.is_shutdown():
		with mp_face_detection.FaceDetection(min_detection_confidence=0.5) as face_detection:
			while True:
				
				# I get a frame from the camera
				success, image = cap.read()
				if not success:
					print("Ignoring empty camera frame.")
					break

				# Convert the BGR image to RGB.
				image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
				image.flags.writeable = False

				# Face detection in the current frame
				results = face_detection.process(image)

				# Draw the face detection annotations on the image.
				image.flags.writeable = True
				image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
				# get dimensions of image
				dimensions = image.shape
				
				# height, width, number of channels in image
				height = image.shape[0]
				width = image.shape[1]
				channels = image.shape[2]

				# If we have detected at least one face in the frame courent
				if results.detections:
					
					# We browse each detected face and retrieve the data
					for detection in results.detections:
						bbox = detection.location_data.relative_bounding_box
						xmin = int(bbox.xmin * width)
						ymin = int(bbox.ymin * height)
						xmax = int(bbox.width * width + bbox.xmin * width)
						ymax = int(bbox.height * height + bbox.ymin * height)
						if xmin < 0:
							xmin = 0
						if ymin < 0:
							ymin = 0
						if xmax > width:
							xmax = width
						if ymax > height:
							ymax = height

						# We publish the data for each face detected on the topic : "/face_detection_data"
						publish_face_detection_data(height, width, xmin, ymin, xmax, ymax)
						mp_drawing.draw_detection(image, detection)
				

				cv2.imshow('Face Detection', image)
				if cv2.waitKey(5) & 0xFF == 27:
					break
		cap.release()
  
		rate.sleep()