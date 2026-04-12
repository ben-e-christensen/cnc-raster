import cv2
from picamera2 import Picamera2

# Initialize the blazing-fast native camera pipeline
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

print("Press 'q' to quit the video window.")

while True:
    # Grab the frame directly as an OpenCV-ready numpy array
    frame = picam2.capture_array()
    
    # Display the frame
    cv2.imshow('Pi Camera Feed', frame)
    
    # Check for 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
picam2.stop()
cv2.destroyAllWindows()