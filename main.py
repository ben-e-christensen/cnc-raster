import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

print("Press 'q' to quit the video window.")

while True:
    frame = picam2.capture_array()
    
    cv2.imshow('Pi Camera Feed', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()