import cv2
import os
# Initialize the camera
cap = cv2.VideoCapture(4)

# Initialize frame counter
frame_count = 0

# Check if the camera opened successfully
if not cap.isOpened():
    print("Failed to open camera")
    exit()

folder_name = 'Image_For_Calib'

# Check if the folder exists
if not os.path.exists(folder_name):
    os.makedirs(folder_name)

try:
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if ret:
            # Display the frame
            cv2.imshow('Camera', frame)

            # Wait for 's' key to be pressed to save the frame
            if cv2.waitKey(1) & 0xFF == ord('s'):
                # Construct the filename with frame number
                filename = os.path.join(folder_name, '{}.jpg'.format(frame_count))
                # Save the frame as a JPEG file
                cv2.imwrite(filename, frame)
                print('Image saved as {}'.format(filename))
                # Increment the frame counter
                frame_count += 1

        # Wait for 'q' key to be pressed to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()
