# Face Tracking and Motor Control using OpenCV and Raspberry Pi

This project demonstrates how to use a Raspberry Pi with OpenCV for face tracking and controlling motors based on the detected face's position. The setup uses a webcam for capturing video, the `haarcascade_frontalface_default.xml` for face detection, and the `adafruit_servokit` for controlling servos.

## Prerequisites

Ensure you have the following installed and configured:

1. **Hardware:**
   - Raspberry Pi (any model with GPIO support)
   - Webcam
   - Servo motor
   - Stepper motor
   - Adafruit Servo Kit
   - Required connections for motors to GPIO pins

2. **Software:**
   - Python 3
   - OpenCV
   - Adafruit Servo Kit library
   - imutils (for FPS calculation)
   - RPi.GPIO (for GPIO control)

## Installation

1. **Update and Upgrade your Raspberry Pi:**

    ```bash
    sudo apt-get update
    sudo apt-get upgrade
    ```

2. **Install Python and pip:**

    ```bash
    sudo apt-get install python3 python3-pip
    ```

3. **Install OpenCV:**

    ```bash
    pip3 install opencv-python opencv-python-headless
    ```

4. **Install Adafruit Servo Kit library:**

    ```bash
    pip3 install adafruit-circuitpython-servokit
    ```

5. **Install imutils:**

    ```bash
    pip3 install imutils
    ```

6. **Install RPi.GPIO:**

    ```bash
    pip3 install RPi.GPIO
    ```

## Running the Code

1. **Download or Clone the Repository:**

    ```bash
    git clone <repository-url>
    cd <repository-directory>
    ```

2. **Ensure the haarcascade file is present:**

    The code uses `haarcascade_frontalface_default.xml` for face detection. Make sure this file is in the same directory as the script or provide the correct path to it.

3. **Run the script:**

    ```bash
    python3 test_BT.py
    ```

## Code Explanation

### motorInput Function

This function takes an angle as input and controls a stepper motor to move a specified number of steps, based on the input angle.

### Main Loop

1. **Face Detection:**
   - Captures video frames from the webcam.
   - Converts frames to grayscale for face detection using the Haar Cascade classifier.
   - If a face is detected, initializes a MOSSE tracker with the face bounding box.

2. **Tracking and Motor Control:**
   - Continuously tracks the face in subsequent frames.
   - Calculates the error between the face position and the frame center.
   - Converts this error to angles (`thetaX` and `thetaY`) and adjusts servo motor angles accordingly.

3. **Display:**
   - Displays the video frame with the bounding box around the detected face.
   - Shows FPS and tracker status on the frame.

### Exiting the Program

Press `q` to exit the program. The script will close the video window and perform necessary cleanup.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- OpenCV for providing the tools for image processing.
- Adafruit for their comprehensive library for controlling servos.
