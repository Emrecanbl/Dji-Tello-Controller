DJI Tello Controller with Raspberry Pi and Pi Pico
This project is a comprehensive DJI Tello controller using Raspberry Pi and Pi Pico. The controller integrates face detection and following, joystick and IMU-based control, video streaming to an ST7735 screen, and is being extended with TensorFlow Lite for object detection and keyword detection for mode selection.

![Sample](https://github.com/Emrecanbl/Dji-Tello-Controller/blob/main/IMG_20240708_140726.jpg?raw=true)

Features
Face Detection and Following: Implemented using OpenCV.
IMU and Joystick Control: Managed by Raspberry Pi Pico and communicated via UART to the Raspberry Pi.
Video Streaming: Raspberry Pi streams video from the DJI Tello to an ST7735 screen.
Upcoming Features: Object detection and following with TensorFlow Lite, keyword detection with TensorFlow for mode selection.
Components
Raspberry Pi 4: Main processing unit for video streaming and advanced features.
Raspberry Pi Pico: Manages IMU and joystick inputs, communicates with Raspberry Pi 4 via UART.
DJI Tello: The drone being controlled.
ST7735 SPI Screen: Displays video stream from DJI Tello.
Microphone: For future keyword detection features.
IMU: For motion-based control.
Joystick: For manual control.
Installation
Clone the repository:


git clone https://github.com/Emrecanbl/Dji-Tello-Controller.git

Set up the environment:

Install necessary dependencies for Raspberry Pi (e.g., OpenCV, TensorFlow Lite).
Configure the UART communication between Raspberry Pi 4 and Raspberry Pi Pico.
Set up the ST7735 SPI screen.
Load the code:

Load the provided code onto the Raspberry Pi and Raspberry Pi Pico.
Usage
Power on the system:
Ensure all components are powered and connected properly.

Initialize the system:
The Raspberry Pi will start video streaming and face detection using OpenCV.

Control the DJI Tello:

Use the joystick for manual control.
Utilize IMU for motion-based control.
Face detection and following will automatically activate when a face is detected.

![Sample](https://github.com/Emrecanbl/Dji-Tello-Controller/blob/main/IMG_20240708_140659.jpg?raw=true)

Upcoming Features:

Object detection and following will be added using TensorFlow Lite.
Keyword detection for mode selection will be integrated.
Detailed Breakdown
Face Detection and Following
Implemented using OpenCV on the Raspberry Pi 4.
Detects faces in the video stream and sends commands to the DJI Tello to follow the detected face.
IMU and Joystick Control
Managed by Raspberry Pi Pico.
Communicates with Raspberry Pi 4 via UART.
Allows for manual and motion-based control of the DJI Tello.
Video Streaming
Raspberry Pi 4 streams video from the DJI Tello to the ST7735 SPI screen.
Provides real-time visual feedback for the user.
Future Enhancements
Object Detection and Following: Will be implemented using TensorFlow Lite on Raspberry Pi 4.
Keyword Detection: Raspberry Pi 4 will use TensorFlow for recognizing keywords to change control modes.
Troubleshooting
No Video Stream: Check the connection and configuration of the ST7735 SPI screen.
Control Issues: Verify the UART communication between Raspberry Pi 4 and Raspberry Pi Pico.
Face Detection Not Working: Ensure OpenCV is properly installed and configured.
Contributing
Contributions are welcome! If you have any ideas, suggestions, or improvements, feel free to open an issue or submit a pull request.

License
This project is licensed under the MIT License. See the LICENSE file for more details.

Acknowledgments
Special thanks to the open-source community and the developers of the libraries and tools used in this project.

