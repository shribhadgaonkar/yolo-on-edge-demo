# yolo-on-edge-demo
First successful live YOLOv8 Object detection on HiWonder Jet Auto pro robot (Jetson Orin Nano) using ROS2 humble and peripheral camera package.


# First Live YOLOv8 Detection on HiWonder JetAuto Pro

**Date:** February 13, 2025  
**Hardware:** HiWonder JetAuto Pro (Jetson Orin Nano 8GB)  
**Software:** ROS 2 Humble, Ultralytics YOLOv8, peripherals camera package

## Live Demo GIFs

<p align="center">
  <img src="media/hiWonder_Teleop_twist.gif" width="45%" alt="Teleop keyboard control" />
  <img src="media/Hiwonmder_yolo_detection.gif" width="45%" alt="YOLOv8 live detection" />
</p>

<p align="center">
  <strong>Left:</strong> Keyboard teleop driving the robot<br>
  <strong>Right:</strong> YOLOv8 detecting objects in real time
</p>

## Problem
The pre-trained YOLOv8 model (COCO-trained) was not detecting small Hot Wheels toy cars reliably when many cars were scattered on the floor. It only detected a single car when placed very close (~0.65 confidence).
Also this model is givinng wrong detection of the car such as toothbrush and other non relevant findings.


## Solution Journey

1. After completing my initial setup, I moved teh robot with inbuilt teleop commands. Robot moved promisingly.

2. When I started, testing for the camera feed. I was not able to initialize the camera when I used command `ros2 launch peripherals depth_camera.launch.py`
    - camera initialization failed wit this Perror (`uvc_open path=... failed, res-6`)
    - Camera used: "ORBBEC Depth Camera FHD 1080P"
    - Workaround: reload uvcvideo driver before launch
    - `sudo modprobe -r uvcvideo && sudo modprobe uvcvideo`
    - After this I was able to see the camera topic being published and verified the feed with following command
        `ros2 run rqt_image_view rqt_image_view /depth_cam/rgb/image_raw`
        This command is used to visualise a live vidoe stream from the robots camera. It opens a graphical window and shows what a HiWonder robot is seeing in rela time.
        breakdown 
        `ros2_run` : Standard ROS2 command to start an executable from a specific package
        `rqt_image_view` : This is a name of the package and contains suite of graphical tools
        `rqt_image_view` : This is name of the executable inside the rqt_image_view package
        `depth_cam/rgb/image_raw` : This is the name o fthe package that we are listening, which is published by the camera.

3. Next i faced issues in installing Ultralytics despite NumPy C-API conflicts
   - Downgraded to NumPy 1.23.5 --user --force-reinstall
   - Used `--no-deps` on reinstall to avoid new conflicts

3. Next I created a simple python file which contains following operations
    - Create a Node "YOLOCamera"
    - Created minimal ROS subscriber which Subscribes to `/depth_cam/rgb/image_raw`
    -  Added a CvBridge to translate the ROS images to OpenCV images
    - Runs inference with YOLOv8n
    - Visualizes with OpenCV `cv2.imshow`
    - I tested the YOLO output while driving the robot with command `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Current Result
- Live detection window shows bounding boxes and labels
- Single close car detected as "car" ~0.65 conf
- Multiple distant/small cars not yet detected reliably (expected – pre-trained model)

## Next Steps
- I am planning to collect 200–500 images of Hot Wheels cars from robot camera
- Label with Roboflow → train custom YOLOv8
- Publish detections as `vision_msgs/Detection2DArray`
- Integrate with Nav2 → drive toward detected car



## Code
See `src/yolo_camera.py`