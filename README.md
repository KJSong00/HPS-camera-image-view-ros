## HPS-camera-image-view-ros

If you use this project, you can view image of HPS camera in ROS environment<br>
You can only test in 64 bit operating system

### Notice
After cloning this repostiory, please rename the directory to "hps_camera"(for preventing build failure)
You can get the SDK of the camera in [here](https://github.com/SeeedDocument/Outsourcing/raw/master/101990386/Doc/HPS3D-SDK%20software%20and%20manual.zip)

### Pre-requirements
* ROS Kinetic

### How to use
1. Clone this repository in your catkin workspace
    ```
    git clone https://github.com/KJSong00/HPS-camera-image-view-ros.git
    ```
2. Copy the .so file 
    ```
    sudo cp ./include/*.so /usr/local/lib/
    sudo ldconfig
    ```

3. Build and Test
    ```
    catkin build
    source ~/catkin_ws/devel/setup.bash
    roscore
    rosrun <this package name> ros_server_client
    rosrun <this package name> ros_camera_client
    ````

4. View image
    ```
    rqt
    ```
    and select the /hps3d_image topic to view image

### Test screen
<img src="https://github.com/KJSong00/HPS-camera-image-view-ros/blob/master/img/test_screen.png">