![IMG20240112153750](https://github.com/FocasLab/tortoisebot/assets/99411053/540963a5-51c7-4cb7-8e46-a4ce1e2b316e)# TortoiseBot Mini

## 1. Installation:

### 1.1 Download Arduino IDE:

Download the latest version Arduino IDE from https://www.arduino.cc/en/software

### 1.2 Setup ESP board in Arduino IDE:

Follow the following tutorial to add ESP board in your Arduino IDE - https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/

Simply go to Files > Preferences and add the following line in the Additional Board Managers URLs:
```
https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json
```
After that go into Tools > Board > Boards Manager and search for "esp32" and install the ESP32 board. That's it!

### 1.3 Install ROS Serial Library for Arduino on Ubuntu:
```
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```



## 2. Setup:
### 2.0 Creating a ROS workspace and cloning the repository
Create a ROS workspace, let's call our workspace tb_ws
```
mkdir -p ~/tb_ws/src
```
Change directory to tb_ws/src folder
```
cd tb_ws/src
```
Clone the TortoiseBot Mini repo into your src folder of your ROS workspace
```
git clone https://github.com/FocasLab/tortoisebot
```
Change directory to tb_ws 
```
cd ..
```
Build or make your workspace
```
catkin_make
```
or 
```
catkin build
```

### 2.1 Build the Arduino serial library and msgs 
Navigate to Arduino > libraries 
Open a terminal in the current directory 
```
rosrun rosserial_arduino make_libraries.py .
```
This should build the required msgs 

In your linux system in home,in Arduino folder-->Library the ros_lib folder will be present after building using the above command
Inside this ros_lib folder create a new folder tortoisebot_mini and paste the [diff.h](https://github.com/FocasLab/tortoisebot/blob/tortoisebot_encoder/Diff.h) file inside

CONNECTIONS- 
There are 6 connecting wire from motor-
Blue - Motor negative pole
Green - Motor positive pole
Yellow - Hall Power positive - connect to 5V pin(18,19 and 16,17)
Orange - Hall H2 output signal - connect to gpio pin
Red - Hall H1 output signal - connect to gpio pin
Brown - Hall power negative - connect to ground pin


// Motor 1  pins and encoder variables
const int encoderPinA1 = 18;
const int encoderPinB1 = 19;
const int motorPWMPin1 = 32; 
const int motorDirPin1_1 = 25; 
const int motorDirPin1_2 = 26; 

// Motor 2  pins and encoder variables
const int encoderPinA2 = 16;

const int encoderPinB2 = 17;
const int motorPWMPin2 = 33; 
const int motorDirPin2_1 = 13; 
const int motorDirPin2_2 = 27; 

![IMG20240112153750](https://github.com/FocasLab/tortoisebot/assets/99411053/c2545423-1dbb-4566-b538-202bffb4bc21)

![IMG20240112160443](https://github.com/FocasLab/tortoisebot/assets/99411053/3e3da45a-0d03-4d5d-a5a9-f11a900e3336)



### 2.2 Change Wi-Fi's SSID and Password:

Open the [tortoisebot_mini_ros](https://github.com/FocasLab/tortoisebot/blob/tortoisebot_encoder/esp/tortoisebot_mini_ros/tortoisebot_mini_ros.ino) code in Arduino IDE and change the SSID and Password of the Wi-Fi Connection.

```
const char* ssid = "";
const char* password = "";
```

### 2.3 Change the ROS Master IP Address:

Run the following command on the machine which will act as ROS Master PC (in our case the lab computer) after connecting to the lab's wifi.
```
hostname -I
```
Use the this IP address the .ino file.
```
IPAddress server(192,168,x,x);
```

Check the IP Address of your ROS Master PC using ifconfig command and put that in line number 13 of [tortoisebot_mini_ros](https://github.com/FocasLab/tortoisebot/blob/tortoisebot_encoder/esp/tortoisebot_mini_ros/tortoisebot_mini_ros.ino) code with commas(,) instead of dot(.)

### 2.4 Check Board and Port:

Make sure the switch on motor driver board is turned off and battery is not connected and then connect your ESP32 board to your computer using USB type B cable. Make sure the board is selected to "ESP32 Dev Module" in Arduino IDE and select the appropriate Port as well.

### 2.5 Upload the Code:

Upload the [tortoisebot_mini_ros](https://github.com/FocasLab/tortoisebot/blob/tortoisebot_encoder/esp/tortoisebot_mini_ros/tortoisebot_mini_ros.ino) code on your ESP32 board.

### 2.6 Connection:

Once code is Successfully uploaded, disconnect the USB Cable and connect the DC Jack of Battery to the Board . <br>
DO NOT TURN ON THE ROBOT YET!

### 2.7 Connecting your laptop to ROS Master:

After Setting up the robot and uploading the code we will now setup the ROS on Multiple Machines. Add this lines to end of you ```.bashrc``` file.

```
export ROS_IP=<IP of your machine>
export ROS_MASTER_URI=http://<IP of Master Machine>:11311
```
Example:-

```
export ROS_IP=192.168.0.203
export ROS_MASTER_URI=http://192.168.0.127:11311
```

## 3. Demo:

### 3.1 Running ROS Master:

Launch the [tortoisebot_mini.launch](https://github.com/FocasLab/tortoisebot/blob/main/launch/tortoisebot_mini.launch) file on your Master PC to start ROS Master along with ROS Serial Node.

```
roslaunch tortoisebot_mini tortoisebot_mini.launch
```
It will prompt a INFO message saying "Waiting for socket connections on port 11411 waiting for socket connection"<br>

### 3.2 Turning on Robot:

Once the above INFO message is popped up, then you can turn on your Robot upon which you will see that different ROS topics start automatically.

### 3.3 Testing out Teleop:

Keep the previous terminal running add on a new terminal type the following command to start the teleoperation node.
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
> Note that the minimum operational speed for the robot is :
> - Linear Velocity : 0.131 m/s
> - Angular Velocity : 2.175 r/s

Now you can control the robot using teleoperation commands.

If the Twist keyboard does not work try directly publishing the velocity commands 

rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'



## References:
- http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
- https://github.com/agnunez/espros
- https://github.com/RoboTakao/NX15A
