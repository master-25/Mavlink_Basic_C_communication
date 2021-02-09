# Mavlink_Basic_C_communication
A sample code to show how to send commands and recieve messages from a mavlink based autopilot in C++ using the C library
This is a sample code for reading messages from a pixhawk flight controller and sending commands in C++ using the mavlink C library on a linux based companion computer
I have used an old pixhawk 2.4.6 and raspberry pi 4b (4gb ram) running raspbian buster for the task. But there is no reason it shouldn't work on an nvidia jetson or odroid
all the included headers should be there by default except the mavlink library

use the commands

sudo apt install git

then go to the include directory

cd /usr/include or cd /usr/local/include

then clone the mavlink directory

git clone https://github.com/mavlink/c_library_v2.git

that should be enough

please check what devce name the pixhawk connection shows when you connect it

