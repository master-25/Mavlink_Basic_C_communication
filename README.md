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

The purpose of doing this was because when I tried to do low level mavlink commuication, I couldnt find resources to quickly implement. More modular programs like the 

https://github.com/mavlink/c_uart_interface_example by Lorenz Meier were a bit overwhelming and I had to really look deepy into each function call

This is meant to be a starting point for those who wish to implement specific and targeted applications using mavlink like testing advanced control algorithms or wish to perform vision based autoomous flight, this might be of use to you as a starting point

Do let me know if there is anything that is unclear, inaccurate and/or could be improved
