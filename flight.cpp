#include <iostream>
#include <termios.h>
#include <fcntl.h> // includes for enabling serial communication and mavlink library
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <c_library_v2/common/mavlink.h>
#include <c_library_v2/ardupilotmega/ardupilotmega.h>
#include <inttypes.h>

 
int main(int charc, char **argv)
{

// declaring variables and Struct inheritances to break down message data
//
uint8_t cp;
uint8_t msgReceived;
mavlink_command_long_t comm = { 0 };// used to send commands to the pixhawk, 
mavlink_scaled_imu_t simu;
mavlink_scaled_imu2_t simu2;
mavlink_scaled_imu3_t simu3;
mavlink_raw_imu_t rimu;
mavlink_statustext_t st;
mavlink_set_position_target_local_ned_t sp;
mavlink_command_ack_t ack;
mavlink_heartbeat_t hb;
mavlink_status_t laststatus;
mavlink_message_t message;
mavlink_message_t msg;
mavlink_status_t status;
pthread_mutex_t lock;
bool debug = false;
uint8_t system_ID = 0;
uint8_t companion_ID = 0;
uint8_t buff[MAVLINK_MAX_PACKET_LEN];// buffer size is the maximum size of a mavlink packet
mavlink_highres_imu_t himu;
//uint8_t len;
mavlink_param_value_t pvl;
mavlink_position_target_local_ned_t ip;
mavlink_attitude_t att;

std::cout<<"Opening Port...\n";
// opening the serial port, on the pi4B it was ttyACM0, please check the serial port your pixhawk is connected to and if the device is being read
int fd = open("/dev/ttyACM0", O_RDWR);

struct termios tty;
if(tcgetattr(fd, &tty) < 0) //setting parameters, try to follow these as default and change if needed
{
std::cout<<"Error could not read configuration";
}
tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
tty.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
#ifdef OLCUC
tty.c_oflag&= ~OLCUC;
#endif
#ifdef ONOEOT
tty.c_oflag &= ~ONOEOT
#endif
tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
tty.c_cflag &= ~(CSIZE | PARENB);
tty.c_cflag |= CS8;
tty.c_cc[VMIN] = 1;
tty.c_cc[VTIME] = 10;
cfsetispeed(&tty, B921600);//baud rate
cfsetospeed(&tty, B921600);
if(tcsetattr(fd, TCSAFLUSH, &tty) < 0)
{
fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
}

for (int i=1; i<=MAVLINK_MAX_PACKET_LEN; i++)// reading messages from the open port
{
pthread_mutex_lock(&lock);
int result = read(fd, &cp, 1);
pthread_mutex_unlock(&lock);
if (result > 0)
{
msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);// parse char function decodes the hexadecimal message into strings
if( (laststatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
{
printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
unsigned char v=cp;
fprintf(stderr,"%02x ", v);
}
laststatus = status;
}
else
{
std::cout<<"couldn't read from port";
}
if(msgReceived > 0)// successfully recieved mavlink messages
{
printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
// msgid 0 is the most common, the heartbeat message
fprintf(stderr,"Received serial data: ");// recieve the message in hexadecimal form, study this if you want to understand the protocol in detail
unsigned int i;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
if (messageLength > MAVLINK_MAX_PACKET_LEN)
{
fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
}

else
{ 
for (i=0; i<messageLength; i++)
{
unsigned char v=buffer[i];
fprintf(stderr,"%02x ", v);
}
fprintf(stderr,"\n");
}
}
// checking message id and reading specific message parameters when a message of interest is detected
// check the mavlink documentation to see the various messages, their parameters and ID
if (message.msgid == 84)
{
mavlink_msg_set_position_target_local_ned_decode(&message, &sp);
printf("frame; %u\n", sp.coordinate_frame);
}
if (message.msgid == 77)
{
mavlink_msg_command_ack_decode(&message, &ack);
printf("command; %u       result: %u\n", ack.command, ack.result);
}
if (message.msgid == 22)
{
mavlink_msg_param_value_decode(&message, &pvl);
printf("id; %s       value: %f \n", pvl.param_id, pvl.param_value);
}
if (message.msgid == 105)
{
mavlink_msg_highres_imu_decode(&message, &himu);
printf("xacc; %f       yacc: %f         zacc:%f\n", himu.xacc, himu.yacc, himu.zacc);
}
if (message.msgid == 253)
{
mavlink_msg_statustext_decode(&message, &st);
printf("severity; %u \n", st.severity);
}
}
//preparing to send a command to the controller
// MAV_CMD_REQUEST_MESSAGE requests a specific message from the autopilot
comm.target_system = 1;
comm.target_component = 0;
comm.command = MAV_CMD_REQUEST_MESSAGE;
comm.confirmation = 0;
comm.param1 = 30;//requesting message whose id is 30, which is ATTITUDE, I am requesting the attitude
comm.param2 = 0;// do not forget the coordinate frame in which the vehicle is set,
comm.param3 = 0; // you can check the coordinate frame by requesting message 85
comm.param4 = 0;
comm.param5 = 0;
comm.param6 = 0;
comm.param7 = 0;


mavlink_msg_command_long_encode(system_ID, companion_ID, &message, &comm);// encoding message for sending to the FC

int len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);

pthread_mutex_lock(&lock);
int byteswritten = write(fd, buff, len);//writing over the serial port to the autopilot
tcdrain(fd);// wait for message to be sent
pthread_mutex_unlock(&lock);
sleep(2);// give autopilot a little breather

for (int i=1; i<=MAVLINK_MAX_PACKET_LEN; i++)// reading more messages
{
pthread_mutex_lock(&lock);
int result = read(fd, &cp, 1);
pthread_mutex_unlock(&lock);
if (result > 0)
{
msgReceived = mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
if( (laststatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
{
printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
unsigned char v=cp;
fprintf(stderr,"%02x ", v);
}
laststatus = status;
}
else
{
std::cout<<"couldn't read from port";
}
if(msgReceived > 0)
{
printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);
fprintf(stderr,"Received serial data: ");
unsigned int i;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
if (messageLength > MAVLINK_MAX_PACKET_LEN)
{
fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
}

else
{ 
for (i=0; i<messageLength; i++)
{
unsigned char v=buffer[i];
fprintf(stderr,"%02x ", v);
}
fprintf(stderr,"\n");
}
}
//All these messages are messages I tried to read on various attempts and was successful
if (message.msgid == 85)
{
mavlink_msg_position_target_local_ned_decode(&message, &ip);
printf("frame; %u\n", ip.coordinate_frame);
printf(" %f\n", ip.vx);
printf(" %f\n", ip.vy);
printf(" %f\n", ip.vz);
}
if (message.msgid == 30)// looking for message #30 after requesting it, should see it if successful
{
mavlink_msg_attitude_decode(&message, &att);
printf("roll: %f\n", att.roll);
printf("pitch: %f\n", att.pitch);
printf("yaw: %f\n", att.yaw);
}
if (message.msgid == 77)
{
mavlink_msg_command_ack_decode(&message, &ack);
printf("command; %u       result: %u\n", ack.command, ack.result);
}
if (message.msgid == 26)
{
mavlink_msg_scaled_imu_decode(&message, &simu);
printf("xacc: %" PRIu16 "\n", simu.xacc);
printf("yacc: %" PRIu16 "\n", simu.yacc);
printf("zacc: %" PRIu16 "\n", simu.zacc);
}
if (message.msgid == 116)
{
mavlink_msg_scaled_imu2_decode(&message, &simu2);
printf("xacc: %" PRIu16 "\n", simu2.xacc);
printf("yacc: %" PRIu16 "\n", simu2.yacc);
printf("zacc: %" PRIu16 "\n", simu2.zacc);
}

if (message.msgid == 27)
{
mavlink_msg_raw_imu_decode(&message, &rimu);
printf("xacc: %" PRIu16 "\n", rimu.xacc);
printf("yacc: %" PRIu16 "\n", rimu.yacc);
printf("zacc: %" PRIu16 "\n", rimu.zacc);
}
if (message.msgid == 253)
{
mavlink_msg_statustext_decode(&message, &st);
printf("severity; %u \n", st.severity);
}

}


close (fd);// close serial port
return 0;
}
