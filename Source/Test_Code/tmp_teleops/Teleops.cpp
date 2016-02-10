#include <iostream>
#include <ros.h>
#include <std_msgs/String.h>

using namespace std;

// PUBLISH METHOD FOR DRIVE CONTROL
std_msgs::String str_msg;
ros::Publisher drive("drive", &str_msg);
void drive_publisher(String msg) {
	char const * temp = msg.c_str();
  str_msg.data = temp;
  drive.publish(&str_msg);
}

// PUBLISH METHOD FOR ARM CONTROL
ros::Publisher arm("arm", &str_msg);
void arm_publisher(String msg) {
	char const * temp = msg.c_str();
  str_msg.data = temp;
  arm.publish(&str_msg);
}

int main() {
	char input, input2='D';
	cin >> input;
	while(input != input2) {
		//DRIVE CONTROLS
		//ros::Publisher drive = nh.advertise<std_msgs::String>("drive", 5);
		//std_msgs::String str;

		if(input == 'e') {
			cout << "drive forward" << endl;
			drive_publisher("e");
		}
		else if(input == 'd') {
			cout << "drive backwards" << endl;
			drive_publisher("d");
		}
		else if(input == 's') {
			cout << "turn left" << endl;
			drive_publisher("s");
		}
		else if(input == 'f') {
			cout << "turn right" << endl;
			drive_publisher("f");
		}

		//ARM CONTROLS
		if(input == 'y') {
			cout << "y key" << endl;
			arm_publisher("y");
		}
		else if(input == 'u') {
			cout << "u key" << endl;
			arm_publisher("u");
		}
		else if(input == 'i') {
			cout << "i key" << endl;
			arm_publisher("i");
		}
		else if(input == 'o') {
			cout << "o key" << endl;
			arm_publisher("o");
		}
		else if(input == 'h') {
			cout << "h key" << endl;
			arm_publisher("h");
		}
		else if(input == 'j') {
			cout << "j key" << endl;
			arm_publisher("j");
		}
		else if(input == 'k') {
			cout << "k key" << endl;
			arm_publisher("k");
		}
		else if(input == 'l') {
			cout << "l key" << endl;
			arm_publisher("l");
		}

		//Get more input
		cin >> input;
	}

	cout << "done" << endl;
	return 0;
}
