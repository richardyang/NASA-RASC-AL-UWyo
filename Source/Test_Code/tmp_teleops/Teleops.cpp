#include <iostream>
#include <ros.h>
#include <std_msgs/String.h>

using namespace std;

std_msgs::String str_msg;
ros::Publisher drive("drive", &str_msg);
void drive_publisher(String msg)
{
	char const * temp = msg.c_str();
  str_msg.data = temp;
  drive.publish(&str_msg);
}

ros::Publisher arm("arm", &str_msg);
void arm_publisher(String msg)
{
	char const * temp = msg.c_str();
  str_msg.data = temp;
  arm.publish(&str_msg);
}


int main()
{
	char input, input2='D';
	cin >> input;
	while(input != input2)
	{
		//DRIVE CONTROLS
		//ros::Publisher drive = nh.advertise<std_msgs::String>("drive", 5);
		//std_msgs::String str;

		if(input == 'e')
		{
			cout << "drive forward" << endl;
			//str.data = "e";
			//drive.publish(str);
			drive_publisher("e");
		}
		else if(input == 'd')
		{
			cout << "drive backwards" << endl;
			//str.data = "d";
			//drive.publish(str);
			drive_publisher("d");
		}
		else if(input == 's')
		{
			cout << "turn left" << endl;
			//str.data = "s";
			//drive.publish(str);
			drive_publisher("s");
		}
		else if(input == 'f')
		{
			cout << "turn right" << endl;
			//str.data = "f";
			//drive.publish(str);
			drive_publisher("f");
		}

		//ARM CONTROLS
		//ros::Publisher arm = nh.advertise<std_msgs::String>("arm", 5);
		//std_msgs::String str;
		if(input == 'y')
		{
			cout << "y key" << endl;
			//str.data = "y";
			//arm.publish(str);
			arm_publisher("y");
		}
		else if(input == 'u')
		{
			cout << "u key" << endl;
			//str.data = "u";
			//arm.publish(str);
			arm_publisher("u");
		}
		else if(input == 'i')
		{
			cout << "i key" << endl;
			//str.data = "i";
			//arm.publish(str);
			arm_publisher("i");
		}
		else if(input == 'o')
		{
			cout << "o key" << endl;
			//str.data = "o";
			//arm.publish(str);
			arm_publisher("o");
		}
		else if(input == 'h')
		{
			cout << "h key" << endl;
			//str.data = "h";
			//arm.publish(str);
			arm_publisher("h");
		}
		else if(input == 'j')
		{
			cout << "j key" << endl;
			//str.data = "j";
			//arm.publish(str);
			arm_publisher("j");
		}
		else if(input == 'k')
		{
			cout << "k key" << endl;
			//str.data = "k";
			//arm.publish(str);
			arm_publisher("k");
		}
		else if(input == 'l')
		{
			cout << "l key" << endl;
			//str.data = "l";
			//arm.publish(str);
			arm_publisher("l");
		}

		//Get more input
		cin >> input;
	}

	cout << "done" << endl;
	return 0;
}
