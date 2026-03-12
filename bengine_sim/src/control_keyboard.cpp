#include "ros/ros.h"
// #include <std_msgs/UInt8.h>
#include "bengine_legged_msgs/MotorPD.h"
// #include <chrono>
#include <termios.h>

// 简单键盘输入处理
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

//自己按需编写
void printHelp() {
    std::cout << "\n=== 四足机器人控制说明 ===" << std::endl;
    std::cout << "w/s: 站立/座下" << std::endl;
    std::cout << "a/d: " << std::endl;
    std::cout << "q/e: " << std::endl;
    std::cout << "1: 髋关节KP+1" << std::endl;
    std::cout << "2: 大腿关节KP+1" << std::endl;
    std::cout << "3: 小腿关节KP+1" << std::endl;
    std::cout << "4: 髋关节kd+1" << std::endl;
    std::cout << "5: 大腿关节kd+1" << std::endl;
    std::cout << "6: 小腿关节kd+1" << std::endl;
    std::cout << "7: " << std::endl;
    std::cout << "空格: " << std::endl;
    std::cout << "h: " << std::endl;
    std::cout << "x: 退出程序" << std::endl;
    std::cout << "========================\n" << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboard_input_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(500);
	ros::Publisher pub = nh.advertise<bengine_legged_msgs::MotorPD>("/motor_pd", 10);

	bengine_legged_msgs::MotorPD msg;
    for(int i=0;i<4;i++)
    {
        msg.KP[i*3+0] = 100;
        msg.KD[i*3+0] = 35;

        msg.KP[i*3+1] = 135;
        msg.KD[i*3+1] = 40;

        msg.KP[i*3+2] = 150;
        msg.KD[i*3+2] = 45;
    }
    
     printHelp();   

	long count = 0;

	while (ros::ok())
	{

		printf("%ld\n", count++);

		char key = getch();
        switch (key) 
        {
            case '1':
                
                msg.KP[0] ++;  
                msg.KP[0+3] ++;
                msg.KP[0+6] ++;
                msg.KP[0+9] ++;
                std::cout << "髋关节KP="<< msg.KP[0] <<std::endl;             
                break;
            case '2':
                msg.KP[1] ++;  
                msg.KP[1+3] ++;
                msg.KP[1+6] ++;
                msg.KP[1+9] ++;
                std::cout << "大腿关节.KP="<< msg.KP[1] <<std::endl;             
                break;
            case '3':
                msg.KP[2] ++;
                msg.KP[2+3] ++;
                msg.KP[2+6] ++;
                msg.KP[2+9] ++;
                std::cout << "小腿关节.KP=" << msg.KP[2] << std::endl;             
                break;
            case '4':
                
                msg.KD[0] ++;  
                msg.KD[0+3] ++;
                msg.KD[0+6] ++;
                msg.KD[0+9] ++;
                std::cout << "髋关节KD="<< msg.KD[0] <<std::endl;             
                break;
            case '5':
                msg.KD[1] ++; 
                msg.KD[1+3] ++;
                msg.KD[1+6] ++;
                msg.KD[1+9] ++;
                std::cout << "大腿关节.KD="<< msg.KD[1] <<std::endl;             
                break;
            case '6':
                msg.KP[2] ++;
                msg.KP[2+3] ++;
                msg.KP[2+6] ++;
                msg.KP[2+9] ++;
                std::cout << "小腿关节.KD=" << msg.KD[2] << std::endl;             
                break;
            case '7':
                
                std::cout << "" <<  std::endl;            
                break;
            case '8':
                std::cout << ""<< std::endl;             
                break;
            case 'w':
                msg.gait = 1;
                std::cout << "站立" <<  std::endl;            
                break;
            case 's':
                msg.gait = 2;
                std::cout << "座下"<< std::endl;             
                break;
            case 'x':
                
                std::cout << "" << std::endl;
                break;
            
        }

		

		pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
