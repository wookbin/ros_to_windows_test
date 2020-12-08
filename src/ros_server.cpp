#define BUF_LEN 4096

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <math.h>
#include <unistd.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
using namespace std;
#include <pthread.h>

#include <sensor_msgs/Joy.h> //joystick msg
#include <sensor_msgs/JointState.h> //Joint msg

/*Pan&Tilt Joint state publisher************************/
//ros::Publisher joint_pub;
//Define the joint state
sensor_msgs::JointState joint_state;

ros::Publisher Doking_publisher; //docking publisher
int docking_flag = 0;

float m_fPan_Rad = 0.0;
float m_fTilt_Rad = 0.0;


FILE *fp;
double poseAMCLx, poseAMCLy, poseAMCLqx, poseAMCLqy, poseAMCLqz, poseAMCLqw;
geometry_msgs::Point cur_position;  // 지도에서의 x,y 위치 정보
geometry_msgs::Quaternion cur_quarter;  // 지도에서의 세타 정보

geometry_msgs::Twist twist_msg;
ros::Publisher ros_server_publisher; 

int cur_waypoint = 0; //현재 이동중인 waypoint
int tetra_battery;

char buffer[BUF_LEN];
char Send_buffer[BUF_LEN];
struct sockaddr_in server_addr, client_addr;
char temp[20];
int server_fd, client_fd;
//server_fd, client_fd : 각 소켓 번호
int len, msg_size;


///ADD_2019.06.07_wbjin/////////////////////////////////////////////////////////////////
//geometry_msgs::Point goal_positionX; //X위치
//geometry_msgs::Point goal_positionY; //Y위치
//geometry_msgs::Quaternion goal_quarter; // 세타
float goal_positionX;
float goal_positionY;
float goal_quarterX;
float goal_quarterY;
float goal_quarterZ;
float goal_quarterW;


void setGoal(move_base_msgs::MoveBaseActionGoal& goal)
{
  ros::Time now = ros::Time::now();

    goal.header.frame_id="map";
    goal.header.stamp=now;

    goal.goal_id.stamp = now;

    goal.goal.target_pose.header.stamp = now;
    goal.goal.target_pose.header.frame_id = "map";

    goal.goal.target_pose.pose.position.x = goal_positionX;
    goal.goal.target_pose.pose.position.y = goal_positionY;
    goal.goal.target_pose.pose.orientation.x = goal_quarterX;
    goal.goal.target_pose.pose.orientation.y = goal_quarterY;
    goal.goal.target_pose.pose.orientation.z = goal_quarterZ;
    goal.goal.target_pose.pose.orientation.w = goal_quarterW;
}

////////////////////////////////////////////////////////////////////////////////////////
void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msgResult)
{
  //uint8_t SUCCEEDED=3;
    int imsg_Result = 0;
    time_t curr_time;
    struct tm *curr_tm;
    curr_time = time(NULL);
    curr_tm = localtime(&curr_time);

    imsg_Result = msgResult->status.SUCCEEDED;
    switch(imsg_Result)
    {
        case 0: // The goal has yet to be processed by the action server

            break;
        case 1: // The goal is currently being processed by the action server

            break;
        case 2: //  The goal received a cancel request after it started executing and has since completed its execution

            break;
        case 3: // The goal was achieved successfully by the action server
            //sprintf(Send_buffer, "%d-%d, %d:%d:%d, %3d\n", curr_tm->tm_mon + 1,curr_tm->tm_mday, curr_tm->tm_hour, curr_tm->tm_min, curr_tm->tm_sec , tetra_battery);
            sprintf(Send_buffer, "Success Goal Point");
            write(client_fd, Send_buffer, sizeof(Send_buffer)); //echo return
            
            break;
        case 4: // The goal was aborted during execution by the action server due to some failure 

            break;
        case 5: // The goal was rejected by the action server without being processed,  because the goal was unattainable or invalid

            break;
        case 6: // The goal received a cancel request after it started executing and has not yet completed execution

            break;
        case 7: // The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled

            break;
        case 8: // The goal received a cancel request before it started executing and was successfully cancelled

            break;
        case 9: // An action client can determine that a goal is LOST. This should not be sent over the wire by an action server

            break;
        default:
                // Error...
                break;

    }
    
    memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
    printf(">>>>>imsg_Result = %d", imsg_Result);
    /*
    if(msgResult->status.SUCCEEDED == SUCCEEDED)
    {
            //printf("%d-%d, %d:%d:%d, %3d\n", curr_tm->tm_mon + 1,curr_tm->tm_mday, curr_tm->tm_hour, curr_tm->tm_min, curr_tm->tm_sec , tetra_battery);
            sprintf(Send_buffer, "%d-%d, %d:%d:%d, %3d\n", curr_tm->tm_mon + 1,curr_tm->tm_mday, curr_tm->tm_hour, curr_tm->tm_min, curr_tm->tm_sec , tetra_battery);
            write(client_fd, Send_buffer, sizeof(Send_buffer)); //echo return
            memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
    }
    */
   
}

void betteryCallback(const std_msgs::Int32::ConstPtr& msg)
{
  tetra_battery = msg->data;
}

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLqx = msgAMCL->pose.pose.orientation.x;
    poseAMCLqy = msgAMCL->pose.pose.orientation.y;
    poseAMCLqz = msgAMCL->pose.pose.orientation.z;
    poseAMCLqw = msgAMCL->pose.pose.orientation.w;

    cur_position = msgAMCL->pose.pose.position;
    cur_quarter = msgAMCL->pose.pose.orientation;
    //printf("poseAMCLCallback call.....");
}

int main(int argc, char* argv[])
{

    ros::init(argc,argv, "ros_server");

    ros::NodeHandle pnh;
    ros_server_publisher = pnh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 50);
    ros::NodeHandle cmdpnh;
    ros::Publisher cmdpub = cmdpnh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::NodeHandle n;
    ros::Subscriber sub_amcl = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10 ,poseAMCLCallback);
    ros::Subscriber result_sub = n.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, resultCallback);
    ros::Subscriber tetra_btry = n.subscribe<std_msgs::Int32>("tetra_battery", 10, betteryCallback);
 
    /*
    //Decleare a joint state publisher
    ros::NodeHandle Jpub_h;
    joint_pub = Jpub_h.advertise<sensor_msgs::JointState>("joint_states",1);
    joint_state.name.push_back("joint1"); //Pan_Joint
    joint_state.name.push_back("joint2"); //Tilt_Joint
    unsigned int nsize = joint_state.name.size();
    joint_state.position.resize(nsize);
    joint_state.velocity.resize(nsize);
    joint_state.effort.resize(nsize);
    */
    
    if(argc != 2)
    {
        printf("usage : %s [port]\n", argv[0]);
        exit(0);
    }
 
    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {// 소켓 생성
        printf("Server : Can't open stream socket\n");
        exit(0);
    }
    
    int option = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
    
    memset(&server_addr, 0x00, sizeof(server_addr));
    //server_Addr 을 NULL로 초기화
 
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(atoi(argv[1]));
    //server_addr 셋팅
 
    if(bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <0)
    {//bind() 호출
        printf("Server : Can't bind local address.\n");
        exit(0);
    }
 
    if(listen(server_fd, 5) < 0)
    {//소켓을 수동 대기모드로 설정
        printf("Server : Can't listening connect.\n");
        exit(0);
    }
 
    memset(buffer, 0x00, sizeof(buffer)); //Send_buffer
    printf("Server : wating connection request.\n");
    len = sizeof(client_addr);

    client_fd = accept(server_fd, (sockaddr *)&client_addr, (socklen_t*)&len);
    if(client_fd < 0)
    {
        printf("Server: accept failed.\n");
        exit(0);
    }

    inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, temp, sizeof(temp));
    printf("Server : %s client connected.\n", temp);

    ///////////////////////////////////////////////////////////////////////////
    pid_t pid1; // pid2, pid3, pid4, pid5, pid6;
    int status;
    char Textbuffer[BUF_LEN];
    char strPath[BUF_LEN];
    
    ///////////////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(10); // Hz

    while(ros::ok())
    { 
        msg_size = read(client_fd, buffer, BUF_LEN);

        if(strcmp(buffer, "exit") == 0)
        {
            printf("[EXIT] = %s \n",buffer);
            memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
            close(server_fd);
            close(client_fd);
        }
        else if(buffer[0] == 'G' && buffer[1] == 'e') //else if(strcmp(buffer, "Getdata") == 0)
        {
            //Get Data Current Pose
            sprintf(Send_buffer, "D=%lf,%lf,%lf,%lf,%lf,%lf,%3d\n", poseAMCLx, poseAMCLy, poseAMCLqx, poseAMCLqy, poseAMCLqz, poseAMCLqw, tetra_battery);
            write(client_fd, Send_buffer, sizeof(Send_buffer)); //echo return
            //memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer
            //memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
        }
        else if(buffer[0] == 'D')
        {
            docking_flag = 0;

            sprintf(Send_buffer, "B=%3d \n", tetra_battery);
            write(client_fd, Send_buffer, sizeof(Send_buffer)); //echo return
            //printf("msg = %s \n",buffer);

             ////','??? ??//
            char* ptr = strtok(buffer, ",");
            int icnt = 0;

           while(ptr != NULL)
            {   
                //printf("%s\n",ptr);
                ptr = strtok(NULL, ",");
                icnt++;
                switch(icnt)
                {
                    case 1:
                        if(ptr != NULL)
                        {
                            printf("1: %s\n", ptr);
                            //msg.linear.x = linx;     msg.angular.z = angZ;
                            twist_msg.linear.x = (atof(ptr) / 10.0);
                        }
                        break;
                    case 2:
                        printf("2: %s\n", ptr);
                        twist_msg.angular.z = (atof(ptr) / 10.0);
                        break;
                    case 3:
                        printf("3: %s\n", ptr);
                        //Button...
                        int m_iMode = atoi(ptr);

                        switch(m_iMode)
                        {
                            case 0:

                                break;
                            case 1: 
                                
                                break;
                            case 2: 
                                
                                break;
                            case 4: 
                                
                                break;
                            case 8: 
                                
                                break;
                            case 16:

                                break;
                            case 32:

                                break;
                            case 64:
                                twist_msg.linear.x = 0.0;
                                twist_msg.angular.z = 0.0;
                                break;
                            case 128:
                                
                                break;
                        }
                        break;
                    
                }
            }

            //publish
            cmdpub.publish(twist_msg);

        }
        else if(strcmp(buffer, "HOME") == 0)
        {
            docking_flag = 0;

            fp = fopen("/home/hyulim/homepoint.txt", "r");  // ??: /home/??/~~~~~   
            if(fp != NULL)
            {

                while(!feof(fp)) //?? ??? ? ??//
                {
                    fgets(Textbuffer, sizeof(Textbuffer), fp);
                    //printf("%s", Textbuffer);
                    
                    ////','??? X, Y, Theta??//
                    char* ptr = strtok(Textbuffer, ",");
                    int icnt = 0;

                    while(ptr != NULL)
                    {   
                        //printf("%s\n",ptr);
                        ptr = strtok(NULL, ",");
                        icnt++;

                        switch(icnt)
                        {
                            case 1:
                                if(ptr != NULL)
                                {
                                    //printf("X: %s\n", ptr);
                                    goal_positionX = atof(ptr);
                                }
                                break;
                            case 2:
                                //printf("Y: %s\n", ptr);
                                goal_positionY = atof(ptr);
                                break;
                            case 3:
                                //printf("Qx: %s\n", ptr);
                                goal_quarterX = atof(ptr);
                                break;
                            case 4:
                                //printf("Qy: %s\n", ptr);
                                goal_quarterY = atof(ptr);
                                break;
                            case 5:
                                //printf("Qz: %s\n", ptr);
                                goal_quarterZ = atof(ptr);    
                                break;
                            case 6:
                                //printf("Qw: %s\n", ptr);
                                goal_quarterW = atof(ptr);
                                break;
                        }
                    }

                }

                fclose(fp);

                //go to HOME
                move_base_msgs::MoveBaseActionGoal goal;
                setGoal(goal);
                ros_server_publisher.publish(goal);

            }   
            else
            {
                printf("goalpoint file?? ??.");
            }
        }
        else if(buffer[0] == 'G' && buffer[1] == 'O')
        {
            docking_flag = 0;
            
            //printf("GO??? ??....");
            sprintf(strPath, "/home/hyulim/goalpoint%c.txt",buffer[2]);


            fp = fopen(strPath, "r");  // ??: /home/??/~~~~~   
            if(fp != NULL)
            {
                while(!feof(fp)) //?? ??? ? ??//
                {
                    fgets(Textbuffer, sizeof(Textbuffer), fp);

                     ////','??? X, Y, Theta??//
                    char* ptr = strtok(Textbuffer, ",");
                    int icnt = 0;

                    while(ptr != NULL)
                    {   
                        //printf("%s\n",ptr);
                        ptr = strtok(NULL, ",");
                        icnt++;

                        switch(icnt)
                        {
                            case 1:
                                if(ptr != NULL)
                                {
                                    //printf("X: %s\n", ptr);
                                    goal_positionX = atof(ptr);
                                }
                                break;
                            case 2:
                                //printf("Y: %s\n", ptr);
                                goal_positionY = atof(ptr);
                                break;
                            case 3:
                               //printf("Qx: %s\n", ptr);
                                goal_quarterX = atof(ptr);
                                break;
                            case 4:
                                //printf("Qy: %s\n", ptr);
                                goal_quarterY = atof(ptr);
                                break;
                            case 5:
                                //printf("Qz: %s\n", ptr);
                                goal_quarterZ = atof(ptr);    
                                break;
                            case 6:
                                //printf("Qw: %s\n", ptr);
                                goal_quarterW = atof(ptr);
                                break;
                        }
                    }
                }

                fclose(fp);
                //go to Goal 1 Position
                move_base_msgs::MoveBaseActionGoal goal;
                setGoal(goal);
                ros_server_publisher.publish(goal);

            }               
            else
            {
                printf("goalpoint file?? ??.");
            }
        }
        else
        {
            pid1 = fork();
            if(pid1 == 0)
            {
                write(client_fd, buffer, msg_size); //echo return
                //printf("msg = %s \n",buffer);
                memset(&buffer, 0x00, sizeof(buffer)); //clear buffer

                system(buffer); //launch call...

                //printf("launch run...\n");
                //printf("pid1 = %d",pid1);
                exit(0);

            }

        }
        
        memset(&buffer, 0x00, sizeof(buffer)); //clear buffer
        memset(&Send_buffer, 0x00, sizeof(Send_buffer)); //clear Send_buffer

        ////Docking////
        Doking_publisher = pnh.advertise<std_msgs::Int32>("Docking",10);
        std_msgs::Int32 docking_start;
        docking_start.data = docking_flag;
        Doking_publisher.publish(docking_start);
        //////////
        
        ros::spinOnce();  
        loop_rate.sleep();
    }

    close(server_fd);
    close(client_fd);
    
    printf("---server_fd Close--.\n");
    return 0;
}
