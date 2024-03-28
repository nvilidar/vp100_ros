#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <string.h>
#include "sensor_msgs/LaserScan.h"
#include "nvilidar_process.h"
#include "nvilidar_def.h"
#include "mytimer.hpp"

using namespace vp100_lidar;
#define ROSVerision "1.0.3"

//get stamp 
uint64_t  get_stamp_callback(){
    uint64_t current_time = 0;
    current_time = ros::Time::now().sec * 1e9 + ros::Time::now().nsec;
	return current_time;
}


int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "vp100_node"); 
    printf(" _   ___      _______ _      _____ _____          _____ \n");
    printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
    printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
    printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
    printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
    printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
    printf("\n");
    fflush(stdout);

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");

    Nvilidar_UserConfigTypeDef cfg;

    //sync para form rviz 
    nh_private.param<std::string>("serialport_name", cfg.serialport_name, "dev/nvilidar"); 
    nh_private.param<int>("serialport_baud", cfg.serialport_baud, 512000); 
    nh_private.param<std::string>("frame_id", cfg.frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", cfg.resolution_fixed, true);
    nh_private.param<bool>("auto_reconnect", cfg.auto_reconnect, false);
    nh_private.param<bool>("reversion", cfg.reversion, false);
    nh_private.param<bool>("inverted", cfg.inverted, false);
    nh_private.param<double>("angle_max", cfg.angle_max , 180.0);
    nh_private.param<double>("angle_min", cfg.angle_min , -180.0);
    nh_private.param<double>("range_max", cfg.range_max , 64.0);
    nh_private.param<double>("range_min", cfg.range_min , 0.001);
    nh_private.param<double>("aim_speed", cfg.aim_speed , 6.0);
    nh_private.param<int>("sampling_rate", cfg.sampling_rate, 3.0);
    nh_private.param<bool>("angle_offset_change_flag",cfg.angle_offset_change_flag,false);
    nh_private.param<double>("angle_offset",  cfg.angle_offset, 0.0);
    nh_private.param<std::string>("ignore_array_string",  cfg.ignore_array_string, "");
    nh_private.param<bool>("log_enable_flag", cfg.log_enable_flag,true);

    //choice use serialport or socket 
    vp100_lidar::LidarProcess laser(cfg.serialport_name,cfg.serialport_baud,get_stamp_callback,1e9);

    //reload lidar parameter 
    laser.LidarReloadPara(cfg);

    ROS_INFO("[NVILIDAR INFO] Now NVILIDAR ROS SDK VERSION:%s .......", ROSVerision);

    //lidar init
    bool ret = laser.LidarInitialialize();
    if (ret) 
    {
        //turn on the lidar 
        ret = laser.LidarTurnOn();
        if (!ret) 
        {
            ROS_ERROR("Failed to start Scan!!!");
        }
    } 
    else 
    {
        ROS_ERROR("Error initializing NVILIDAR Comms and Status!!!");
    }
    ros::Rate rate(50);
    LidarScan scan;

    while (ret && ros::ok()) 
    {
        if(laser.LidarSamplingProcess(scan))
        {
            if(scan.points.size() > 0)
            {
                sensor_msgs::LaserScan scan_msg;
                ros::Time start_scan_time;
                int avaliable_count = 0;
                start_scan_time.sec = scan.stamp_start/1000000000ul;
                start_scan_time.nsec = scan.stamp_start%1000000000ul;
                scan_msg.header.stamp = start_scan_time;
                scan_msg.header.frame_id = cfg.frame_id;
                scan_msg.angle_min =(scan.config.min_angle);
                scan_msg.angle_max = (scan.config.max_angle);
                scan_msg.angle_increment = (scan.config.angle_increment);
                scan_msg.scan_time = scan.config.scan_time;
                scan_msg.time_increment = scan.config.time_increment;
                scan_msg.range_min = (scan.config.min_range);
                scan_msg.range_max = (scan.config.max_range);
                int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
                
                scan_msg.ranges.clear();
                scan_msg.ranges.resize(size);
                scan_msg.intensities.clear();
                scan_msg.intensities.resize(size);
                avaliable_count = 0;
                for(int i=0; i < scan.points.size(); i++) {
                    int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
                    if((index >=0) && (index < size)) 
                    {
                        avaliable_count++;

                        scan_msg.ranges[index] = scan.points[i].range;
                        scan_msg.intensities[index] = scan.points[i].intensity;
                    }
                }
                if(cfg.resolution_fixed){   //fix counts  
                    if(size > avaliable_count){
                        for(int j = avaliable_count; j<size; j++){
                            scan_msg.ranges[j] = 0;
                            scan_msg.intensities[j] = 0;
                        }
                    }
                }
                scan_pub.publish(scan_msg);
            }
            else 
            {
                ROS_WARN("Lidar Data Invalid!");
            }
        }  
        else        //can't receive lidar data for 15 times,reconnect 
        {
            ROS_ERROR("Failed to get Lidar Data!");
            break;
        }  
        ros::spinOnce();
        rate.sleep();    
    }
    laser.LidarTurnOff();
    printf("[NVILIDAR INFO] Now NVILIDAR is stopping .......\n");
    laser.LidarCloseHandle();
    return 0;
}
