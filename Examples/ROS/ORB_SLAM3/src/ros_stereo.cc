/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "gnss_tools.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FILE* groundTruth  = fopen( "/home/wws/ORB_SLAM3/groundTruthorb.csv", "w+");
using namespace std;
void span_bp_callback();

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void img0_callback(const sensor_msgs::ImageConstPtr &img_msg);
    void img1_callback(const sensor_msgs::ImageConstPtr &img_msg);
    
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

image_transport::Publisher imagePub;
image_transport::Publisher imagePubR;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    std_msgs::Header h;
    try
    {
    cv_ptrLeft = cv_bridge::toCvShare(img_msg); 
    sensor_msgs::Image imgRec;
    h=img_msg->header;

    cv::Mat img;
    int width_ = 0;
    int height_ = 0;
    
    /*re-compute the image size,because the part of vehicle itself appear in image*/
    width_ = cv_ptrLeft->image.size().width;  
    height_ = cv_ptrLeft->image.size().height;
  
    //resize
    cv::resize(cv_ptrLeft->image, img, cv::Size(width_,height_));
    
    // set the car head area color black, rows:i cols:j, because resize may lead to intrinsic parameters change
    int n_row = img.rows;
    int n_col = img.cols*img.channels();
    for (int i=296; i<n_row; i++)
    {
    for(int j =0; j<n_col; j++)
    {
        img.at<uchar>(i,j) = 0;
    }

    }

    cv_bridge::CvImage resizeimg;
    resizeimg.encoding = "bgr8";
    resizeimg.header = h;
    resizeimg.image = img;
    
    sensor_msgs::ImagePtr imagePtr=resizeimg.toImageMsg();
    imagePtr->width = width_;
    imagePtr->height = height_;
    
    imgRec = *imagePtr;

    imagePub.publish(imgRec);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptrRight;
    std_msgs::Header h;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(img_msg); 
        sensor_msgs::Image imgRecR;
        h = img_msg->header;

        cv::Mat img;
        int width_ = 0;
        int height_ = 0;
        
        /*re-compute the image size,because the part of vehicle itself appear in image*/
        width_ = cv_ptrRight->image.size().width;  
        height_ = cv_ptrRight->image.size().height;
        
        //resize
        cv::resize(cv_ptrRight->image, img, cv::Size(width_,height_));

        // img = img(cv::Range(0,296), cv::Range(0,672));
        // set the car head area color black, rows:i cols:j, because resize may lead to intrinsic parameters change
        int n_row = img.rows;
        int n_col = img.cols*img.channels();
        for (int i=296; i<n_row; i++)
        {
            for(int j =0; j<n_col; j++)
            {
                img.at<uchar>(i,j) = 0;
            }

        }

        cv_bridge::CvImage resizeimg;
        resizeimg.encoding = "bgr8";
        resizeimg.header = h;
        resizeimg.image = img; // 
        
        sensor_msgs::ImagePtr imagePtr=resizeimg.toImageMsg();
        imagePtr->width = width_;
        imagePtr->height = height_;
        
        imgRecR = *imagePtr;

        imagePubR.publish(imgRecR);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

/*for GPSweek to unix time*/
double GPSWeek2UnixTime(int gps_week, int gps_seconds)
{
   double SECONDS_PER_WEEK = 60*60*24*7*1000;
   double gps_unix_offset = 315964800000;
   double lipsecond = 18000; //units:ms
   double milliseconds = gps_week*SECONDS_PER_WEEK + gps_seconds - lipsecond + gps_unix_offset;
   return milliseconds;
}

Eigen::MatrixXd original1;  //referene point in llh 
bool flag=0; // get the reference point or not?
nav_msgs::Odometry spanOdom;
GNSS_Tools gnss_tools_1;

double original_heading = 0;
double span_heading = 0;
double vins_heading = 0;
double seconds_;
ros::Publisher pub_odometrySpan;
tf2::Quaternion gt_q;
double  time_now=0;

void span_bp_callback(const novatel_msgs::INSPVAXConstPtr& fix_msg)
{
    std::cout << std::setprecision(12);

    double gt_current_stamp = GPSWeek2UnixTime(fix_msg->header.gps_week, fix_msg->header.gps_week_seconds);
    gt_current_stamp = gt_current_stamp/1000; 

    std_msgs::Header header;
    Eigen::MatrixXd laloal;
    laloal.resize(3,1);
    laloal(0) = fix_msg->longitude;
    laloal(1) = fix_msg->latitude;
    laloal(2) = fix_msg->altitude;

    Eigen::MatrixXd ecef;
    ecef.resize(3,1);
    ecef=gnss_tools_1.llh2ecef(laloal);

    original1.resize(3,1);
    if(flag==0) // the initial value
    {
      flag=1;
      original1(0)=laloal(0); 
      original1(1)=laloal(1);
      original1(2)=laloal(2);
      original_heading = fix_msg->azimuth;
    }

    span_heading = fix_msg->azimuth;

    /* get the ENU solution from SPAN-CPT */
    Eigen::MatrixXd enu1;
    enu1.resize(3,1);
    enu1= gnss_tools_1.ecef2enu(original1, ecef);
    
    /* Assume the vehicle is on the plane at the first epoch*/
    double prex_ = enu1(0);
    double prey_ = enu1(1);
    double theta = (original_heading )*( 3.141592 / 180.0 ); //
    enu1(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
    enu1(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 
    
    /* publish the odometry from span-cpt */
    spanOdom.header.frame_id = "world";
    spanOdom.child_frame_id = "world";
    spanOdom.pose.pose.position.x = enu1(0);
    spanOdom.pose.pose.position.y = enu1(1);
    spanOdom.pose.pose.position.z = 0;
    pub_odometrySpan.publish(spanOdom);

    #if 1
    double gps_w = fix_msg->header.gps_week;
    double gps_s = fix_msg->header.gps_week_seconds;
    double lat = fix_msg->latitude;
    double lon = fix_msg->longitude;
    double alt = fix_msg->altitude;
    #endif

    /* transform the euclidean to quanternion*/
    double roll = fix_msg->roll;
    double pitch = fix_msg->pitch;
    double yaw = -(fix_msg->azimuth - original_heading); // align the GPS and vio coordinate

    // tf2::Quaternion gt_q;
    gt_q.setRPY(roll * 3.1415926/180, pitch * 3.1415926/180, yaw * 3.1415926/180);
    gt_q.normalize();
    time_now = time_now+1;
  
   time_now = time_now+1;
    /* save the groundTruth using fprintf*/
    fprintf(groundTruth, "%3.2f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f\n",gt_current_stamp, enu1(0), enu1(1), enu1(2), gt_q[0], gt_q[1], gt_q[2], gt_q[3]);
    fflush(groundTruth);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    
    /*by xiwei 20220618*/
    ros::Subscriber sub_img0 = nh.subscribe("/zed2/camera/left/image_raw", 1, img0_callback);
    ros::Subscriber sub_img1 = nh.subscribe("/zed2/camera/right/image_raw", 1, img1_callback);

    image_transport::ImageTransport it(nh);
    imagePub = it.advertise("imgRec",1);
    imagePubR = it.advertise("imgRecR",1);
   
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "imgRec", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "imgRecR", 1);
    ros::Subscriber span_BP_sub =nh.subscribe("/novatel_data/inspvax", 500, span_bp_callback);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;

        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        
        mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());

    }
    else
    {
        // mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        
    }

}