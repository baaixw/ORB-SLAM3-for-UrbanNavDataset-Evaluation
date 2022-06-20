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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>


#include <image_transport/image_transport.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

#include "gnss_tools.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FILE* groundTruth  = fopen( "/home/wws/ORB_SLAM3/groundTruthorb.csv", "w+");

using namespace std;

void span_bp_callback();
class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}
    
    void imgLeft_callback(const sensor_msgs::ImageConstPtr &img_msg);

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> img0Buf;
    std::mutex mBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

image_transport::Publisher imagePubLeft;

void imgLeft_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    std_msgs::Header h;
    try
    {
    cv_ptrLeft = cv_bridge::toCvShare(img_msg); 
    sensor_msgs::Image imgRecLeft;
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
    
    imgRecLeft = *imagePtr;

    imagePubLeft.publish(imgRecLeft);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
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

double imu_time = -1;

void span_bp_callback(const novatel_msgs::INSPVAXConstPtr& fix_msg)
{
    std::cout << std::setprecision(12);
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
    fprintf(groundTruth, "%3.2f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f %6.8f\n",imu_time, enu1(0), enu1(1), enu1(2), gt_q[0], gt_q[1], gt_q[2], gt_q[3]);
    fflush(groundTruth);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mono_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }


  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM,&imugb,bEqual); // TODO
  
  /*by xiwei 20220618*/
  ros::Subscriber sub_imgLeft = n.subscribe("/zed2/camera/left/image_raw", 100, imgLeft_callback);
  
  image_transport::ImageTransport it(n);
  imagePubLeft = it.advertise("imgRecLeft",100);

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu = n.subscribe("/imu/data", 1000, &ImuGrabber::GrabImu, &imugb); 
  ros::Subscriber sub_img0 = n.subscribe("imgRecLeft", 100, &ImageGrabber::GrabImage,&igb);

  ros::Subscriber span_BP_sub =n.subscribe("/novatel_data/inspvax", 500, span_bp_callback);
  pub_odometrySpan = n.advertise<nav_msgs::Odometry>("odometryenu", 1000);

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
  ros::spin();


// Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    // SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    // ros::shutdown();
    
  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.toSec();
      if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;
      {
      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());

      /*crop the image,because the part of vehicle itself appear in image*/
      // cv::Mat crop = im(cv::Rect(0,0,672,296)).clone();
      // im = crop;

      img0Buf.pop();
      this->mBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tIm)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

      mpSLAM->TrackMonocular(im,tIm,vImuMeas);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  imu_time = imu_msg->header.stamp.toSec();
  mBufMutex.unlock();
  return;
}


