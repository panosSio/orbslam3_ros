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
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"
#include"../../../include/Atlas.h"
#include "../../../include/MapPoint.h"


using namespace std;

 
 ros::Time current_frame_time_;
 std::string map_frame_id_param_;
 int min_observations_per_point_ = 2;
 image_transport::Publisher rendered_image_publisher_;
 nav_msgs::Path path;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::NodeHandle& nh):mpSLAM(pSLAM){
        pub_pose_current_frame0_ = nh.advertise<nav_msgs::Odometry>("/tesse/odom",10);
        pub_pose_current_frame1_ = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/odom",10);
        map_points_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/map_points", 1);
        //marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/path", 10);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    //void PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Publisher map_pub);
    void MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Publisher map_pub);
    void PublishRenderedImage (ORB_SLAM3::FrameDrawer * frame);

    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher pub_pose_current_frame0_;
    ros::Publisher pub_pose_current_frame1_;
    ros::Publisher map_points_publisher_;
    ros::Publisher path_pub_;
    //ros::Publisher marker_pub;
    
    tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char **argv)
{
        ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,false); 
    ros::NodeHandle nh;
    //image_transport::ImageTransport image;

    ImageGrabber igb(&SLAM,nh);//,image);
    image_transport::ImageTransport debug_image(nh);
    rendered_image_publisher_ = debug_image.advertise ("/debug_image", 1);

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

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/gray_image0", 1);//
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/gray_image1", 1);//
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

    cv::Mat Tcw;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if(Tcw.empty())
        return;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
    cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
    vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = msgLeft->header.stamp;
    msg.child_frame_id = "base_link_gt";
    msg.pose.pose.orientation.w = q[3];
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.position.x = twc.ptr<float>(0)[0];
    msg.pose.pose.position.y = twc.ptr<float>(0)[1];
    msg.pose.pose.position.z = twc.ptr<float>(0)[2];
    pub_pose_current_frame0_.publish(msg);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp = msg.header.stamp;
    p.pose.orientation.w = q[3];
    p.pose.orientation.x = q[0];
    p.pose.orientation.y = q[1];
    p.pose.orientation.z = q[2];
    p.pose.position.x = twc.ptr<float>(0)[0];
    p.pose.position.y = twc.ptr<float>(0)[1];
    p.pose.position.z = twc.ptr<float>(0)[2];
    pub_pose_current_frame1_.publish(p);

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link_gt";
    transform.header.stamp = msgLeft->header.stamp;
    transform.transform.rotation.x = msg.pose.pose.orientation.x;
    transform.transform.rotation.y = msg.pose.pose.orientation.z;
    transform.transform.rotation.z = -1.0*msg.pose.pose.orientation.y;
    transform.transform.rotation.w = msg.pose.pose.orientation.w;

    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.z;
    transform.transform.translation.z = -1.0*msg.pose.pose.position.y;
    br_.sendTransform(transform);

    current_frame_time_ = msg.header.stamp;
    map_frame_id_param_ = "world";
    
    //PublishMapPoints(mpSLAM->GetTrackedMapPoints(), map_points_publisher_);
    //MapPointsToPointCloud (mpSLAM->GetTrackedMapPoints(), map_points_publisher_);
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = msg.header.stamp;
    pose.pose.orientation.x = msg.pose.pose.orientation.x;
    pose.pose.orientation.y = msg.pose.pose.orientation.z;
    pose.pose.orientation.z = -1.0*msg.pose.pose.orientation.y;
    pose.pose.orientation.w = msg.pose.pose.orientation.w;
    pose.pose.position.x = msg.pose.pose.position.x;
    pose.pose.position.y = msg.pose.pose.position.z;
    pose.pose.position.z = -1.0*msg.pose.pose.position.y;

    path.header.frame_id = "world";
    path.header.stamp = msg.header.stamp;
    path.poses.push_back(pose);
    path_pub_.publish(path);

    //Publish image with points
    PublishRenderedImage(mpSLAM->getMpFrameDrawer());

}


 //void ImageGrabber::PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points,ros::Publisher map_points_publisher_ ) {
   //sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud (map_points);
   //map_points_publisher_.publish (cloud);
   
//}

void ImageGrabber::MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points,ros::Publisher map_points_publisher_ ) {
  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::PointCloud2 cloud;

  const int num_channels = 3; // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z"};
  for (int i = 0; i<num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }

   cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

  float data_array[num_channels];
  ORB_SLAM3::Atlas* mpAtlas = mpSLAM->getAtlas();
  const std::vector<ORB_SLAM3::MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
  const std::vector<ORB_SLAM3::MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();
  set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

  if(vpMPs.empty())
      return;

  for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
  {
      if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
         continue;
      //cv::Mat pos = vpMPs[i]->GetWorldPos();
      data_array[0] = vpMPs[i]->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
      data_array[1] = -1.0* vpMPs[i]->GetWorldPos().at<float>(0); //y. Do the transformation by just reading at the position of x instead of y
      data_array[2] = -1.0* vpMPs[i]->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
       //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
  }

  map_points_publisher_.publish (cloud);
  //return cloud;
}


void ImageGrabber::PublishRenderedImage (ORB_SLAM3::FrameDrawer * frame) {
    
    cv::Mat toShow;
    cv::Mat im = frame->DrawFrame(true);
    cv::Mat imRight = frame->DrawRightFrame();
    cv::hconcat(im,imRight,toShow);
       
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", toShow).toImageMsg();
    rendered_image_publisher_.publish(rendered_image_msg);
}




