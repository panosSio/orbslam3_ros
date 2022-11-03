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
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include<opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

#include"../../../include/System.h"
#include"../../../include/Converter.h"
#include"../../../include/Atlas.h"
#include "../../../include/MapPoint.h"


using namespace std;

ros::Publisher pub_pose_current_frame_;

ros::Time current_frame_time_;
std::string map_frame_id_param_;
image_transport::Publisher rendered_image_publisher_;
nav_msgs::Path path;

class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM3::System* pSLAM,ros::NodeHandle& nh):mpSLAM(pSLAM){
        pub_pose_current_frame0_ = nh.advertise<nav_msgs::Odometry>("/tesse/odom",10);
        //pub_pose_current_frame1_ = nh.advertise<geometry_msgs::PoseStamped>("/orbslam3/odom",10);
        path_pub_ = nh.advertise<nav_msgs::Path>("/path", 10);
    }


    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void PublishRenderedImage (ORB_SLAM3::FrameDrawer * frame);

    ORB_SLAM3::System* mpSLAM;
    ros::Publisher pub_pose_current_frame0_;
    ros::Publisher pub_pose_current_frame1_;
    ros::Publisher path_pub_;
    tf2_ros::TransformBroadcaster br_;

    
    //ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}
   // ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);//false: no viewer

    ros::NodeHandle nodeHandler;

    ImageGrabber igb(&SLAM,nodeHandler);//,image);
    image_transport::ImageTransport debug_image(nodeHandler);
    rendered_image_publisher_ = debug_image.advertise ("/debug_image", 1);
    
    ros::Subscriber camera_sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);///camera/image_raw
    pub_pose_current_frame_ = nodeHandler.advertise<nav_msgs::Odometry>("/tesse/odom",10);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg_mono)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg_mono);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

     cv::Mat Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	// Check for successful track
	if (Tcw.rows == 4 && Tcw.cols == 4) 
	{

	   /* cv::Mat rot   = track.rowRange(0,3).colRange(0,3).t(); // Orientation
	    cv::Mat trans = -rot*track.rowRange(0,3).col(3);       // Position
	    vector<float> quat = ORB_SLAM3::Converter::toQuaternion(rot);

	    geometry_msgs::Pose pose_msgs;
	    pose_msgs.orientation.x = 1;//= {{1,2,3},{4,5,6,7}};
	    pose_msgs.orientation.y = 2;
	    pose_msgs.orientation.z = 3;
        
	    pose_pub.publish(pose_msgs);
           */

	    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
        vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);

        nav_msgs::Odometry msg;
        msg.header.frame_id = "world";
        //msg.header.stamp = tIm_ros;
        msg.header.stamp = msg_mono->header.stamp;
        msg.child_frame_id = "base_link_gt";
        msg.pose.pose.orientation.w = q[3];
        msg.pose.pose.orientation.x = q[0];
        msg.pose.pose.orientation.y = q[1];
        msg.pose.pose.orientation.z = q[2];
        msg.pose.pose.position.x = twc.ptr<float>(0)[0];
        msg.pose.pose.position.y = twc.ptr<float>(0)[1];
        msg.pose.pose.position.z = twc.ptr<float>(0)[2];
        pub_pose_current_frame_.publish(msg);


        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = "world";
        transform.child_frame_id = "base_link_gt";
        transform.header.stamp = msg.header.stamp;
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

        //Publish path
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

        PublishRenderedImage(mpSLAM->getMpFrameDrawer());


	    // trans.at<float>(0, 0), trans.at<float>(0, 1), trans.at<float>(0, 2); // (x, y, z)

	    // quat[0], quat[1], quat[2], quat[3] ; // (x, y, z, w)
	}

}


void ImageGrabber::PublishRenderedImage (ORB_SLAM3::FrameDrawer * frame) {
    
    cv::Mat im = frame->DrawFrame(true);
       
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", im).toImageMsg();
    rendered_image_publisher_.publish(rendered_image_msg);
}


