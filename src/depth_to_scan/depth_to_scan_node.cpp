


/*
 * depth_to_scan.cpp
 *
 *  Created on: Jun 12, 2022
 *      Author: yakir huri
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */
#include <iostream>
#include <limits>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>


#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <angles/angles.h>
#include <opencv2/opencv.hpp>

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>


using namespace std;
using namespace cv;
using namespace pcl;

class Depth2Scan{


public:

    Depth2Scan(){

        ros::NodeHandle nodePrivate("~"); 
         //rosparam
        nodePrivate.param("max_distance_meter", maxDistMeters_, 2.0);
        nodePrivate.param("min_distance_meter", minDistMeters_, 0.1);        
        nodePrivate.param("min_h", minHeight_, -0.5);  
        nodePrivate.param("max_h", maxHeight_, 1.0);

        nodePrivate.param("min_deg_angle", minDegAngle_, -90.0);
        nodePrivate.param("max_deg_angle", maxDegAngle_, 90.0);

        nodePrivate.param("base_frmae", base_frame_, string("base_link")); 
   


        depth_image_sub = node_.subscribe("/camera/depth/image_rect_raw", 1,
             &Depth2Scan::depthCallback, this);

        info_sub = node_.subscribe("/camera/depth/camera_info", 1,
             &Depth2Scan::cameraInfoCallback, this);

        scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/scan_from_depth", 50);

        pubPclLaser_ = node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >
		 ("/yakir_cloud", 1, true); 
    }

    ~Depth2Scan(){}

private:

    

    geometry_msgs::PointStamped transformToByFrames(
        Point3d objectPoint3d, string base_Frame, string child_Frame)  {


        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = child_Frame;
        pointStampedIn.header.stamp = ros::Time(0);
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;

        if( !initTransform_){

            try{
                

                if  (tfListener_.waitForTransform(base_Frame, child_Frame, 
                    ros::Time(0), ros::Duration(0.005))){
                        
                    initTransform_ = true;

                    tfListener_.lookupTransform(base_Frame, child_Frame,  
                                        ros::Time(0), transform_);

                    tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);

                    return pointStampedOut;
                    
                }else {

                   //cerr<<"Failed to find transform between "<<base_Frame<<" and "<<child_Frame<<endl;
                   return pointStampedOut;
                }

            }   
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }
        else {


            tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);


            return pointStampedOut;
        }  

      
    }



    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info) {

        if (cameraInfoInited_ == false)
        {
            if (pinholeCameraModel_.fromCameraInfo(*cam_info))
            {
                cameraInfoInited_ = true;

                cx_ = pinholeCameraModel_.cx();
                cy_ = pinholeCameraModel_.cy();

                fx_ = pinholeCameraModel_.fx();
                fy_ = pinholeCameraModel_.fy();

            }
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &image) {

        if (cameraInfoInited_) {           
          
          
            std::map<int, float > deg_dist_map;

            string depth_camera_link_ = image->header.frame_id;


            cv::Mat depthColor;
            cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat depth = cvBridgePtr->image;

          

            float minAngleRad = 9999;
            float maxAngleRad = -9999;

            // loop over dpeth img
            for(int j =0; j <depth.rows ; j++){

                float* pixel = depth.ptr<float>(j);  
                
                for(int i = 0; i< depth.cols ; i++){

                    float distValMM = pixel[i]; 
                    float distM = distValMM / 1000.0;


                    if (!std::isfinite(pixel[i]) || distValMM == 0) {
                        continue;
                    }


                    // point to far or too close
                    if ( distM > maxDistMeters_
                         || distM < minDistMeters_ ){
                        continue;
                    }
                  
                    geometry_msgs::PointStamped pose;
                    if(!extractDepthFromBboxObject( cv::Point2d(i, j), distValMM, pose)){
                        continue;
                    }

                    // if too high or too low
                    if(  pose.point.z < minHeight_ || pose.point.z > maxHeight_) {
                        continue;
                    }              
                                      
    
                    int angleDeg  = int(angles::to_degrees(atan2(pose.point.y, pose.point.x)));

                    if( angleDeg < minDegAngle_ || angleDeg > maxDegAngle_){

                        continue;
                    }

                    // get the min deg
                    if ( angles::from_degrees(angleDeg) < minAngleRad   ){
                        minAngleRad = angles::from_degrees(angleDeg);
                    }

                    // get the max deg
                    if ( angles::from_degrees(angleDeg) > maxAngleRad   ){
                        maxAngleRad = angles::from_degrees(angleDeg);
                    }

                    if( deg_dist_map.find(angleDeg) != deg_dist_map.end()){
                        //angle exsist
                        
                        // update te vaule (distnace)
                        if(  distM < deg_dist_map[angleDeg] ) {

                            // cerr<<" map angle "<<angleDeg<<" old distance "<<deg_dist_map.at(angleDeg)<<endl;
                            // cerr<<"new distnace "<<distM<<endl;
                            deg_dist_map.at(angleDeg) = distM;
                            // cerr<<" after update: map angle "<<angleDeg<<" new distance "<<deg_dist_map.at(angleDeg)<<endl;
                            // cerr<<"---------------------------------"<<endl;
                        }

                    } else {
                        //create new pair
                        deg_dist_map[angleDeg] = distM;

                    }

                }
            }           


            // //  publish cloud
            // publishPointCloud(deg_dist_map);


            //  publish scan
            publishScan(deg_dist_map);          

            
           

        }
            
    }

    void publishPointCloud(const std::map<int, float >& deg_dist_map){

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = base_frame_;

        //publish point cloud
            
        for (auto it = deg_dist_map.begin(); it != deg_dist_map.end(); ++it)
        {
            float closestDistM = it->second;
            float radAngle = angles::from_degrees(it->first);


            pcl::PointXYZRGB pRGB;                


            pRGB.x =   (closestDistM) * cos(radAngle);
            pRGB.y =   (closestDistM) * sin(radAngle);          
            pRGB.z =    0;                   
            

            //cerr<<" closestDistM "<<closestDistM<<" angle "<<it->first<<" x "<<pRGB.x<<" y "<<pRGB.y<<endl;

            uint8_t r = 0;
            uint8_t g = 255;
            uint8_t b = 255;

            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pRGB.rgb = *reinterpret_cast<float*>(&rgb);

            cloud.points.push_back(pRGB); 

        }

        pubPclLaser_.publish(cloud); 

    }

    void publishScan(const std::map<int, float >& deg_dist_map){


        unsigned int num_readings = 360;
        double laser_frequency = 40;
        double ranges[num_readings];
        double intensities[num_readings];

        ros::Time scan_time = ros::Time::now();


        float angle = angles::from_degrees(num_readings);
        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "base_link";
        scan.angle_min = angles::from_degrees(-(depth_angle / 2));
        scan.angle_max = angles::from_degrees( depth_angle / 2);
        scan.angle_increment = angle / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        // scan.ranges[0] = 5;  //-90 deg
        // scan.ranges[num_readings-1] = 2; // +90 deg
        // scan.ranges[num_readings /2 ] = 4; // 0 deg
        

        for (auto it = deg_dist_map.begin(); it != deg_dist_map.end(); ++it)
        {   
            int deg =  it->first;
            

            if( deg < 0){

                int cellNum = deg + (depth_angle /2);
                scan.ranges[cellNum] =  it->second;
                //scan.intensities[cellNum] = 100;
                // cerr<<" cellNum "<<cellNum<<" deg "<<deg<<" distance "<<it->second<<endl;


            } 
            else if( deg >= 0 && deg < (depth_angle /2)) {
                scan.ranges[deg + (depth_angle /2)] =  it->second;
            }
            

        }  

        
        scan_pub_.publish(scan);


    }
    
   

    bool extractDepthFromBboxObject( cv::Point2d pix, float d,
           geometry_msgs::PointStamped& pose) {
       

        if( isinf(d) || d == 0){
            return false;
        }

        float dM = d / 1000;
        cv::Point3d p = cv::Point3d(((pix.x - cx_) * dM / fx_), ((pix.y - cy_) * dM / fy_), dM);

        pose = transformToByFrames(p, base_frame_ , source_frame );
           

        return true;
    }


private:

    ros::Publisher scan_pub_;

    ros::Publisher pubPclLaser_;

    ros::Subscriber depth_image_sub;

    ros::Subscriber  info_sub;

    ros::NodeHandle node_;

    bool cameraInfoInited_ = false;

    tf::TransformListener tfListener_;


    image_geometry::PinholeCameraModel pinholeCameraModel_;


    string base_frame_= "base_link";

    string source_frame = "camera_depth_optical_frame";


    double maxDistMeters_ = 2.0;
    double minDistMeters_ = 0.0;

    double minHeight_ = -0.5;
    double maxHeight_ = 2.5;

    int depth_angle = 180;

    double minDegAngle_ = -90.0;
    double maxDegAngle_ = 90.0;

    float cx_ = 0;
    float cy_ = 0;
    float fx_ = 0;
    float fy_ = 0;

    bool initTransform_ = false;
    tf::StampedTransform transform_;



};


int main(int argc, char **argv){


    ros::init(argc, argv,"depth_to_scan");
    Depth2Scan depth2Scan;

    ros::spin();
    return 0;
}
