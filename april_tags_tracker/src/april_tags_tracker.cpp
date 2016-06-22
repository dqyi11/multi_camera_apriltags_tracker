#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Pose2D.h>
#include "april_tags_tracker/april_tags_tracker.h"
#include "april_tags_tracker/april_tag_pos.h"

using namespace std;
using namespace cv;

#define APRIL_TAGS_TRACKER_VIEW "April Tags Tracker"
#define APRIL_TAG_POS_MSG_NAME  "april_tag_pos"
#define CAMERA_ID               1

#define PI 3.1415926
#define REACH_THRESHOLD 30

float convRadius(float radius) {
  if( radius < 0 ) {
    radius = 2*PI + radius;
  }
  return radius;
}

void AprilTagsTracker::imageCallback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
  vector<AprilTags::TagDetection> tags = extractTags( cv_ptr->image ); 
  if( tags.size() > 0) {
    april_tags_tracker::april_tag_pos msg;
    for( unsigned int i=0; i<tags.size(); i++ ){
      AprilTags::TagDetection tag = tags[i];
      circle( cv_ptr->image, Point2f( tag.cxy.first, tag.cxy.second ), 2, Scalar(0,255,0), 4 ); 
      line( cv_ptr->image, Point( tag.p[0].first, tag.p[0].second ), Point( tag.p[1].first, tag.p[1].second ), Scalar(0,255,0), 2 );
      line( cv_ptr->image, Point( tag.p[1].first, tag.p[1].second ), Point( tag.p[2].first, tag.p[2].second ), Scalar(0,255,0), 2 );
      line( cv_ptr->image, Point( tag.p[2].first, tag.p[2].second ), Point( tag.p[3].first, tag.p[3].second ), Scalar(0,255,0), 2 );
      line( cv_ptr->image, Point( tag.p[3].first, tag.p[3].second ), Point( tag.p[0].first, tag.p[0].second ), Scalar(0,255,0), 2 );
      double orientation_length = sqrt( pow(tag.p[0].first-tag.p[1].first,2) + pow(tag.p[0].second-tag.p[1].second,2) );
      line( cv_ptr->image, Point( tag.cxy.first, tag.cxy.second ), Point( tag.cxy.first+orientation_length*cos(tag.getXYOrientation()), tag.cxy.second+orientation_length*sin(tag.getXYOrientation()) ), Scalar(0,255,0), 2 );

      
      msg.id.push_back( tag.id );
      geometry_msgs::Pose2D pose;
      pose.x = tag.cxy.first;
      pose.y = tag.cxy.second;
      pose.theta = convRadius( tag.getXYOrientation() );
      msg.pose.push_back( pose );     

    }
    m_pos_pub.publish(msg); 
  }

  //std::cout << "paint " << std::endl;
  cv::imshow(APRIL_TAGS_TRACKER_VIEW, cv_ptr->image );
  /*
  int key_value = cv::waitKey(30);
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }*/
  if( false == ros::ok() ) {
    ros::shutdown();
  }
}

AprilTagsTracker::AprilTagsTracker( AprilTags::TagCodes codes  ) : m_it( m_nh ) , m_tag_codes( codes )  {

  cv::namedWindow(APRIL_TAGS_TRACKER_VIEW);
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/camera/image_raw", 1, &AprilTagsTracker::imageCallback, this);
  mp_tag_detector = new AprilTags::TagDetector( m_tag_codes ); 

  m_pos_pub = m_nh.advertise<april_tags_tracker::april_tag_pos>( APRIL_TAG_POS_MSG_NAME, 1000 );

}

AprilTagsTracker::~AprilTagsTracker() {
  if( mp_tag_detector ) {
    delete mp_tag_detector;
    mp_tag_detector = NULL;
  }
  cv::destroyWindow( APRIL_TAGS_TRACKER_VIEW );
}
  
std::vector<AprilTags::TagDetection> AprilTagsTracker::extractTags( cv::Mat& image) {

   cv::Mat gray_img;
   cvtColor( image, gray_img, CV_BGR2GRAY );
   return mp_tag_detector->extractTags( gray_img );
}
  
 

