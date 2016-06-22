#ifndef APRIL_TAGS_TRACKER_H_
#define APRIL_TAGS_TRACKER_H_

#include <utility>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

struct Pos2D{
  int x;
  int y;
};

class AprilTagsTracker {
public:
  AprilTagsTracker( AprilTags::TagCodes codes = AprilTags::tagCodes36h11 );
  virtual ~AprilTagsTracker();

  void imageCallback( const sensor_msgs::ImageConstPtr& msg);
  std::vector<AprilTags::TagDetection> extractTags( cv::Mat& image);
 
  ros::NodeHandle                 m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber     m_sub;

  ros::Publisher                  m_pos_pub;
  ros::Publisher                  m_t_pos_pub;

  AprilTags::TagDetector* mp_tag_detector;
  AprilTags::TagCodes     m_tag_codes;

};

#endif // APRIL_TAGS_TRACKER_H_
