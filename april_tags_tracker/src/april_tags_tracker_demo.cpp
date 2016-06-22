#include <ros/ros.h>
#include "april_tags_tracker/april_tags_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "april_tags_tracker" );
  AprilTagsTracker tracker;  
  ros::spin();
  return 0;
}
