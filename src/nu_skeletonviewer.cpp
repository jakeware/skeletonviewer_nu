// nu_skeletonviewer.cpp
// Jake Ware and Jarvis Schultz
// Winter 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

// ROS core
#include <signal.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
// PCL includes
#include <pcl/point_types.h>
#include <pcl_visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
// NU includes
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <skeletonmsgs_nu/Skeletons.h>

using pcl::PointCloud;
using pcl::PointXYZ;


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

// Global data
sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
skeletonmsgs_nu::Skeletons skels_;
boost::mutex m;
bool gotskel = false;


//---------------------------------------------------------------------------
// Functions and Classes
//---------------------------------------------------------------------------

void sigIntHandler (int sig) {
  exit (0);
}


class Update {

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloudsub_,skelsub_;

public:

  Update() {
    cloudsub_ = nh_.subscribe("/camera/depth/points", 30, &Update::cloudcb, this);
    skelsub_ = nh_.subscribe("/skeletons", 1, &Update::skelcb, this);

    boost::thread visualization_thread (&Update::viewercb, this);
  }

  pcl::PointXYZ getjoint(const skeletonmsgs_nu::SkeletonJoint &skeljoint) {
    pcl::PointXYZ joint_position;
      
    joint_position.x = skeljoint.transform.translation.x;
    joint_position.y = skeljoint.transform.translation.y;
    joint_position.z = skeljoint.transform.translation.z;

    return joint_position;
  }

  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &cloud) {
    m.lock ();
    cloud_ = cloud;
    m.unlock ();
  }
  
  void skelcb(const skeletonmsgs_nu::Skeletons skels) {
    m.lock ();
    skels_ = skels;
    m.unlock();

    gotskel = true;
  }

  void viewercb() {
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_xyz_rgb;

    ros::Duration d (0.01);
    bool rgb = false;
    std::vector<sensor_msgs::PointField> fields;
  
    // Create the visualizer
    pcl_visualization::PCLVisualizer p ("Northwestern Skeleton Tracker Viewer");

    // Add a coordinate system to screen
    p.setBackgroundColor (0, 0, 0);
    p.addCoordinateSystem (0.1);

    while (true) {
      d.sleep ();
      // If no cloud received yet, continue
      if (!cloud_)
	continue;
      
      p.spinOnce (1);

      if (cloud_old_ == cloud_)
	continue;
    
      // m.lock ();
    
      // // Convert to PointCloud<T>
      // if (pcl::getFieldIndex (*cloud_, "rgb") != -1) {
      // 	rgb = true;
      // 	pcl::fromROSMsg (*cloud_, cloud_xyz_rgb);
      // }
      // else {
      // 	rgb = false;
      // 	pcl::fromROSMsg (*cloud_, cloud_xyz);
      // 	pcl::getFields (cloud_xyz, fields);
      // }
      // cloud_old_ = cloud_;
      // m.unlock ();

      // p.removePointCloud ("cloud");
      p.removeShape ("head");
      p.removeShape ("right shoulder");
      p.removeShape ("left shoulder");
      p.removeShape ("right elbow");
      p.removeShape ("left elbow");
      p.removeShape ("right hand");
      p.removeShape ("left hand");
      p.removeShape ("torso RS");
      p.removeShape ("torso LS");
      p.removeShape ("torso RH");
      p.removeShape ("torso LH");
      p.removeShape ("hips");
      p.removeShape ("right knee");
      p.removeShape ("left knee");
      p.removeShape ("right foot");
      p.removeShape ("left foot");
            
      // If no RGB data present, use a simpler white handler
      if(false){
      // if (rgb && pcl::getFieldIndex (cloud_xyz_rgb, "rgb", fields) != -1) {
	pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler (cloud_xyz_rgb);
	p.addPointCloud (cloud_xyz_rgb, color_handler, "cloud");

	// add wireframe to viewer if data is being trasmitted
	if (gotskel) {
	  // add head
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].head), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "head");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "head");	  	  

	  // add right shoulder
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_shoulder), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "right shoulder");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right shoulder");	  

	  // add left shoulder
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_shoulder), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "left shoulder");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left shoulder");	  

	  // add right elbow
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_elbow), getjoint(skels_.skeletons[0].right_shoulder), 0.0, 1.0, 0.0, "right elbow");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right elbow");	  

	  // add left elbow
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_elbow), getjoint(skels_.skeletons[0].left_shoulder), 0.0, 1.0, 0.0, "left elbow");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left elbow");	  

	  // add right hand
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_hand), getjoint(skels_.skeletons[0].right_elbow), 0.0, 1.0, 0.0, "right hand");	
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right hand");  

	  // add left hand
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_hand), getjoint(skels_.skeletons[0].left_elbow), 0.0, 1.0, 0.0, "left hand");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left hand");	  

	  // add torso
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].right_shoulder), 0.0, 1.0, 0.0, "torso RS");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso RS");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].left_shoulder), 0.0, 1.0, 0.0, "torso LS");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso LS");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].right_hip), 0.0, 1.0, 0.0, "torso RH");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso RH");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "torso LH");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso LH");	  

	  // add hips
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_hip), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "hips");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "hips");

	  // add right knee
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_knee), getjoint(skels_.skeletons[0].right_hip), 0.0, 1.0, 0.0, "right knee");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right knee");

	  // add left knee
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_knee), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "left knee");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left knee");

	  // add right foot
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_foot), getjoint(skels_.skeletons[0].right_knee), 0.0, 1.0, 0.0, "right foot");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right foot");

	  // add left foot
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_foot), getjoint(skels_.skeletons[0].left_knee), 0.0, 1.0, 0.0, "left foot");	  
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left foot");
	  	  
	  gotskel = false;
	}
      }
      else {
	// pcl_visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler (cloud_xyz, 255, 0, 255);
	// p.addPointCloud (cloud_xyz, color_handler, "cloud");

	// add wireframe to viewer if data is being trasmitted
	if (gotskel) {
	  // add head
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].head), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "head");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "head");	  	  

	  // add right shoulder
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_shoulder), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "right shoulder");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right shoulder");	  

	  // add left shoulder
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_shoulder), getjoint(skels_.skeletons[0].neck), 0.0, 1.0, 0.0, "left shoulder");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left shoulder");	  

	  // add right elbow
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_elbow), getjoint(skels_.skeletons[0].right_shoulder), 0.0, 1.0, 0.0, "right elbow");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right elbow");	  

	  // add left elbow
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_elbow), getjoint(skels_.skeletons[0].left_shoulder), 0.0, 1.0, 0.0, "left elbow");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left elbow");	  

	  // add right hand
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_hand), getjoint(skels_.skeletons[0].right_elbow), 0.0, 1.0, 0.0, "right hand");	
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right hand");  

	  // add left hand
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_hand), getjoint(skels_.skeletons[0].left_elbow), 0.0, 1.0, 0.0, "left hand");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left hand");	  

	  // add torso
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].right_shoulder), 0.0, 1.0, 0.0, "torso RS");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso RS");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].left_shoulder), 0.0, 1.0, 0.0, "torso LS");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso LS");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].right_hip), 0.0, 1.0, 0.0, "torso RH");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso RH");
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].torso), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "torso LH");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "torso LH");	  

	  // add hips
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_hip), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "hips");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "hips");

	  // add right knee
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_knee), getjoint(skels_.skeletons[0].right_hip), 0.0, 1.0, 0.0, "right knee");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right knee");

	  // add left knee
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_knee), getjoint(skels_.skeletons[0].left_hip), 0.0, 1.0, 0.0, "left knee");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left knee");

	  // add right foot
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].right_foot), getjoint(skels_.skeletons[0].right_knee), 0.0, 1.0, 0.0, "right foot");
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "right foot");

	  // add left foot
	  p.addLine<PointXYZ, PointXYZ> (getjoint(skels_.skeletons[0].left_foot), getjoint(skels_.skeletons[0].left_knee), 0.0, 1.0, 0.0, "left foot");	  
	  p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "left foot");
	  	  
	  gotskel = false;
	}
      }
    }
  }
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main (int argc, char** argv)
{
  ros::init (argc, argv, "nu_skeletonviewer", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh ("~");

  signal (SIGINT, sigIntHandler);

  printf("Check1\n");

  Update update;

  // Spin
  ros::spin ();

  printf("Check3\n");

  // Join, delete, exit
  // visualization_thread.join ();
  return (0);
}
