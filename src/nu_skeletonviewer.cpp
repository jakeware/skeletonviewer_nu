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
// NU includes
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <nu_skeletonmsgs/Skeletons.h>

using pcl::PointCloud;
using pcl::PointXYZ;


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------

// Global data
sensor_msgs::PointCloud2ConstPtr cloud_, cloud_old_;
nu_skeletonmsgs::SkeletonsConstPtr skels_;
boost::mutex m;


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
    cloudsub_ = nh_.subscribe("input", 30, &Update::cloudcb, this);
    skelsub_ = nh_.subscribe("/skeletons", 1, &Update::skelcb, this);

    boost::thread visualization_thread (&Update::viewercb, this);
  }

  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &cloud){
    m.lock ();
    cloud_ = cloud;
    m.unlock ();
  }
  
  void skelcb(const nu_skeletonmsgs::SkeletonsConstPtr &skels){
    m.lock ();
    skels_ = skels;
    m.unlock();
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
    p.addCoordinateSystem (0.1);

    while (true) {
      d.sleep ();
      // If no cloud received yet, continue
      if (!cloud_)
	continue;
      
      p.spinOnce (1);

      if (cloud_old_ == cloud_)
	continue;
    
      m.lock ();
    
      // Convert to PointCloud<T>
      if (pcl::getFieldIndex (*cloud_, "rgb") != -1) {
	rgb = true;
	pcl::fromROSMsg (*cloud_, cloud_xyz_rgb);

      // // add wireframe to viewer
      // p.addPolygon (cloud_xyz_rgb, 1.0, 0.0, 0.0, "polygon", 0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
  
      // p.addLine<PointXYZ, PointXYZ> (cloud_xyz_rgb.points[0], cloud_xyz_rgb.points[1], 0.0, 1.0, 0.0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

      // p.addSphere<PointXYZ> (cloud_xyz_rgb.points[0], 1, 0.0, 1.0, 0.0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere");
      }
      else {
	rgb = false;
	pcl::fromROSMsg (*cloud_, cloud_xyz);
	pcl::getFields (cloud_xyz, fields);


      // // add wireframe to viewer
      // p.addPolygon (cloud_xyz, 1.0, 0.0, 0.0, "polygon", 0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "polygon");
  
      // p.addLine<PointXYZ, PointXYZ> (cloud_xyz.points[0], cloud_xyz.points[1], 0.0, 1.0, 0.0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 50, "line");

      // p.addSphere<PointXYZ> (cloud_xyz.points[0], 1, 0.0, 1.0, 0.0);
      // p.setShapeRenderingProperties (pcl_visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "sphere");
      }
      cloud_old_ = cloud_;
      m.unlock ();

      p.removePointCloud ("cloud");
    
      // If no RGB data present, use a simpler white handler
      if (rgb && pcl::getFieldIndex (cloud_xyz_rgb, "rgb", fields) != -1) {
	pcl_visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler (cloud_xyz_rgb);
	p.addPointCloud (cloud_xyz_rgb, color_handler, "cloud");
      }
      else {
	pcl_visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler (cloud_xyz, 255, 0, 255);
	p.addPointCloud (cloud_xyz, color_handler, "cloud");
      }
    }
  }
};


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main (int argc, char** argv)
{
  ros::init (argc, argv, "openni_viewer", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh ("~");

  Update update;

  signal (SIGINT, sigIntHandler);

  // Spin
  ros::spin ();

  printf("Check1");

  // Join, delete, exit
  //visualization_thread.join ();
  return (0);
}
