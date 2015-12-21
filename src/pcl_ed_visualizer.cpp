/* \author Geoffrey Biggs */
/* \author Ariel Hernandez */


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/console/parse.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>


using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace cv;


// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-help          this help\n"
            << "-pcd           Simple viewer with a pointcloud file\n"
			<< "-rgb           Simple viewer with a rgb file\n"
            << "-point         Simple viewer with a pointcloud file and a specific point P\n"
			<< "-plane         Simple viewer with a pointcloud, a point P and its plane C\n"
			<< "-edge          Simple viewer with the edgedetection over a pointcloud file (Not yet)\n"
			<< "-all           Simple viewer with the above characteristics (Not yet)\n"
            << "\n\n";
}

// --------------------------------------------
// -----Open 3D viewer and add point cloud-----
// --------------------------------------------
boost::shared_ptr<visualization::PCLVisualizer> simpleVisualisation (PointCloud<PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<PointXYZ> (cloud, "PointCloud sample");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloud sample");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


// --------------------------------------------
// -----Compute EdgeDetection------------------
// --------------------------------------------
boost::shared_ptr<visualization::PCLVisualizer> computeEdgeDetection (PointCloud<PointXYZ>::ConstPtr cloud, float th_dd, int max_search)
{
  boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("3D Viewer"));

  PointCloud<Normal>::Ptr normal (new PointCloud<Normal>);
  IntegralImageNormalEstimation<PointXYZ, Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize (10.0f);
  ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
  ne.setInputCloud (cloud);
  ne.compute (*normal);

  OrganizedEdgeFromNormals<PointXYZ, Normal, Label> oed;
  //OrganizedEdgeFromRGBNormals<PointXYZ, Normal, Label> oed;
  oed.setInputNormals (normal);
  oed.setInputCloud (cloud);
  oed.setDepthDisconThreshold (th_dd);
  oed.setMaxSearchNeighbors (max_search);
  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
  PointCloud<Label> labels;
  vector<PointIndices> label_indices;
  oed.compute (labels, label_indices);


  // Display edges in PCLVisualizer
  viewer->setSize (640, 480);
  viewer->addCoordinateSystem (0.2f, "global");
  viewer->addPointCloud (cloud, "original point cloud");

  PointCloud<PointXYZ>::Ptr occluding_edges (new PointCloud<PointXYZ>),
    occluded_edges (new PointCloud<PointXYZ>),
    nan_boundary_edges (new PointCloud<PointXYZ>),
    high_curvature_edges (new PointCloud<PointXYZ>),
    rgb_edges (new PointCloud<PointXYZ>);

  copyPointCloud (*cloud, label_indices[0].indices, *nan_boundary_edges);
  copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
  copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
  copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
  copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

  const int point_size = 2;
//  viewer->addPointCloud<PointXYZ> (nan_boundary_edges, "nan boundary edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "nan boundary edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "nan boundary edges");

  viewer->addPointCloud<PointXYZ> (occluding_edges, "occluding edges");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluding edges");

//  viewer->addPointCloud<PointXYZ> (occluded_edges, "occluded edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges");
//
//  viewer->addPointCloud<PointXYZ> (high_curvature_edges, "high curvature edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges");
//  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges");

  viewer->addPointCloud<PointXYZ> (rgb_edges, "rgb edges");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------

  if (find_argument (argc, argv, "-help") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool open_file(false),simple_vis(false), image_vis(false), edge_vis(false),point_vis(false),plane_vis(false),all(false);
  if (find_argument (argc, argv, "-pcd") >= 0)
  {
    simple_vis = true;
    open_file=true;
    cout << "Simple visualisation of point cloud file\n";
  }
  else if (find_argument (argc, argv, "-rgb") >= 0)
  {
    image_vis = true;
    cout << "Open a depth image\n";
  }
  else if (find_argument (argc, argv, "-point") >= 0)
  {
    point_vis = true;
    open_file=true;
    cout << "Simple visualisation: pointcloud + P\n";
  }
  else if (find_argument (argc, argv, "-plane") >= 0)
  {
	open_file=true;
	plane_vis = true;
    cout << "Simple visualisation: pointcloud + P + plane\n";
  }
  else if (find_argument (argc, argv, "-edge") >= 0)
  {
	edge_vis = true;
    cout << "EdgeDetection in the point cloud file (NOT YET)\n";
  }
  else if (find_argument (argc, argv, "-all") >= 0)
  {
    simple_vis = false;
    point_vis = true;
    plane_vis = true;
    edge_vis = true;
    cout << "All characteristics activated!\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Open a PCD file ---------------

  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr point (new PointCloud<PointXYZ>);

  if (open_file) //TODO: delete hard code, implements args
  {
    if (loadPCDFile<PointXYZ> ("/home/ahestevenz/cloud.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file cloud.pcd \n");
      return (-1);
    }
    if (loadPCDFile<PointXYZ> ("/home/ahestevenz/point.pcd", *point) == -1) //* load the file
    {
       PCL_ERROR ("Couldn't read file point.pcd \n");
       return (-1);
    }
    cout << "Data points loaded!" << endl;
  }

  // ------------------------------------
  // -----Open the image ---------------
  // ------------------------------------
//  Mat image;
//  if (image_vis) //TODO: delete hard code, implements args
//  {
//    image = imread("/home/ahestevenz/image.png", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file
//    image.convertTo(image, CV_32F); // convert the image data to float type
//  }

  // -----Build the plane ---------------
  // ------------------------------------

  ModelCoefficients plane_coeff;
  plane_coeff.values.resize(4);    // We need 4 values
  plane_coeff.values[0] = -0.314149; //TODO: delete hard code, implements args.
  plane_coeff.values[1] = 0.0466992;
  plane_coeff.values[2] = -0.948224;
  plane_coeff.values[3] = 1.11784;

  // ------------------------------------

  boost::shared_ptr<visualization::PCLVisualizer> viewer;
  if (simple_vis)
  {
	  viewer = simpleVisualisation(cloud);
  }
  else if (image_vis)
  {
//	  namedWindow( "Image", WINDOW_AUTOSIZE );// Create a window for display.
//	  imshow( "Image", image);
  }
  else if (point_vis)
  {
	  viewer = simpleVisualisation(cloud);
	  viewer->addPointCloud<PointXYZ> (point, "Point");
	  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Point");
	  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "Point");
  }
  else if (plane_vis)
  {
	  viewer = simpleVisualisation(cloud);
	  viewer = simpleVisualisation(cloud);
	  viewer->addPointCloud<PointXYZ> (point, "Point");
	  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Point");
	  viewer->setPointCloudRenderingProperties (visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "Point");
	  viewer->addPlane(plane_coeff, "plane",0);
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep(boost::posix_time::microseconds (100000));
  }
}
