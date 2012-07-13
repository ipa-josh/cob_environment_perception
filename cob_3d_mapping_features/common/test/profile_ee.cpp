#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/edge_estimation_3d.h"

using namespace pcl;
typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PrecisionStopWatch t;
  std::string file_ = "/home/goa-sf/pcd_data/pc_0.pcd";
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

  IntegralImageNormalEstimation<PointXYZRGB,Normal> ne;
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setDepthDependentSmoothing(true);
  ne.setInputCloud(p);
  ne.compute(*n);

  t.precisionStart();
  OrganizedDataIndex<PointXYZRGB>::Ptr oTree (new OrganizedDataIndex<PointXYZRGB> );
  cob_3d_mapping_features::EdgeEstimation3D<PointXYZRGB, Normal, InterestPoint> ee;
  ee.setRadiusSearch(0.04);
  ee.setSearchMethod(oTree);
  ee.setInputCloud(p);
  ee.setInputNormals(n);
  ee.dist_threshold_ = 0.02;
  ee.compute(*ip);
  std::cout << t.precisionStop() << "s\t for 3D edge estimation" << std::endl;

  for (size_t i = 0; i < ip->points.size(); i++)
  {
    int col = (ip->points[i].strength) * 255;
    if (col > 255)
    {
      p->points[i].r = 0;
      p->points[i].g = 255;
      p->points[i].b = 0;
    }
    else if (col < 0)
    { 
      p->points[i].r = 255;
      p->points[i].g = 0;
      p->points[i].b = 0;
    }
    else
    {
      p->points[i].r = col;
      p->points[i].g = col;
      p->points[i].b = col;
    }
  }

  visualization::PCLVisualizer v("slow");
  ColorHdlRGB col_hdl(p);
  v.setBackgroundColor(0,127,127);
  v.addPointCloud<PointXYZRGB>(p,col_hdl, "segmented1");

  while(!v.wasStopped())
  {
    v.spinOnce(100);
    usleep(100000);
  }

  return 0;
}
