#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

#include "cob_3d_mapping_common/stop_watch.h"
#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_mapping_features/fast_edge_estimation_3d_omp.h"
#include "cob_3d_mapping_features/organized_normal_estimation_omp.h"

using namespace pcl;
typedef visualization::PointCloudColorHandlerRGBField<PointXYZRGB> ColorHdlRGB;

int main(int argc, char** argv)
{
  PointCloud<PointXYZRGB>::Ptr p(new PointCloud<PointXYZRGB>);
  PointCloud<Normal>::Ptr n(new PointCloud<Normal>);
  PointCloud<InterestPoint>::Ptr ip(new PointCloud<InterestPoint>);
  PrecisionStopWatch t;
  std::string file_ = argv[1];
  PCDReader r;
  if (r.read(file_, *p) == -1) return(0);

//  IntegralImageNormalEstimation<PointXYZRGB,Normal> ne;
//  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
//  ne.setMaxDepthChangeFactor(0.02f);
//  ne.setNormalSmoothingSize(10.0f);
//  ne.setDepthDependentSmoothing(true);
//  ne.setInputCloud(p);
//  ne.compute(*n);
  t.precisionStart();
  cob_3d_mapping_features::OrganizedNormalEstimationOMP<PointXYZRGB,Normal,PointLabel> ne;
  ne.setPixelSearchRadius(8,2,2);
  ne.setInputCloud(p);
  PointCloud<PointLabel>::Ptr labels(new PointCloud<PointLabel>);
  ne.setOutputLabels(labels);
  ne.compute(*n);
  std::cout << t.precisionStop() << "s\t for normal estimation" << std::endl;

  t.precisionStart();
  cob_3d_mapping_features::FastEdgeEstimation3DOMP<PointXYZRGB, Normal, InterestPoint> ee3d;
  ee3d.setPixelSearchRadius(16,2,4);
  ee3d.setInputCloud(p);
  ee3d.setInputNormals(n);
  ee3d.compute(*ip);
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
    else if (col > 100)
    {
      p->points[i].r = 0;
      p->points[i].g = 0;
      p->points[i].b = 255;
    }
    else
    {
      p->points[i].r = col;
      p->points[i].g = col;
      p->points[i].b = col;
    }
  }

  visualization::PCLVisualizer v;
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
