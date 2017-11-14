#ifndef TSDF_INTEGRATION_H
#define TSDF_INTEGRATION_H

#include <string>

#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

int tsdf_integrate(std::string in_dir, std::string out_dir);

class TSDFIntegration
{
public:
    TSDFIntegration();
    int process(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, Eigen::Affine3d pose);
    pcl::PolygonMesh::Ptr generateMesh();

    bool visualize, verbose, flatten, cleanup, invert, organized, world_frame, zero_nans, save_ascii, binary_poses, cloud_only, integrate_color;
    float cloud_units, pose_units, max_sensor_dist, min_sensor_dist, min_weight, trunc_dist_pos, trunc_dist_neg;
    int num_random_splits, tsdf_res;
    pcl::console::TicToc tt;
    cpu_tsdf::TSDFVolumeOctree::Ptr tsdf;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr aggregate;
    Eigen::Affine3d origPose;
    bool firstFrame;
};

#endif // TSDF_INTEGRATION_H
