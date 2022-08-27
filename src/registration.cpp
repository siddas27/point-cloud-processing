#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int
 main ()
{
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(5,1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/lo1.pcd", *cloud_in) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/map.pcd", *map) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
  // Fill in the CloudIn data
//   for (auto& point : *cloud_in)
//   {
//     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
//   }
  
  std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
  Eigen::Matrix4d tf_matrix;

tf_matrix << 0.22464277,  0.97303472,  0.05233596, -1,
-0.95752283,  0.23038866, -0.1734102,2,
-0.18079176, -0.01115753, 0.98345811,5,
0,0,0,1;

   pcl::transformPointCloud (*cloud_in, *cloud_in, tf_matrix);
//   for (auto& point : *cloud_in)
//     std::cout << point << std::endl;
      
//   *map = *cloud_in;
  pcl::io::savePCDFileASCII ("../data/tf1.pcd", *cloud_in);
  std::cout << "size:" << map->size() << std::endl;
//   for (auto& point : *map)
//     point.x += 0.7f;

  std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
//   for (auto& point : *map)
//     std::cout << point << std::endl;



//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(cloud_in);
//   icp.setInputTarget(map);
//   icp.setMaximumIterations (100);
//   pcl::PointCloud<pcl::PointXYZ> Final;
//   icp.align(Final);

//   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//   icp.getFitnessScore() << std::endl;
//   std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Matrix4f transform_2;
//   transform_2 = icp.getFinalTransformation();
    transform_2 <<   0.999721,  -0.0102929,  -0.0214961,    0.141907,
  0.0103274,    0.999948,   0.0013943,     0.21722,
  0.0214802, -0.00161466,    0.999766,    0.252701,
          0,           0,           0,          1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_in, *transformed_cloud, transform_2);
  pcl::io::savePCDFileASCII ("../data/final3_pcd.pcd", *transformed_cloud);

  tf_matrix *= transform_2;

  std::cout<< tf_matrix <<std::endl;

  return (0);
}