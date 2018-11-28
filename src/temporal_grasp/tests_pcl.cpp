#include  <cstdlib>
#include <cmath>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/visualization/cloud_viewer.h>




int compute_principal_curvatures(int argc, char** argv){


    const char* cloud_path = std::getenv("CLOUD_PATH");
    if( !cloud_path ){
      printf("Set CLOUD_PATH environment variable to point to a .pcd file!\n");
      exit(1);
      return 1;
    }

    printf("Loading cloud %s\n", cloud_path);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_path, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file");
      return (-1);
    }

    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    // Compute the normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud (cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

    normalEstimation.setRadiusSearch (0.03);

    std::cout << " Computing normals..." << std::endl;
    normalEstimation.compute (*cloudWithNormals);

    pcl::PointCloud<pcl::Normal>::Ptr outputCloudNormals (new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloudPoints (new pcl::PointCloud<pcl::PointXYZ>());

    int idx = 0;
    for(auto p : cloudWithNormals->points){

        bool is_nan = false;
        for(int k = 0; k < 3; k++){
          is_nan = is_nan || std::isnan(p.normal[k]);
        }

        if(!is_nan){
          outputCloudNormals->points.push_back(p);
          outputCloudPoints->points.push_back(cloud->points[idx]);
        }
        idx++;
    }

    // cloudWithNormals = outputCloud;
    // for(auto p : outputCloudNormals->points){


    //   // if(! isnan(p.principal_curvature[0]) )
    //     std::cout << p << std::endl;
    // }


    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    // Provide the original point cloud (without normals)
    principalCurvaturesEstimation.setInputCloud (outputCloudPoints);

    // Provide the point cloud with normals
    principalCurvaturesEstimation.setInputNormals(outputCloudNormals);

    // Use the same KdTree from the normal estimation
    principalCurvaturesEstimation.setSearchMethod (tree);
    principalCurvaturesEstimation.setRadiusSearch(0.1);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());

    std::cout << " Computing curvatures..." << std::endl;
    principalCurvaturesEstimation.compute (*principalCurvatures);

    std::cout << "output points.size (): " << principalCurvatures->points.size () << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    // pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];

    for(auto p : principalCurvatures->points){

      // if(! isnan(p.principal_curvature[0]) )
        std::cout << p << std::endl;
    }

    return 0;
}

int compute_normals_pca(int argc, char** argv){

  const char* cloud_path = std::getenv("CLOUD_PATH");
    if( !cloud_path ){
      printf("Set CLOUD_PATH environment variable to point to a .pcd file!\n");
      exit(1);
      return 1;
    }

    printf("Loading cloud %s\n", cloud_path);


   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile (cloud_path, *cloud);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*


     // visualize normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }

  return 0;
}


int compute_normals_integral_image(int argc, char** argv){



    const char* cloud_path = std::getenv("CLOUD_PATH");
    if( !cloud_path ){
      printf("Set CLOUD_PATH environment variable to point to a .pcd file!\n");
      exit(1);
      return 1;
    }

    printf("Loading cloud %s\n", cloud_path);


   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::io::loadPCDFile (cloud_path, *cloud);

   // estimate normals
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

   pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
   ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
   ne.setMaxDepthChangeFactor(0.02f);
   ne.setNormalSmoothingSize(10.0f);
   ne.setInputCloud(cloud);
   ne.compute(*normals);

   // visualize normals
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   viewer.setBackgroundColor (0.0, 0.0, 0.5);
   viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

   while (!viewer.wasStopped ())
   {
     viewer.spinOnce ();
   }
   return 0;
}


int main(int argc, char** argv){


  return compute_principal_curvatures(argc, argv);
}