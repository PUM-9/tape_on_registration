#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointCloud<pcl::PointXYZ> cloud;
typedef cloud::Ptr cloudPtr;

int main() {






    // Code snippet below doesnt work.
    /*
    cloudPtr cube (new Cloud());
    int side = 10 + 1;
    cube->points.resize(pow (side, 3));
    cube->width = cube ->points.size();
    cube->height = 1;
    int p = 0;
    for (size_t i = 0; i < side; i++){
        for (size_t j = 0; j < side; j++){
            for (size_t k = 0; k < side; k++){

                cube->points[p].getVector3fMap()=Eigen::Vector3f(i,j,k);
                p++;
            }
        }

    }
    pcl::io::savePCDFileASCII("test.pcd", *cube);
    */

    return 0;
}