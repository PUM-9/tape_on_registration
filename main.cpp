#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef float millimeter;

int main() {
    
    millimeter depth = 460.0;
    millimeter height = 430.0;
    millimeter width = 430.0;
    millimeter step_size = 0.5;
    Cloud cube = Cloud();

    // Create two xy planes in the xzy space.
    for (millimeter i=400; i<width; i+=step_size) {

        for (millimeter j=400; j<height; j+=step_size) {
            //Add one point for each side of the cube
            Point point1 = Point(i, j, -400);
            Point point2 = Point(i, j, -depth);
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    // Create two yz planes in the xyz space.
    for (millimeter i=400; i<depth; i+=step_size) {

        for (millimeter j=400; j<height; j+=step_size) {
            Point point1 = Point(400, j, -i);
            Point point2 = Point(width, j, -i);
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    // Create two xz planes in the xyz space.
    for (millimeter i=400; i<depth; i+=step_size) {

        for (millimeter j=400; j<width; j+=step_size) {
            Point point1 = Point(j, 400, -i);
            Point point2 = Point(j, height, -i);
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    pcl::io::savePCDFileASCII("test.pcd", cube);

    return 0;
}