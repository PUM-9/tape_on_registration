#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;

typedef float millimeter;

/**
 * This functions creates a cuboid shaped pointcloud based on 3 parameters height, width and depth. these are modified
 * so that the cloud is placed on the right coordinates in a 3D space.
 * @return
 */
int main() {
    
    millimeter depth = 60.0;
    millimeter height = 30.0;
    millimeter width = 30.0;
    millimeter modifier = 400; // used to modify the position of the pointcloud
    millimeter step_size = 0.5;
    Cloud cube = Cloud();

    // Create two xy planes in the xzy space.
    for (millimeter i=(0+modifier); i<(width+modifier); i+=step_size) {

        for (millimeter j=(0+modifier); j<(height+modifier); j+=step_size) {
            //Add one point for each side of the cube
            Point point1 = Point(i, j, -(0+modifier)); //places a point at negative z value to get cloud in right place
            Point point2 = Point(i, j, -(depth+modifier)); //places a point at negative z value to get cloud in right place
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    // Create two yz planes in the xyz space.
    for (millimeter i=(0+modifier); i<(depth+modifier); i+=step_size) {

        for (millimeter j=(0+modifier); j<(height+modifier); j+=step_size) {
            Point point1 = Point((0+modifier), j, -i);//places a point at negative z value to get cloud in right place
            Point point2 = Point((width+modifier), j, -i);//places a point at negative z value to get cloud in right place
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    // Create two xz planes in the xyz space.
    for (millimeter i=(0+modifier); i<(depth+modifier); i+=step_size) {

        for (millimeter j=(0+modifier); j<(width+modifier); j+=step_size) {
            Point point1 = Point(j, (0+modifier), -i);//places a point at negative z value to get cloud in right place
            Point point2 = Point(j, (height+modifier), -i);//places a point at negative z value to get cloud in right place
            cube.push_back(point1);
            cube.push_back(point2);
        }

    }

    pcl::io::savePCDFileASCII("test.pcd", cube);

    return 0;
}