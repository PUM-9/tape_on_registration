#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <map>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef float millimeter;

Cloud generate_cuboid(const millimeter width, const millimeter height, const millimeter depth, Cloud &cuboid) {

    millimeter step_size = 0.1;

    // Create two xy planes in the xzy space.
    for (millimeter i=0; i<width; i+=step_size) {

        for (millimeter j=0; j<height; j+=step_size) {
            //Add one point for each side of the cube
            Point point1 = Point(i, j, 0);
            Point point2 = Point(i, j, depth);
            cuboid.push_back(point1);
            cuboid.push_back(point2);
        }

    }

    // Create two yz planes in the xyz space.
    for (millimeter i=0; i<depth; i+=step_size) {

        for (millimeter j=0; j<height; j+=step_size) {
            Point point1 = Point(0, j, i);
            Point point2 = Point(width, j, i);
            cuboid.push_back(point1);
            cuboid.push_back(point2);
        }

    }

    // Create two xz planes in the xyz space.
    for (millimeter i=0; i<depth; i+=step_size) {

        for (millimeter j=0; j<width; j+=step_size) {
            Point point1 = Point(j, 0, i);
            Point point2 = Point(j, height, i);
            cuboid.push_back(point1);
            cuboid.push_back(point2);
        }

    }
}

bool replace_origo(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y + current_point.z;
    float new_sum = new_point.x + new_point.y + new_point.z;
    return new_sum < current_sum;
}

bool replace_x(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y - current_point.z;
    float new_sum = new_point.x - new_point.y - new_point.z;
    return new_sum > current_sum;
}

bool replace_y(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y - current_point.x - current_point.z;
    float new_sum = new_point.y - new_point.x - new_point.z;
    return new_sum > current_sum;
}

bool replace_z(const Point current_point, const Point &new_point) {
    float current_sum = current_point.z - current_point.y - current_point.x;
    float new_sum = new_point.z - new_point.y - new_point.x;
    return new_sum > current_sum;
}

bool replace_xyz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y + current_point.z;
    float new_sum = new_point.x + new_point.y + new_point.z;
    return new_sum > current_sum;
}

bool replace_yz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y + current_point.z - current_point.x;
    float new_sum = new_point.y + new_point.z - new_point.x;
    return new_sum > current_sum;
}

bool replace_xy(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y - current_point.z;
    float new_sum = new_point.x + new_point.y - new_point.z;
    return new_sum > current_sum;
}

bool replace_xz(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y + current_point.z;
    float new_sum = new_point.x - new_point.y + new_point.z;
    return new_sum > current_sum;
}

bool replace_rect_origo(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y;
    float new_sum = new_point.x + new_point.y;
    return new_sum < current_sum;
}

bool replace_rect_x(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x - current_point.y;
    float new_sum = new_point.x - new_point.y;
    return new_sum > current_sum;
}

bool replace_rect_y(const Point current_point, const Point &new_point) {
    float current_sum = current_point.y - current_point.x;
    float new_sum = new_point.y - new_point.x;
    return new_sum > current_sum;
}

bool replace_rect_xy(const Point current_point, const Point &new_point) {
    float current_sum = current_point.x + current_point.y;
    float new_sum = new_point.x + new_point.y;
    return new_sum > current_sum;
}

std::map<const std::string, Point> find_cuboid_corners(const Cloud &pointCloud) {
    std::map<const std::string, Point> corners;
    corners.insert(std::pair<const std::string, Point>(std::string("origo"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("x"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("y"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("z"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("xy"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("yz"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("xz"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("xyz"), pointCloud.points[0]));
    for (int i = 1; i < pointCloud.size(); i++) {

        Point current_point = pointCloud.points[i];

        if (replace_origo(corners["origo"], current_point)) {
            corners["origo"] = current_point;
        }

        if (replace_x(corners["x"], current_point)) {
            corners["x"] = current_point;
        }

        if (replace_y(corners["y"], current_point)) {
            corners["y"] = current_point;
        }

        if (replace_z(corners["z"], current_point)) {
            corners["z"] = current_point;
        }

        if (replace_xy(corners["xy"], current_point)) {
            corners["xy"] = current_point;
        }

        if (replace_xz(corners["xz"], current_point)) {
            corners["xz"] = current_point;
        }

        if (replace_yz(corners["yz"], current_point)) {
            corners["yz"] = current_point;
        }

        if (replace_xyz(corners["xyz"], current_point)) {
            corners["xyz"] = current_point;
        }

    }

    return corners;
}

std::map<const std::string, Point> find_rectangle_corners(const Cloud &pointCloud) {
    std::map<const std::string, Point> corners;
    corners.insert(std::pair<const std::string, Point>(std::string("origo"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("x"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("y"), pointCloud.points[0]));
    corners.insert(std::pair<const std::string, Point>(std::string("xy"), pointCloud.points[0]));

    for (int i = 1; i < pointCloud.size(); i++){

        Point current_point = pointCloud.points[i];

        if (replace_rect_origo(corners["origo"], current_point)) {
            corners["origo"] = current_point;
        }

        if (replace_rect_x(corners["x"], current_point)) {
            corners["x"] = current_point;
        }

        if (replace_rect_y(corners["y"], current_point)) {
            corners["y"] = current_point;
        }

        if (replace_rect_xy(corners["xy"], current_point)) {
            corners["xy"] = current_point;
        }
    }
    return corners;
}

int main() {

    Cloud pointCloud = Cloud();

    pcl::io::loadPCDFile("cube/cube0000.pcd", pointCloud);

    std::map<const std::string, Point> corners = find_rectangle_corners(pointCloud);

    cout << "origo: " << corners["origo"] << std::endl;
    cout << "x: " << corners["x"] << std::endl;
    cout << "y: " << corners["y"] << std::endl;
//    cout << "z: " << corners["z"] << std::endl;
    cout << "xy: " << corners["xy"] << std::endl;
 //   cout << "yz: " << corners["yz"] << std::endl;
 //   cout << "xz: " << corners["xz"] << std::endl;
 //   cout << "xyz" << corners["xyz"] << std::endl;

    return 0;
}