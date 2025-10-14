#pragma once

#include <rclcpp/rclcpp.hpp>
#include <easy_grid_core/easy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <string>
#include <mutex>

class EasyGridDemo : public rclcpp::Node
{

public:
    struct OccupancyCell
    {
        uint8_t occupancy {0}; // 0 is free, 1 is occupied
    };

    EasyGridDemo();

private:

    size_t demo_mode_ {0};
    double demo_rad_ {1.0};
    size_t demo_vertices_ {5};
    size_t demo_submap_width_ {5};
    size_t demo_submap_height_ {5};
    std::mutex keypress_mut_;

    //Polygon
    std::vector<Eigen::Vector3d> ray_start_end;
    std::vector<Eigen::Vector3d> vertices;

    // Occupancy Grid
    easy_grid::GridHandler<OccupancyCell> occupancy_grid;
    easy_grid::MetaData grid_meta;

    // Timer
    rclcpp::TimerBase::SharedPtr keypress_timer_;
    rclcpp::CallbackGroup::SharedPtr keypress_cbg_;
    
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr user_pt_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pt_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

    std::vector<std::string> MODE_LUT_ = {"RAY ITERATOR DEMO","SUBMAP ITERATOR DEMO",
                                         "CIRCLE PERIMETER ITERATOR DEMO","CIRCLE AREA ITERATOR DEMO",
                                         "POLYGON PERIMETER ITERATOR DEMO","POLYGON AREA ITERATOR DEMO"};

    //Callbacks
    void KeyPressTimerCallback();
    void ClickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr ptmsg);

    // Get keypress
    std::string getKey();
    // Process the key press and populate the correct variable
    void processKey(std::string &key);

    // Load parameters from the config
    void loadParams();
    // Initialise the grid
    void initGrid();
    //Convert To Polygon Msg
    geometry_msgs::msg::PolygonStamped toPolygonMsg(std::vector<Eigen::Vector3d> &points);
    //Convert To Visualization Msgs Points
    visualization_msgs::msg::MarkerArray toPtMsg(std::vector<Eigen::Vector3d> &points);
    //Convert to OccupancyGrid Msg
    nav_msgs::msg::OccupancyGrid toOccupancyGridMsg();

};