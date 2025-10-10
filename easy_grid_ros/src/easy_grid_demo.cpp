#include <easy_grid_ros/easy_grid_demo.hpp>


EasyGridDemo::EasyGridDemo() : rclcpp::Node("easy_grid_demo")
{
    //Load the params
    loadParams();
}


void EasyGridDemo::loadParams()
{
    this->declare_parameter<double>("resolution",0.1);
    this->declare_parameter<size_t>("map_width",200);
    this->declare_parameter<size_t>("map_height",200);
    this->declare_parameter("map_translation",std::vector<double>());
    this->declare_parameter("map_translation",std::vector<double>());
    this->declare_parameter("mode",0);
    this->declare_parameter("default_radius",2.0);

    this->get_parameter("resolution",grid_meta.resolution);
    this->get_parameter("map_width",grid_meta.map_width);
    this->get_parameter("map_height",grid_meta.map_height);
    std::vector<double> translation = this->get_parameter("map_translation").as_double_array();
    std::vector<double> rotation = this->get_parameter("map_rotation").as_double_array(); 
    this->get_parameter("mode",demo_mode_);
    this->get_parameter("default_radius",demo_rad_);
}