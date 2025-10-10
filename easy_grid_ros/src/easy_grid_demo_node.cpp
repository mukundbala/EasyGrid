#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <easy_grid_ros/easy_grid_demo.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<EasyGridDemo>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();    
}