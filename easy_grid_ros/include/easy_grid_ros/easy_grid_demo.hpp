#include <rclcpp/rclcpp.hpp>
#include <easy_grid_core/easy_grid.hpp>

class EasyGridDemo : public rclcpp::Node
{

public:
    struct OccupancyCell
    {
        uint8_t occupancy; // 0 is free, 1 is occupied
    };

    EasyGridDemo();

private:

    size_t demo_mode_ {0};
    double demo_rad_ {1.0};

    // Occupancy Grid
    easy_grid::GridHandler<OccupancyCell> occupancy_grid;
    easy_grid::MetaData grid_meta;
    
    // Load parameters from the config
    void loadParams();
    // Initialise the grid
    // void initGrid();

};