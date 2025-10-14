#include <easy_grid_ros/easy_grid_demo.hpp>


EasyGridDemo::EasyGridDemo() : rclcpp::Node("easy_grid_demo")
{
    using std::placeholders::_1;
    //Load the params
    loadParams();
    // Init the grid
    initGrid();

    //Create the callback group
    keypress_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    double key_press_period = 1.0 / 5.0;
    keypress_timer_ = create_wall_timer(std::chrono::duration<double>(key_press_period),
                                        std::bind(&EasyGridDemo::KeyPressTimerCallback,this),
                                        keypress_cbg_);

    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point",
                                                                               10,
                                                                               std::bind(&EasyGridDemo::ClickedPointCallback,this,_1));
    
    //Publishers
    user_pt_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/user_point",10);
    polygon_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/polygon",10);
    pt_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/points",10);
    occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid",10);
}

void EasyGridDemo::KeyPressTimerCallback()
{
    std::scoped_lock kplock(keypress_mut_);
    auto key = getKey();
    if (!key.empty())
    {
        RCLCPP_WARN(get_logger(),"Key Pressed!");
        processKey(key);
    }
    // Publish the map
    auto grid_msg = toOccupancyGridMsg();
    occ_pub_->publish(grid_msg);
}

void EasyGridDemo::ClickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr ptmsg)
{
    geometry_msgs::msg::PointStamped user_pt_;
    user_pt_.header.frame_id = ptmsg->header.frame_id;
    user_pt_.header.stamp = this->now();
    user_pt_.point= ptmsg->point;
    user_pt_pub_->publish(user_pt_);

    //0: Ray Iterator
    //1: Submap Iterator
    //2: Circle Perimeter Iterator
    //3: Circle Area Iterator
    //4: Polygon Perimeter Iterator
    //5: Polygon Area Iterator
    size_t mode_to_use {0};
    double rad_to_use {1.0};
    size_t num_vertices_to_use {5};
    size_t submap_width_to_use {5};
    size_t submap_height_to_use {5};
    {
        std::scoped_lock kplock(keypress_mut_);
        mode_to_use = demo_mode_;
        rad_to_use = demo_rad_;
        num_vertices_to_use = demo_vertices_;
        submap_width_to_use = demo_submap_width_;
        submap_height_to_use = demo_submap_height_;
    }

    // Get the clicked point
    Eigen::Vector3d clicked_point;
    clicked_point.x() = ptmsg->point.x;
    clicked_point.y() = ptmsg->point.y;
    clicked_point.z() = ptmsg->point.z;

    RCLCPP_INFO_STREAM(get_logger(),"Clicked Point: (" << clicked_point.x() << " , " << clicked_point.y() << " , " << clicked_point.z() << ")");

    const auto filler = [this](Eigen::Vector2i &grid_coord)
                  {
                    auto fill = OccupancyCell();
                    fill.occupancy = 1;
                    this->occupancy_grid.setCell(grid_coord,fill);
                  };

    if (mode_to_use == 0)
    {
        // RayIteratorDemo
        if (ray_start_end.size() < 2)
        {
            ray_start_end.push_back(clicked_point);
            
            auto marker_msg = toPtMsg(ray_start_end);
            auto poly_msg = toPolygonMsg(ray_start_end);
            polygon_pub_->publish(poly_msg);
            pt_pub_->publish(marker_msg);

            if (ray_start_end.size() == 1)
            {
                RCLCPP_INFO(get_logger(),"Click One More Point For RayIterator End Point");
            }

            else
            {
                Eigen::Vector2d start = ray_start_end[0].head(2);
                Eigen::Vector2d end = ray_start_end[1].head(2);

                //RayIterator
                {
                    std::scoped_lock kplock(keypress_mut_);
                    occupancy_grid.RayIterator(start,end,true,filler);
                    ray_start_end.clear();
                }
            }
        }
    }

    else if (mode_to_use == 1)
    {
        RCLCPP_INFO_STREAM(get_logger(),"Iterating over Submap of " << submap_width_to_use << "W x " << submap_height_to_use <<"H");
        Eigen::Vector2d topl;
        topl.x() = clicked_point.x();
        topl.y() = clicked_point.y();
        //Start of lock
        std::scoped_lock kplock(keypress_mut_);
        occupancy_grid.SubMapIterator(topl,submap_width_to_use,submap_height_to_use,filler);
        //End of lock
    }

    else if (mode_to_use == 2)
    {
        RCLCPP_INFO_STREAM(get_logger(),"Iterating over Circle Perimeter with radius " << rad_to_use << "m");
        Eigen::Vector2d centr;
        centr.x() = clicked_point.x();
        centr.y() = clicked_point.y();
        //Start of lock
        std::scoped_lock kplock(keypress_mut_);
        occupancy_grid.CirclePerimeterIterator(centr,rad_to_use,filler);
        //End of lock
    }

    else if (mode_to_use == 3)
    {
        RCLCPP_INFO_STREAM(get_logger(),"Iterating over Circle Area with radius " << rad_to_use << "m");
        Eigen::Vector2d centr;
        centr.x() = clicked_point.x();
        centr.y() = clicked_point.y();
        //Start of lock
        std::scoped_lock kplock(keypress_mut_);
        occupancy_grid.CircleAreaIterator(centr,rad_to_use,filler);
        //End of lock
    }

    else if (mode_to_use == 4)
    {
        if (vertices.size() < num_vertices_to_use - 1)
        {
            vertices.push_back(clicked_point);
            auto marker_msg = toPtMsg(vertices);
            auto poly_msg = toPolygonMsg(vertices);
            pt_pub_->publish(marker_msg);
            polygon_pub_->publish(poly_msg);
            
            RCLCPP_INFO_STREAM(get_logger(),"Click " << num_vertices_to_use - vertices.size() << " more points!");
        }
        else
        {
            RCLCPP_INFO_STREAM(get_logger(),"Iterating over Polygon Perimeter with  " << num_vertices_to_use << " vertices");
            vertices.push_back(clicked_point);
            auto marker_msg = toPtMsg(vertices);
            auto poly_msg = toPolygonMsg(vertices);
            pt_pub_->publish(marker_msg);
            polygon_pub_->publish(poly_msg);

            {
                std::scoped_lock kplock(keypress_mut_);
                std::vector<Eigen::Vector2d> vertices_2d;
                vertices_2d.reserve(vertices.size());
                for (const auto &v3d : vertices)
                {
            
                    vertices_2d.emplace_back(v3d.x(),v3d.y());
                }
                occupancy_grid.PolygonPerimeterIterator(vertices_2d,filler); 
                vertices.clear();
            }
        }
        
    }

    else if (mode_to_use == 5)
    {
        if (vertices.size() < num_vertices_to_use - 1)
        {
            vertices.push_back(clicked_point);
            auto marker_msg = toPtMsg(vertices);
            auto poly_msg = toPolygonMsg(vertices);
            pt_pub_->publish(marker_msg);
            polygon_pub_->publish(poly_msg);
            
            RCLCPP_INFO_STREAM(get_logger(),"Click " << num_vertices_to_use - vertices.size() << " more points!");
        }
        else
        {
            vertices.push_back(clicked_point);
            RCLCPP_INFO_STREAM(get_logger(),"Iterating over Polygon Area with  " << num_vertices_to_use << " vertices");
            auto marker_msg = toPtMsg(vertices);
            auto poly_msg = toPolygonMsg(vertices);
            pt_pub_->publish(marker_msg);
            polygon_pub_->publish(poly_msg);

            {
                std::scoped_lock kplock(keypress_mut_);
                std::vector<Eigen::Vector2d> vertices_2d;
                vertices_2d.reserve(vertices.size());
                for (const auto &v3d : vertices)
                {
            
                    vertices_2d.emplace_back(v3d.x(),v3d.y());
                }
                occupancy_grid.PolygonAreaIterator(vertices_2d,filler); 
                vertices.clear();
            }
        }
        
    }
}

std::string EasyGridDemo::getKey()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    std::string key;
    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
        char buf[3] = {0};
        int len = read(STDIN_FILENO, buf, 3);
        if (len == 1) {
            key = std::string(buf, 1);   // normal key
        } else if (len == 3) {
            key = std::string(buf, 3);   // arrow key
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return key; 
}

void EasyGridDemo::processKey(std::string &key)
{
    if (key == "\x1b[A")  // UP
    {          
        demo_rad_ += 0.1;
        RCLCPP_INFO(this->get_logger(), "Radius increased: %.2f m", demo_rad_);
    } 
    
    else if (key == "\x1b[B")  // DOWN
    {   
        demo_rad_ -= 0.1;
        demo_rad_ = std::max(demo_rad_,0.1);
        RCLCPP_INFO(this->get_logger(), "Radius decreased: %.2f m", demo_rad_);
    }

    else if (key == "\x1b[C")  // RIGHT
    {   
        demo_vertices_ += 1;
        RCLCPP_INFO(this->get_logger(), "Vertices increased: %.2li vertices", demo_vertices_);
        vertices.clear();
    }

    else if (key == "\x1b[D")  // LEFT
    {   
        demo_vertices_ -= 1;
        demo_vertices_ = std::max(demo_vertices_,static_cast<size_t>(5));
        RCLCPP_INFO(this->get_logger(), "Vertices decreased: %.2li vertices", demo_vertices_);
        vertices.clear();
    }

    else if (key == "l")  // l
    {   
        demo_submap_width_ += 5;
        RCLCPP_INFO(this->get_logger(), "Submap Width increased: %.2li cells", demo_submap_width_);
    }

    else if (key == "j")  // j
    {   
        demo_submap_width_ -= 5;
        demo_submap_width_ = std::max(demo_submap_width_,static_cast<size_t>(5));
        RCLCPP_INFO(this->get_logger(), "Submap Width decreased: %.2li cells", demo_submap_width_);
    }

    else if (key == "p")  // p
    {   
        demo_submap_height_ += 5;
        RCLCPP_INFO(this->get_logger(), "Submap Height increased: %.2li cells", demo_submap_height_);
    }

    else if (key == "i")  // i
    {   
        demo_submap_height_ -= 5;
        demo_submap_height_ = std::max(demo_submap_height_,static_cast<size_t>(5));
        RCLCPP_INFO(this->get_logger(), "Submap Width decreased: %.2li cells", demo_submap_height_);
    }

    else if (key == "0")
    {
        demo_mode_ = static_cast<size_t>(std::stoi(key));
        auto demo_mode_name = MODE_LUT_.at(demo_mode_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Mode Changed To: " << demo_mode_name);
        ray_start_end.clear();
    }
    
    else if (key == "1" || key == "2" || key == "3") 
    {
        demo_mode_ = static_cast<size_t>(std::stoi(key));
        auto demo_mode_name = MODE_LUT_.at(demo_mode_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Mode Changed To: " << demo_mode_name);
        return;
    }

    else if (key == "4" || key == "5") 
    {
        demo_mode_ = static_cast<size_t>(std::stoi(key));
        auto demo_mode_name = MODE_LUT_.at(demo_mode_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Mode Changed To: " << demo_mode_name);
        vertices.clear();
        return;
    }

    else if (key == "r")
    {
        // Reset everything
        RCLCPP_INFO_STREAM(this->get_logger(), "RESETTING MAP");
        ray_start_end.clear();
        vertices.clear();
        occupancy_grid.forEachCellDo([this](Eigen::Vector2i &grid_coord)
                                    {
                                        auto free = OccupancyCell();
                                        this->occupancy_grid.setCell(grid_coord,free);
                                    });
    }

    else 
    {
        RCLCPP_ERROR(get_logger(),"Wrong Key Pressed.");
    }


    return;
}

void EasyGridDemo::loadParams()
{
    this->declare_parameter<double>("resolution",0.1);
    this->declare_parameter<int>("map_width",200);
    this->declare_parameter<int>("map_height",200);
    this->declare_parameter("map_translation",std::vector<double>());
    this->declare_parameter("map_rotation",std::vector<double>());
    this->declare_parameter("mode",0);
    this->declare_parameter("default_radius",2.0);
    this->declare_parameter("default_vertices",5);
    this->declare_parameter("default_submap_width",5);
    this->declare_parameter("default_submap_height",5);

    this->get_parameter("resolution",grid_meta.resolution);
    this->get_parameter("map_width",grid_meta.map_width);
    this->get_parameter("map_height",grid_meta.map_height);
    std::vector<double> translation = this->get_parameter("map_translation").as_double_array();
    std::vector<double> rotation = this->get_parameter("map_rotation").as_double_array(); 
    Eigen::Vector3d translation_vec;
    Eigen::Quaterniond quat;
    translation_vec.x() = translation[0];
    translation_vec.y() = translation[1];
    translation_vec.z() = translation[2];
    quat.x() = rotation[0];
    quat.y() = rotation[1];
    quat.z() = rotation[2];
    quat.w() = rotation[3];
    grid_meta.set_transform(translation_vec,quat);

    this->get_parameter("mode",demo_mode_);
    this->get_parameter("default_radius",demo_rad_);
    this->get_parameter("default_vertices",demo_vertices_);
    this->get_parameter("default_submap_width",demo_submap_width_);
    this->get_parameter("default_submap_height",demo_submap_height_);
}

void EasyGridDemo::initGrid()
{
    RCLCPP_INFO(get_logger(),"Initializing Grid");

    easy_grid::GridHandler<OccupancyCell> other(grid_meta); //Default initialises each CellT, in this case occupancy will be set to 0
    occupancy_grid = std::move(other);

    double resolution = occupancy_grid.getResolution();
    double map_width = occupancy_grid.getWidth();
    double map_height = occupancy_grid.getHeight();
    
    RCLCPP_INFO_STREAM(get_logger(),"OccupancyGrid of Resolution@" << resolution << " WxH@" << map_width << "x" << map_height <<" Created!");
}

geometry_msgs::msg::PolygonStamped EasyGridDemo::toPolygonMsg(std::vector<Eigen::Vector3d> &points)
{
    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header.frame_id = "map";
    polygon.header.stamp = this->now();
    polygon.polygon.points.reserve(points.size());

    for (const auto& p : points)
    {
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        polygon.polygon.points.push_back(pt);
    }

    return polygon;
}

visualization_msgs::msg::MarkerArray EasyGridDemo::toPtMsg(std::vector<Eigen::Vector3d> &points)
{
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "vertices";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = points[i].x();
        marker.pose.position.y = points[i].y();
        marker.pose.position.z = points[i].z();
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration(0, 0);

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

nav_msgs::msg::OccupancyGrid EasyGridDemo::toOccupancyGridMsg()
{
    nav_msgs::msg::OccupancyGrid occ;

    occ.header.frame_id = "map";
    occ.header.stamp = this->now();
    occ.info.height = occupancy_grid.getHeight();
    occ.info.width = occupancy_grid.getWidth();
    occ.info.resolution = occupancy_grid.getResolution();
    
    //Get the translation and rotation
    auto t = occupancy_grid.getMapTranslation();
    auto r_quat = occupancy_grid.getMapRotation();

    occ.info.origin.position.x = t.x();
    occ.info.origin.position.y = t.y();
    occ.info.origin.position.z = t.z();
    occ.info.origin.orientation.x = r_quat.x();
    occ.info.origin.orientation.y = r_quat.y();
    occ.info.origin.orientation.z = r_quat.z();
    occ.info.origin.orientation.w = r_quat.w();
    
    size_t TOTAL_CELLS = occupancy_grid.getWidth() * occupancy_grid.getHeight();
    occ.data.reserve(TOTAL_CELLS);

    occupancy_grid.forEachCellDo([this,&occ](Eigen::Vector2i &grid_coord)
                                {
                                    auto cellToPush = this->occupancy_grid.getCell(grid_coord);
                                    occ.data.push_back(cellToPush.occupancy*100);
                                });
    return occ;
}