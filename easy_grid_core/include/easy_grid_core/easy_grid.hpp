#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <vector>
#include <array>
#include <optional>
#include <unordered_set>
#include <cmath>
#include <type_traits>
#include <algorithm>

/*
Easy Grid is a simple, header only implementation for handling grids
in 2D. Use cases include Cost Maps, Traversibility Maps, Occupancy Grids and so on.
The main implementation can be found in the easy_grid::GridHandler class. 
GridHandler class is templated to take in some user defined struct CellT. 
This struct can contain as many fields as the user requires. This means that multiple grid layers,
for example Occupancy AND Traversibility AND Navigation Costs can be combined.
The grid cells, defined by CellT, are stored in a 1D std::vector. Coordinate frames are inevitably
important to this implementation. The 2D grid is relative to some coordinate frame. This coordinate frame
is referred to as the Parent frame. The origin of the 2D grid is defined by some SE3 Homogeonous transformation
from the Parent frame. This, along with map characteristics, make up the easy_grid::MetaData struct. Therefore, all
euclidean position information is relative to the Parent frame. Important note that the vertical translation of 
the map in parent frame is 0m. 

This implementation in particular provides a useful suite of features. Most notably, it provides several iterators
that can "do something" to grid_cells touched by the iterator using lambdas. This allows user to implement a wide variety 
of features depending on the semantic meaning the grid takes.

Useful Features:
1. Parent <---> Grid <---> Index conversions
2. Out Of Bounds checks
3. In polygon checks
4. Cell Iterator using lambdas
5. Bresenham Line Algorithm Iterator using lambdas
6. SubMap Iterator using lambdas
7. Circle Area and Circle Perimeter iterators using lambdas
8. Polygon Area and Polygon Perimeter iterators using lambdas

Recommended Usage:
You might use easy_grid::GridHandler within some larger implementation that gives meaning to each grid cell.
For example, an OccupancyGrid implementation might define a CellT with occupancy information and hold an instance
of easy_grid::GridHandler<CellT>. The logic to update cells from sensor information may be implemented by the OccupancyGrid
class, using features like the Iterators  
*/
namespace easy_grid
{
    /**
     * @brief Struct that contains information required to completely define a 2D grid in space with respect to some parent frame
     * @param resolution The spatial dimension (meters) of discretization of continuous space ie the size of each grid cell
     * @param map_width The number of cells that make up the width of the grid
     * @param map_height The number of cells that make up the height of the grid
     * @param map_frame_transform 3x3 SE(2) Transform of the origin of the grid in parent frame
     */
    struct MetaData
    {
        double resolution {0.1}; //m
        size_t map_width  {100}; //num_cells
        size_t map_height {100}; //num_cells
        Eigen::Matrix4d map_frame_transform = Eigen::Matrix4d::Identity(); //wTm

        // Set Transform
        void set_transform(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);
        // Get the transformation as 7d x,y,z,qx,qy,qz,qw
        Eigen::VectorXd get_transform_as_vec() const;
        // Get the transformation as Matrix4d
        Eigen::Matrix4d get_transform_as_mat() const;
        // Get Translation
        Eigen::Vector3d get_translation() const;
        // Get Rotation rotation matrix
        Eigen::Matrix3d get_rotation_as_matrix() const;
        // Get Rotation as Quaternion
        Eigen::Quaterniond get_rotation_as_quat() const;
    };
    

    template <class CellT>
    class GridHandler
    {
    public:
        /*
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        //////////////////////////////////CONSTRUCTORS AND ASSIGNMENT OPERATORS//////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        */

        /**
         * @brief Default Constructor
         */
        GridHandler() = default;

        /**
         * @brief Constructor that initializes a default initialized grid cells based on the metadata
         * @param MetaData Metadata object containing metadata information of the map
         * ok
         */
        explicit GridHandler(MetaData meta);

        /**
         * @brief Constructor with metadata and cells
         * @param MetaData MetaData Object
         * @param std::vector<CellT> cells
         * @see easy_grid::MetaData 
         * @warning This invalidates other_grid_cells as a move operation happens
         * ok
         */
        explicit GridHandler(MetaData meta, std::vector<CellT> &other_grid_cells);

        /**
         * @brief Default dtor
         */
        ~GridHandler() = default;

        /**
         * @brief Copy Constructor (Deleted!)
         * @remarks No copying allowed.
         */
        GridHandler(const GridHandler& other_grid) = delete;

        /**
         * @brief Copy Assignment Operator (Deleted!)
         * @remarks No copying allowed.
         * @test ok
         */
        GridHandler& operator=(const GridHandler& other_grid) = delete;

        /**
         * @brief Move Constructor
         * @param GridHandler &&other_grid rvalue ref
         * @warning std::vector<CellT> in other_grid will be invalidated. Do not reuse other_grid
         */
        GridHandler(GridHandler &&other_grid);

        /**
         * @brief Move Assignment Operator
         * @param GridHandler&& other_grid rvalue ref
         * @warning std::vector<CellT> in other_grid will be invalidated. Do not reuse other_grid
         */
        GridHandler& operator=(GridHandler &&other_grid);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        //////////////////////////////////////SETTERS, CLONING, CLEARING/////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Populate the grid handler with another set of grid_cells
         * @param std::vector<CellT> &cells
         * @warning If the incoming grid_cells.size() != TOTAL_CELLS = MetaData::map_width * MetaData::map_height cells, this will throw
         */
        void setGridCells(std::vector<CellT> &grid_cells);

        /**
         * @brief Set the metadata. This will erase existing grid_cells and default initialise the cells based on the new metadata
         * @param MetaData meta
         * @warning The existing grid_cells will be deleted, clearing and deallocating the vector that holds the data. A new vector will be initialized with default CellT instances
         */
        void setMetaData(MetaData meta);

        /**
         * @brief Creates a copy of an existing grid and returns the grid
         * @param GridHandler& other_grid The grid to copy
         * @return GridHandler Copy of other_grid
         */
        GridHandler cloneGrid(GridHandler &other_grid);
        
        /**
         * @brief Clears the grid, however space remains allocated
         */
        void clearGrid();
        
        /**
         * @brief Clears the grid and deallocates space
         */
        void clearAndDeallocGrid();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        ////////////////////////////////////////////PUBLIC GETTERS///////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Get the map resolution in meters
         * @return double resolution of the grid
         * @see easy_grid::MetaData
         */
        [[nodiscard]] double getResolution() const noexcept;

        /**
         * @brief Get the width of the map in number of cells
         * @return size_t width of the grid
         * @see easy_grid::MetaData
         */
        [[nodiscard]] size_t getWidth() const noexcept;

        /**
         * @brief Get the height of the map in number of cells
         * @return size_t height of the grid
         * @see easy_grid::MetaData
         */
        [[nodiscard]] size_t getHeight() const noexcept;

        /**
         * @brief Get the SE3 transform of the grid in parent frame
         * @return Eigen::Matrix4d SE3 transformation
         * @see easy_grid::MetaData
         */
        [[nodiscard]] Eigen::Matrix4d getMapTransform() const noexcept;

        /**
         * @brief Get the translation from of the grid in parent frame
         * @return Eigen::Vector3d Translation with height set to 0
         * @see easy_grid::MetaData
         */
        [[nodiscard]] Eigen::Vector3d getMapTranslation() const noexcept;

        /**
         * @brief Get the rotation from of the grid in parent frame
         * @return Eigen::Quaterniond 
         * @see easy_grid::MetaData
         */
        [[nodiscard]] Eigen::Quaterniond getMapRotation() const noexcept;

        /**
         * @brief Get the metadata of the map
         * @return MetaData
         * @see easy_grid::MetaData
         */
        [[nodiscard]] MetaData getMetaData() const noexcept;

        /**
         * @brief Get the total number of cells in the grid
         * @return size_t The number of cells in the grid
         */
        [[nodiscard]] size_t getTotalCells() const noexcept;

        /**
         * @brief Get the total capacity of the grid
         * @return size_t The capacity of the vector holding the cells.
         * @warning Capacity >= TotalCells
         */
        [[nodiscard]] size_t getCapacity() const noexcept;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        ///////////////////////////////////////COMMON CONVERSION METHODS/////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Convert a coordinate in parent frame to a grid coordinate
         * @param Eigen::Vector2d parent_coord
         * @return Eigen::Vector2i 2D Vector of integers containing grid coordinates
         */
        [[nodiscard]] Eigen::Vector2i parentToGrid(const Eigen::Vector2d &parent_coord) const noexcept;

        /**
         * @brief Convert a grid coordinate to a parent frame coordinate at the center of the cell
         * @param grid_coord
         * @return Eigen::Vector2d 2D Vector of doubles containing parent euclidean coordinates
         */
        [[nodiscard]] Eigen::Vector2d gridToParent(const Eigen::Vector2i &grid_coord) const noexcept;

        /**
         * @brief Convert a grid coordinate to an 1D index that can be used to access the data in the vector
         * @param Eigen::Vector2i grid_coord
         * @return size_t index to access the 1D vector containing the cells
         */
        [[nodiscard]] size_t gridToIndex(const Eigen::Vector2i &grid_coord) const noexcept;

        /**
         * @brief Convert an index to a grid coordinate
         * @param index
         * @return Eigen::Vector2i 2D Vector of integers containing grid coordinates
         */
        [[nodiscard]] Eigen::Vector2i indexToGrid(size_t index) const noexcept;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        /////////////////////////////////////////COMMON CHECK METHODS////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Check if a coordinate is out of bounds
         * @param Eigen::Vector2d& 2D coordinate of doubles in parent frame
         * @return true if out of bounds
         * @return false if in bounds
         */
        [[nodiscard]] bool outOfBound(Eigen::Vector2d &test_coord) const noexcept;

        /**
         * @brief Check if a coordinate is out of bounds
         * @param Eigen::Vector2d& 2D integer coordinates in grid coordinates
         * @return true if out of bounds
         * @return false if in bounds 
        */
        [[nodiscard]] bool outOfBound(Eigen::Vector2i &test_coord) const noexcept;

        /**
         * @brief Check if a coordinate is out of bounds
         * @param test_coord
         * @return true if out of bounds
         * @return false if in bounds
         */
        [[nodiscard]] bool outOfBound(size_t index) const noexcept;

        /**
         * @brief Check the sign of an arithmetic value
         * @param ArithmeticT value
         * @return int -1 if negative, 0 if zero, 1 if positive
         */
        template <typename ArithmeticT>
        [[nodiscard]] int whatsign(ArithmeticT value);

        /**
         * @brief Check if an arithmetic value is near 0 (0+ or 0-)
         * @param ArithmeticT value
         * @param ArithmeticT eps Epsilon value that defines what near is, such that abs(value) - eps <= 0
         * @return true if value is near 0
         * @return false if value is not near 0
         */
        template <typename ArithmeticT>
        [[nodiscard]] constexpr bool nearZero(ArithmeticT value,ArithmeticT eps = static_cast<ArithmeticT>(1e-12)) noexcept;

        /**
         * @brief Check if a point is in a polygon
         * @param Eigen::Vector2d&  2D coordinates of doubles in parent frame
         * @param std::vector<Eigen::Vector2d>& polygon The polygon in parent coordinates
         * @return true if in polygon
         * @return false if not in polygon
         */
        [[nodiscard]] bool pointInPolygon(Eigen::Vector2d& point, 
                                            std::vector<Eigen::Vector2d>& polygon,
                                            bool include_perimeter);

        /**
         * @brief Check if a point is on a segment of the polgon
         * @param Eigen::Vector2d& p The point in parent coordinates
         * @param Eigen::Vector2d& a The start of the segment in parent coordinates
         * @param Eigen::Vector2d& b The end of the segment in parent coordinates
         * @return true if on segment
         * @return false if not on segment
         */
        [[nodiscard]] bool pointOnSegment(Eigen::Vector2d& p, Eigen::Vector2d& a, Eigen::Vector2d& b);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        ///////////////////////////////////////CELL NEIGHBORHOOD METHODS/////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Compute and return the cardinal neighbors
         * @param Eigen::Vector2i &source_grid_coord The source 2D grid coordinate
         * @return std::array<Eigen::Vector2i,4> of cardinal neighbors in 2D integer grid coordinates
         * @warning Out of bounds checks are not done, this is up to the user to use outOfBounds()
         * @see easy_grid::GridHandler::outOfBounds()
         */
        std::array<Eigen::Vector2i,4> getCardinalNeighbors(Eigen::Vector2i &source_grid_coord);

        /**
         * @brief Compute and return the octile neighbors
         * @param Eigen::Vector2i &source_grid_coord The source 2D grid coordinate
         * @return std::array<Eigen::Vector2i,8> of octile neighbors in 2D integer grid coordinates
         * @warning Out of bounds checks are not done, this is up to the user to use outOfBounds()
         * @see easy_grid::GridHandler::outOfBounds()
         */
        std::array<Eigen::Vector2i,8> getOctileNeighbors(Eigen::Vector2i &source_grid_coord);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        /////////////////////////////////////CELL ACCESSORS AND MODIFIERS////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Access the grid as a Cell<T>&
         * @param size_t index
         * @return CellT&
         * @warning No bounds checking, bounds overflow will throw
         * @warning Cell will be mutable
         */
        CellT& getCell(size_t index);
        
        /**
         * @brief Access the grid as a Cell<T>&
         * @param Eigen::Vector2i grid_coord
         * @return CellT&
         * @warning No bounds checking, bounds overflow will throw
         * @warning Cell will be mutable
         */
        CellT& getCell(Eigen::Vector2i &grid_coord);

        /**
         * @brief Access the grid as a Cell<T>&
         * @param world coordinates
         * @return CellT&
         * @warning No bounds checking, bounds overflow will throw
         * @warning Cell will be mutable
         */
        CellT& getCell(Eigen::Vector2d &parent_coord);

        /**
         * @brief Get values of cells.
         * @param indices Indices of cells to query.
         * @return vector of cell values.
         * * @warning No bounds checking, bounds overflow will throw
         */
        std::vector<CellT> getCells(const std::vector<size_t>& indices);

        /**
         * @brief Set the value of a cell
         * @param size_t index
         * @param cell
         * @warning No bounds checking, bounds overflow will throw
         */
        void setCell(size_t index, const CellT &cell);

        /**
         * @brief Set the value of a cell
         * @param Eigen::Vector2i grid_coord
         * @param cell
         * @warning No bounds checking, bounds overflow will throw
         */
        void setCell(Eigen::Vector2i &grid_coord, const CellT &cell);

        /**
         * @brief Set the value of a cell
         * @param Eigen::Vector2d parent_coord 
         * @param CellT cell
         * @warning No bounds checking, bounds overflow will throw
         */
        void setCell(Eigen::Vector2d &parent_coord, const CellT &cell);

        /**
         * @brief Set cells with values.
         * @param indices Indices of cells to set.
         * @param values Values to set, must be same size as indices.
         */
        void setCells(const std::vector<size_t>& indices, const std::vector<CellT> values);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////DEFINITION/////////////////////////////////////////////////////
        ////////////////////////////////////////////CELL ITERATORS///////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
         * @brief Iterate over the cells in the grid
         * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
         * @warning This function is not const, as it will mutate the cells
         */
        template<class VisitorFunction>
        void forEachCellDo(VisitorFunction&& visitor,
                            std::optional<std::vector<size_t>> selected_indices = std::nullopt);
        
        /**
        * @brief Ray Iterator for the grid that applies VisitorFunction to the points in the ray
        * @param Eigen::Vector2d& key_origin The origin of the ray in world coordinates
        * @param Eigen::Vector2d& key_end The end of the ray in world coordinates
        * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
        */
        template <class VisitorFunction>
        void RayIterator(const Eigen::Vector2d& origin_point,
                            const Eigen::Vector2d& end_point, bool include_end, VisitorFunction&& visitor_func);

        /**
         * @brief Iterates over a submap of the grid
         * @param Eigen::Vector2d& top_left_world The top left corner of the submap in world coordinates
         * @param size_t submap_width The width of the submap
         * @param size_t submap_height The height of the submap
         * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
         * @warning Submap iterator will continue even if the top left grid coordinate is out of bounds
         */
        template <class VisitorFunction>
        void SubMapIterator(Eigen::Vector2d& top_left_parent , size_t submap_width, size_t submap_height, VisitorFunction&& visitor_func);
        
        /**
        * @brief Circle Iterator for the grid that applies VisitorFunction to the points in circle perimeter given a center
        * @param Eigen::Vector2d& key_origin The origin of the ray in world coordinates
        * @param Eigen::Vector2d& key_end The end of the ray in world coordinates
        * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coordc  
        */
        template <class VisitorFunction>
        void CirclePerimeterIterator(Eigen::Vector2d& center_parent, double radius, VisitorFunction&& visitor_func);
        
        /**
        * @brief Circle Iterator for the grid that applies VisitorFunction to the points in circle perimeter and inside the circle given a center
        * @param Eigen::Vector2d& key_origin The origin of the ray in world coordinates
        * @param Eigen::Vector2d& key_end The end of the ray in world coordinates
        * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
        */
        template <class VisitorFunction>
        void CircleAreaIterator(Eigen::Vector2d& center_world, double radius, VisitorFunction&& visitor_func);
                    
        /**
         * @brief Iterates over the perimeter of a given polygon on the map
         * @param std::vector<Eigen::Vector2d>& polygon The polygon in world coordinates
         * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
         * @remarks The approach calls RayIterator between each pair of vertices
         * @warning Ensure that the polygon points are ordered clockwise
         */
        template <class VisitorFunction>
        void PolygonPerimeterIterator(std::vector<Eigen::Vector2d>& polygon, VisitorFunction&& visitor_func);

        /**
         * @brief Iterates over the area of a given polygon on the map
         * @param std::vector<Eigen::Vector2d>& polygon The polygon in world coordinates
         * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coord
         * @remarks The approach calls RayIterator between each pair of vertices
         */
        template <class VisitorFunction>
        void PolygonAreaIterator(std::vector<Eigen::Vector2d>& polygon, VisitorFunction&& visitor_func);
    
    private:
        MetaData meta_;
        std::vector<CellT> grid_cells_;
        
        //Cardinal Neighbor Hops
        const std::array<Eigen::Vector2i,4> CARDINAL_HOPS = 
        {{
            Eigen::Vector2i{ 1 ,   0},  // right
            Eigen::Vector2i{ 0 ,   1},  // up
            Eigen::Vector2i{ -1,   0},  // left
            Eigen::Vector2i{ 0 ,  -1},  // down
        }};

        //Octile Neighbor Hops
        const std::array<Eigen::Vector2i,8> OCTILE_HOPS = 
        {{
            Eigen::Vector2i{ 1,  0},  // right
            Eigen::Vector2i{ 0,  1},  // up
            Eigen::Vector2i{-1,  0},  // left
            Eigen::Vector2i{ 0, -1},  // down
            Eigen::Vector2i{ 1,  1},  // up-right
            Eigen::Vector2i{-1,  1},  // up-left
            Eigen::Vector2i{-1, -1},  // down-left
            Eigen::Vector2i{ 1, -1}   // down-right
        }};
        
    };
    /*
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    //////////////////////////////////////////easy_grid::Metadata////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */
    inline void MetaData::set_transform(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation)
    {
        // Ensure rotation is normalized to avoid drift
        if (std::abs(rotation.norm() - 1.0) > 1e-9)
            rotation.normalize();

        map_frame_transform = Eigen::Matrix4d::Identity();
        map_frame_transform.topLeftCorner<3,3>() = rotation.toRotationMatrix(); // Rotation block
        map_frame_transform.topRightCorner<3,1>() = translation;    
    }


    inline Eigen::VectorXd MetaData::get_transform_as_vec() const
    {
        /**
         * @brief Returns [x, y, z, qx, qy, qz, qw] as a 7D vector.
         */
        Eigen::Vector3d t = get_translation();
        Eigen::Quaterniond q = get_rotation_as_quat();

        Eigen::VectorXd vec(7);
        vec << t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w();
        return vec;
    }

    inline Eigen::Matrix4d MetaData::get_transform_as_mat() const
    {
        /**
         * @brief Returns the full 4x4 homogeneous transform.
         */
        return map_frame_transform;
    }

    inline Eigen::Vector3d MetaData::get_translation() const
    {
        /**
         * @brief Extracts translation (x, y, z) from the 4x4 matrix.
         */
        return map_frame_transform.topRightCorner<3,1>();
    }

    inline Eigen::Matrix3d MetaData::get_rotation_as_matrix() const
    {
        /**
         * @brief Extracts rotation matrix (3x3) from the 4x4 transform.
         */
        return map_frame_transform.topLeftCorner<3,3>();
    }

    inline Eigen::Quaterniond MetaData::get_rotation_as_quat() const
    {
        /**
         * @brief Returns rotation as quaternion.
         */
        Eigen::Matrix3d R = get_rotation_as_matrix();
        return Eigen::Quaterniond(R);
    }

    /*
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    ////////////////////////////////////////easy_grid::GridHandler/////////////////////////////////////////////////// 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */

    /*
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////IMPLEMENTATION////////////////////////////////////////////////////////
    //////////////////////////////////CONSTRUCTORS AND ASSIGNMENT OPERATORS//////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */
        template <class CellT>
        GridHandler<CellT>::GridHandler(MetaData meta) 
        : meta_(meta)
        {
            //Default Populate
            grid_cells_.assign(meta_.map_width * meta_.map_height,CellT());
            
        }

        template <class CellT>
        GridHandler<CellT>::GridHandler(MetaData meta, std::vector<CellT> &other_grid_cells)
        : meta_(meta)
        {
            size_t total_cells = meta_.map_height * meta_.map_width;
            if (other_grid_cells.size() != total_cells)
            {
                //precent the user from providing cells that dont match the metdata
                throw std::runtime_error("easy_grid::GridHandler::GridHandler(MetaData,std::vector<CellT> &) Invalid number of other cells provided");
            }
            else
            {
                //Otherwise, just move it
                grid_cells_ = std::move(other_grid_cells);
            }
        }

        template <class CellT>
        GridHandler<CellT>::GridHandler(GridHandler &&other_grid)
        {
            //Receive another GridHanlder, just move it over
            meta_ = other_grid.meta_;
            grid_cells_ = std::move(other_grid.grid_cells_);
        }

        template <class CellT>
        GridHandler<CellT>& GridHandler<CellT>::operator=(GridHandler &&other_grid)
        {
            meta_ = other_grid.meta_;
            grid_cells_ = std::move(other_grid.grid_cells_);
            return *this;
        }

    /*
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////IMPLEMENTATION////////////////////////////////////////////////////////
    //////////////////////////////////////SETTERS, CLONING, CLEARING/////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */
        template <class CellT>
        void GridHandler<CellT>::setGridCells(std::vector<CellT> &grid_cells) 
        {
            //When the user decides to set just the grid, we must make sure that the meta is the same for height and width at the very least
            //This will throw if the size is not the same
            size_t width = getWidth();
            size_t height = getHeight();
            size_t metadata_total_size = width * height;
            if (metadata_total_size != grid_cells.size())
            {
                throw std::runtime_error("GridHandler::setGrid: Invalid number of cells provided");
                return;
            }
            // Move the grid
            grid_cells_ = std::move(grid_cells);
        }

        template <class CellT>
        void GridHandler<CellT>::setMetaData(MetaData meta) 
        {
            meta_ = meta;
            clearAndDeallocGrid();
            grid_cells_.assign(meta_.map_width * meta_.map_height,CellT());
        }

        template <class CellT>
        GridHandler<CellT> GridHandler<CellT>::cloneGrid(GridHandler &other_grid)
        {
            size_t TOTAL_CELLS = other_grid.meta_.map_height * other_grid.meta_.map_width;
            std::vector<CellT> copy_cells;
            copy_cells.reserve(TOTAL_CELLS);
            other_grid.forEachCellDo([this,&copy_cells,&other_grid](Eigen::Vector2i &grid_coord)
                                     {
                                        CellT cell2copy = other_grid.getCell(grid_coord);
                                        copy_cells.push_back(cell2copy);
                                     });
            
            GridHandler grid2return(other_grid.meta_,copy_cells);
            return grid2return;
        }

        template <class CellT>
        void GridHandler<CellT>::clearGrid()
        {
            grid_cells_.clear();
        }

        template <class CellT>
        void GridHandler<CellT>::clearAndDeallocGrid()
        {
            //Swap an empty vector with 0 allocated space with our full grid
            std::vector<CellT>().swap(grid_cells_);
            //At this point, the temporary vector goes out of scope, bringing the cells along with it
            //Our grid_ is empty
        }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    ////////////////////////////////////////////PUBLIC GETTERS///////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <class CellT>
    double GridHandler<CellT>::getResolution() const noexcept {return meta_.resolution;}

    template <class CellT>
    size_t GridHandler<CellT>::getWidth() const noexcept {return meta_.map_width;}

    template <class CellT>
    size_t GridHandler<CellT>::getHeight() const noexcept {return meta_.map_height;}

    template <class CellT>
    Eigen::Matrix4d GridHandler<CellT>::getMapTransform() const noexcept {return meta_.map_frame_transform;}

    template <class CellT>
    Eigen::Vector3d GridHandler<CellT>::getMapTranslation() const noexcept
    {
        return this->meta_.get_translation();
    }

    template <class CellT>
    Eigen::Quaterniond GridHandler<CellT>::getMapRotation() const noexcept
    {
        return this->meta_.get_rotation_as_quat();   
    }

    template <class CellT>
    MetaData GridHandler<CellT>::getMetaData() const noexcept {return meta_;}

    template <class CellT>
    size_t GridHandler<CellT>::getTotalCells() const noexcept {return grid_cells_.size();}

    template <class CellT>
    size_t GridHandler<CellT>::getCapacity() const noexcept {return grid_cells_.capacity();}

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    ///////////////////////////////////////COMMON CONVERSION METHODS/////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template <class CellT>
    Eigen::Vector2i GridHandler<CellT>::parentToGrid(const Eigen::Vector2d &parent_coord) const noexcept
    {
        //World Coord in a point in world coord WP
        //We need to convert it to a point in map coord --> MP
        //This is done by MP = MTW * WP
        //WTM is the transformation from the origin to the map frame
        //Which is equivalent to MP = (WTM)^-1 * WP, where WTM is a 3x3 matrix gotten from grid.map_tf()

        //Get the map wrt to world frame transform
        auto WTM = getMapTransform(); //4x4 matrix
        //Invert it to bring points from world frame to map frame
        auto MTW = WTM.inverse();
        
        //Get the world frame
        Eigen::Vector4d WP_4 {parent_coord.x(),parent_coord.y(),0.0,1.0};
        //Do MP = MTW * WP
        Eigen::Vector4d MP_4 = MTW * WP_4;
        
        //And finally, convert it to grid coordiate
        double res = getResolution();
        //Logical grid (u,v) wjere u -> +x (right) and v -> +y (up), origin at bottom left
        int u = static_cast<int>(std::floor(MP_4(0) / res));
        int v = static_cast<int>(std::floor(MP_4(1) / res));
        
        return {u,v};
    }

    template <class CellT>
    Eigen::Vector2d GridHandler<CellT>::gridToParent(const Eigen::Vector2i &grid_coord) const noexcept
    {
        //Get the grid coordiate
        int u = grid_coord.x();
        int v = grid_coord.y();

        //Grab the resolution
        double res = getResolution();
        
        //Get the transformation
        Eigen::Matrix4d WTM = getMapTransform(); //4x4

        Eigen::Vector4d MP_4{  (static_cast<double>(u) + 0.5) * res, //returns the cell center position
                               (static_cast<double>(v) + 0.5) * res,
                                                 0.0,
                                                 1.0
                            };
        //WP = WTM * MP
        Eigen::Vector4d WP_4 = WTM * MP_4;

        return {WP_4(0),WP_4(1)};
    }

    template <class CellT>
    size_t GridHandler<CellT>::gridToIndex(const Eigen::Vector2i &grid_coord) const noexcept
    {
        int u = grid_coord.x();
        int v = grid_coord.y();
        
        size_t width = getWidth();

        return static_cast<size_t>(v) * width + static_cast<size_t>(u);
    }

    template <class CellT>
    Eigen::Vector2i GridHandler<CellT>::indexToGrid(size_t index) const noexcept
    {
        auto width = getWidth();

        int v = static_cast<int>(index / width);
        const int u = static_cast<int>(index % width);

        return Eigen::Vector2i {u,v};
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    /////////////////////////////////////////COMMON CHECK METHODS///////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template <class CellT>
    bool GridHandler<CellT>::outOfBound(Eigen::Vector2d &test_coord) const noexcept
    {
        Eigen::Vector2i test_coord_int = this->parentToGrid(test_coord);
        return this->outOfBound(test_coord_int);
    }

    template <class CellT>
    bool GridHandler<CellT>::outOfBound(Eigen::Vector2i &test_coord) const noexcept
    {   
        //Number of cells along the width
        const int W = static_cast<int>(getWidth());
        const int H = static_cast<int>(getHeight());

        return test_coord.x() < 0  || 
                test_coord.x() >= W || 
                test_coord.y() < 0  || 
                test_coord.y() >= H;
    }

    template <class CellT>
    bool GridHandler<CellT>::outOfBound(size_t index) const noexcept
    {
        const size_t total = static_cast<size_t>(getWidth()) * static_cast<size_t>(getHeight());

        return index >= total;
    }

    template <class CellT>
    template <class ArithmeticT>
    int GridHandler<CellT>::whatsign(ArithmeticT value) 
    {
        static_assert(std::is_arithmetic<ArithmeticT>::value, "whatsign requires arithmetic type.");
        
        if (value > 0) return 1;
        else if (value < 0) return -1;
        else return 0;
    }

    template <class CellT>
    template <class ArithmeticT>
    constexpr bool GridHandler<CellT>::nearZero(ArithmeticT value,ArithmeticT eps) noexcept
    {
        static_assert(std::is_arithmetic<ArithmeticT>::value, "nearZero requires arithmetic type.");
        
        return std::fabs(static_cast<long double>(value)) <= static_cast<long double>(eps);
    }

    template <class CellT>
    bool GridHandler<CellT>::pointInPolygon(Eigen::Vector2d& point, 
                                            std::vector<Eigen::Vector2d>& polygon,
                                            bool include_perimeter)
    {
        // Polygon must have at least 3 vertices
        size_t n = polygon.size();
        if (n < 3) return false;

        // Perimeter inclusion (robust)
        if (include_perimeter) 
        {
            for (size_t i = 0; i < n; ++i) 
            {
                Eigen::Vector2d& a = polygon[i];
                Eigen::Vector2d& b = polygon[(i + 1) % n];
                if (this->pointOnSegment(point, a, b)) return true;
            }
        }

        // Even-odd rule (ray-cast to +x)
        size_t count = 0;
        const double px = point.x();
        const double py = point.y();

        for (size_t i = 0; i < n; ++i) 
        {
            const Eigen::Vector2d& v1 = polygon[i];
            const Eigen::Vector2d& v2 = polygon[(i + 1) % n];

            // Skip horizontal edges to avoid double-count at vertices
            if (nearZero(v1.y() - v2.y())) continue;

            // Check if the ray at y=py crosses edge (v1,v2)
            const bool crosses = ((v1.y() > py) != (v2.y() > py));
            if (crosses) 
            {
                const double xin = (v2.x() - v1.x()) * (py - v1.y()) / (v2.y() - v1.y()) + v1.x();
                if (px < xin) ++count;
            }
        }
        return (count % 2) == 1;
    }

    template <class CellT>
    bool GridHandler<CellT>::pointOnSegment(Eigen::Vector2d& p, Eigen::Vector2d& a, Eigen::Vector2d& b)
    {
        const Eigen::Vector2d ab = b - a;
        const double ab_len2 = ab.squaredNorm();

        // Degenerate segment (a == b): treat as a point
        if (nearZero(ab_len2)) {
            return (p - a).norm() <= 1e-9;
        }

        const Eigen::Vector2d ap = p - a;
        const double t = ap.dot(ab) / ab_len2;  // param along segment

        // allow tiny numerical slack beyond [0,1]
        if (t < -1e-12 || t > 1.0 + 1e-12) return false;

        const Eigen::Vector2d proj = a + t * ab;
        const double dist = (p - proj).norm();
        return dist <= 1e-9;  // tune if needed
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////IMPLEMENTATION///////////////////////////////////////////////////////
    ///////////////////////////////////////CELL NEIGHBORHOOD METHODS////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template <class CellT>
    std::array<Eigen::Vector2i,4> GridHandler<CellT>::getCardinalNeighbors(Eigen::Vector2i &source_grid_coord)
    {
        std::array<Eigen::Vector2i,4> neighbors;
        for (size_t i = 0 ; i < 4 ; ++i)
        {
            neighbors[i]  = source_grid_coord + CARDINAL_HOPS[i];
        }
        return neighbors;
    }

    template <class CellT>
    std::array<Eigen::Vector2i,8> GridHandler<CellT>::getOctileNeighbors(Eigen::Vector2i &source_grid_coord)
    {
        std::array<Eigen::Vector2i,8> neighbors;
        for (size_t i = 0 ; i < 8 ; ++i)
        {
            neighbors[i]  = source_grid_coord + OCTILE_HOPS[i];
        }
        return neighbors;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////IMPLEMENTATION/////////////////////////////////////////////////////
    /////////////////////////////////////CELL ACCESSORS AND MODIFIERS////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template <class CellT>
    CellT& GridHandler<CellT>::getCell(size_t index)
    {
        return grid_cells_.at(index);
    }

    template <class CellT>
    CellT& GridHandler<CellT>::getCell(Eigen::Vector2i &grid_coord)
    {
        size_t index = this->gridToIndex(grid_coord);
        return getCell(index);
    }

    template <class CellT>
    CellT& GridHandler<CellT>::getCell(Eigen::Vector2d &parent_coord)
    {
        Eigen::Vector2i grid_coord = this->parentToGrid(parent_coord);
        
        return getCell(grid_coord);
    }

    template <class CellT>
    inline std::vector<CellT> GridHandler<CellT>::getCells(const std::vector<size_t> &indices)
    {
        std::vector<CellT> out;
        out.reserve(indices.size());

        for (size_t index : indices)
        {
            CellT cell = getCell(index);
            out.push_back(cell);
        }

        return out;
    }

    template <class CellT>
    void GridHandler<CellT>::setCell(size_t index, const CellT &cell)
    {
        grid_cells_.at(index) = cell;
    }

    template <class CellT>
    void GridHandler<CellT>::setCell(Eigen::Vector2i &grid_coord, const CellT &cell)
    {
        size_t index = this->gridToIndex(grid_coord);
        setCell(index, cell);
    }

    template <class CellT>
    void GridHandler<CellT>::setCell(Eigen::Vector2d &parent_coord, const CellT &cell)
    {
        Eigen::Vector2i grid_coord = this->parentToGrid(parent_coord);
        setCell(grid_coord, cell);
    }

    template<class CellT>
    inline void GridHandler<CellT>::setCells(const std::vector<size_t>& indices, const std::vector<CellT> values)
    {
        if (indices.size() != values.size())
        {
            throw std::runtime_error("easy_grid::GridHandler::setCells receieved different sized indices and values vectors");
        }
        for (size_t i = 0; i < indices.size(); ++i)
        {
            size_t index = indices[i];
            setCell(index,values[i]);
        }
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::forEachCellDo(VisitorFunction&& visitor,
                                std::optional<std::vector<size_t>> selected_indices)
    {
        if (selected_indices)
        {
            //Apply the visitor only to the selected indices
            for (size_t index : *selected_indices)
            {
                Eigen::Vector2i grid_coord = this->indexToGrid(index);
                visitor(grid_coord);
            }
        }

        else
        {
            //Apply to all the indices
            for (size_t index = 0 ; index < grid_cells_.size() ; ++index)
            {
                Eigen::Vector2i grid_coord = this->indexToGrid(index);
                visitor(grid_coord);
            }
        }
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::RayIterator(const Eigen::Vector2d& origin_point,
                                        const Eigen::Vector2d& end_point, bool include_end, VisitorFunction&& visitor_func)
    {
        Eigen::Vector2i src =  this->parentToGrid(origin_point);
        Eigen::Vector2i dest = this->parentToGrid(end_point);
        
        //Same point, just return
        if (src == dest)
        {
            if (!outOfBound(src))
            {
                visitor_func(src);
            }
            
            return;
        }

        int Di = dest.x() - src.x();
        int Dj = dest.y() - src.y();
        int Dl;
        int Ds;
        int l;
        int s;
        int lf;
        int sf;
        bool swapped;

        //Visit the first cell
        if (!this->outOfBound(src)) {visitor_func(src);}
        

        if (std::abs(Di) > std::abs(Dj))
        {
            swapped = false;
            Dl = Di;
            Ds = Dj;
            l = src.x();
            s = src.y();
            lf = dest.x();
            sf = dest.y();
        }

        else
        {
            swapped = true;
            Dl = Dj;
            Ds = Di;
            l = src.y();
            s = src.x();
            lf = dest.y();
            sf = dest.x();
        }

        int ds = this->whatsign(Ds);
        int dl = this->whatsign(Dl);
        int abs_Dl = std::abs(Dl);
        int d_esl = abs_Dl * ds;
        int esl = 0;

        while(l != lf || s != sf)
        {
            l += dl;
            esl += Ds;
            if (2 * std::abs(esl) >= abs_Dl)
            {
                esl -= d_esl;
                s += ds;
            }
            //Break if end cell, we'll add it separately
            if (l == lf && s == sf) break;
            int i,j;
            i = swapped ? s : l;
            j = swapped ? l : s;
            Eigen::Vector2i pt {i,j};
            if (this->outOfBound(pt)) continue;
            visitor_func(pt);
        }

        if (include_end)
        {
            if (!this->outOfBound(dest)) {visitor_func(dest);}
        }
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::SubMapIterator(Eigen::Vector2d& top_left_parent , size_t submap_width, size_t submap_height, VisitorFunction&& visitor_func)
    {   
        //Convert to grid coordinate
        Eigen::Vector2i top_left_grid_coord = this->parentToGrid(top_left_parent);

        //Iterate
        for (size_t di = 0 ; di < submap_height ; ++di)
        {
            //Adjust the coordinates
            int v = top_left_grid_coord.y() - static_cast<int>(di);
            
            for (size_t dj = 0 ; dj < submap_width ; ++dj)
            {
                //Adjust the coordinates
                int u = top_left_grid_coord.x()  + static_cast<int>(dj);
                //Get the coordinate to apply the function
                Eigen::Vector2i coord_to_apply_func {u,v};
                //Skip it if it is out of bounds
                if (this->outOfBound(coord_to_apply_func)) continue;
                //Apply the visitor function
                visitor_func(coord_to_apply_func);
            }
        }
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::CirclePerimeterIterator(Eigen::Vector2d& center_parent, double radius, VisitorFunction&& visitor_func)
    {   
        /*
        Implementation uses Circle Midpoint Algorithm, aka Bresenham Circle Algorithm
        */
        //Convert to grid
        Eigen::Vector2i center_grid = this->parentToGrid(center_parent);
        double resolution = getResolution();
        int radius_cells = static_cast<int>(std::lround(radius / resolution));
        
        if (radius < 0.0) return;

        //Visited indices set to prevent duplicates
        std::unordered_set<size_t> visited_indices;
        //Lambda to walk through symmetric points
        auto visit_symmetric_points = [&](int dx, int dy) 
        {
            std::array<Eigen::Vector2i, 8> points = 
            {{
                center_grid + Eigen::Vector2i{ dx,  dy},
                center_grid + Eigen::Vector2i{ dy,  dx},
                center_grid + Eigen::Vector2i{-dx,  dy},
                center_grid + Eigen::Vector2i{-dy,  dx},
                center_grid + Eigen::Vector2i{-dx, -dy},
                center_grid + Eigen::Vector2i{-dy, -dx},
                center_grid + Eigen::Vector2i{ dx, -dy},
                center_grid + Eigen::Vector2i{ dy, -dx}
            }};

            for (auto& pt : points) 
            {
                //Continue if we are out of bounds
                if (this->outOfBound(pt)) continue;
                size_t perimeter_index = this->gridToIndex(pt);
                //I have visited it before, so I can just skip this
                if (visited_indices.count(perimeter_index)) continue;
                //Mark as visited
                visited_indices.insert(perimeter_index);
                //Apply the visitor function
                visitor_func(pt);
            }
        };

        int x = 0;
        int y = radius_cells;
        int d = 1 - radius_cells;

        //Visit the top most part of the circle and 
        visit_symmetric_points(x,y);

        while (x < y) 
        {
            ++x;
            if (d < 0) {d += 2 * x + 1;} 
            
            else 
            {
                --y;
                d += 2 * (x - y) + 1;
            }
            visit_symmetric_points(x, y);
        }
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::CircleAreaIterator(Eigen::Vector2d& center_parent, double radius, VisitorFunction&& visitor_func)
    {
        //Return if the radius is negative
        if (radius < 0.0) return;

        //Convert to grid
        Eigen::Vector2i center_grid = this->parentToGrid(center_parent);
        //Get the radius
        int radius_cells = static_cast<int>(std::round(radius / getResolution()));
        //Get the rad squared for faster euclidean comp without rooting
        int radius_sq = radius_cells * radius_cells;
        
        //Find the top left corner of the submap encompassing the circle
        Eigen::Vector2d top_left_parent( center_parent.x() - radius, 
                                        center_parent.y() + radius);
        
        size_t width =  static_cast<size_t>(2  * radius_cells + 1);
        size_t height = static_cast<size_t>(2  * radius_cells + 1);

        //Resuse the submap iterator
        SubMapIterator(top_left_parent, width, height,
                        [&](Eigen::Vector2i& pt)
                        {    
                            //Do a bounds check
                            if (this->outOfBound(pt)) return;
                            //Compute how far away the cell is
                            Eigen::Vector2i diff = pt - center_grid;
                            //Compute the squared euc distance in cell coordinate
                            int dist_sq = diff.x() * diff.x() + diff.y() * diff.y();
                            if (dist_sq < radius_sq)
                            {
                                //If its less than or equal to the radius diff, its inside the circle
                                //So we can apply the visitor func
                                visitor_func(pt);
                            }
                        }
                        );
    }

    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::PolygonPerimeterIterator(std::vector<Eigen::Vector2d>& polygon, VisitorFunction&& visitor_func)
    {
        //Get the number of vertices
        size_t num_vertices = polygon.size();

        //Return if its less than 3
        if (num_vertices < 3) {return;}

        std::unordered_set<size_t> deduplicator;
        //We start at index 0, and assume index 0 and index 1 have an edge, 1 and 2 have an edge, ... n-1 and n have an edge
        for (size_t m = 0 ; m < num_vertices ; ++m)
        {
            //Get the end coordinate, wrapping around back to 0 whe i = num_vertices - 1
            size_t n = (m + 1) % num_vertices;
            //Do the ray iterator
            RayIterator(polygon[m],polygon[n],
                        /*include_end=*/false,
                        [&](Eigen::Vector2i&pt)
                        {
                            //Do a bounds check, skip if outofbounds
                            if (this->outOfBound(pt)) return;
                            //Convert to index
                            size_t perimeter_index = this->gridToIndex(pt);
                            //Skip if the index exists in the deduplicator
                            if (deduplicator.count(perimeter_index)) return;
                            //Add to deduplicator
                            deduplicator.insert(perimeter_index);
                            //Apply visitor
                            visitor_func(pt);
                        }
                        );
        }
    }
    template <class CellT>
    template <class VisitorFunction>
    void GridHandler<CellT>::PolygonAreaIterator(std::vector<Eigen::Vector2d>& polygon, VisitorFunction&& visitor_func)
    {
        const size_t n = polygon.size();
        if (n < 3) return;

        // 1) World-space AABB
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const auto& p : polygon) 
        {
            min_x = std::min(min_x, p.x());
            max_x = std::max(max_x, p.x());
            min_y = std::min(min_y, p.y());
            max_y = std::max(max_y, p.y());
        }

        Eigen::Vector2d top_left_parent {min_x, max_y};
        size_t sub_w = static_cast<size_t>(std::ceil((max_x - min_x) / getResolution()) + 1);
        size_t sub_h = static_cast<size_t>(std::ceil((max_y - min_y) / getResolution()) + 1);

        //Iterate via SubMapIterator (top row -> bottom row, left -> right)
        this->SubMapIterator(top_left_parent, sub_w, sub_h, 
            [&](Eigen::Vector2i& pt)
            {
                //Return if outOfBound
                if (this->outOfBound(pt)) return;
                //Convert pt to world
                Eigen::Vector2d pt_world = this->gridToParent(pt);
                //Return if not in polygon
                if (!this->pointInPolygon(pt_world, polygon,true)) return;
                //Apply visitor only if in polygon and in bounds of the map
                visitor_func(pt);
            }
        );
    }

}