#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <vector>
#include <array>
#include <optional>
#include <unordered_set>

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
        Eigen::Matrix3d map_frame_transform = Eigen::Matrix3d::Identity(); //wTm
    };

    template <class CellT>
    class GridHandler
    {
    public:
        /**
         * @brief Default Constructor
         */
        GridHandler() = default;

        /**
         * @brief Constructor that initializes a default initialized grid cells based on the metadata
         * @param MetaData Metadata object containing metadata information of the map
         */
        explicit GridHandler(MetaData meta);

        /**
         * @brief Constructor with metadata and cells
         * @param MetaData MetaData Object
         * @param std::vector<CellT> cells
         * @see easy_grid::MetaData 
         */
        explicit GridHandler(MetaData meta, std::vector<CellT> &other_grid_cells);

        /**
         * @brief Default dtor
         * @test ok
         */
        ~GridHandler() = default; //tested

        /**
         * @brief Copy Constructor (Deleted!)
         * @remarks No copying allowed.
         * @test ok
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

        ///////////////////////////////////////////////////////////////
        ////////////////Public Datastructure Setters///////////////////
        ///////////////////////////////////////////////////////////////
        /**
         * @brief Populate the grid handler with another set of grid_cells
         * @param std::vector<CellT> &cells
         * @warning If the incoming grid_cells.size() != TOTAL_CELLS = MetaData::map_width * MetaData::map_height cells, this will throw
         */
        void setGridCells(std::vector<CellT> &grid_cells);

        /**
         * @brief Set the metadata. This will erase existing grid_cells and default initialise the cells based on the new metadata
         * @param MetaData meta
         */
        void setMetaData(MetaData meta);

        /**
         * @brief Creates a copy of an existing grid and returns the grid
         * @param GridHandler& other_grid The grid to copy
         * @return GridHandler Copy of other_grid
         */
        GridHandler cloneGrid(GridHandler &other_grid);

        ///////////////////////////////////////////////////////////////
        ////////////////Public Datastructure Clears////////////////////
        ///////////////////////////////////////////////////////////////
        
        /**
         * @brief Clears the grid, however space remains allocated
         */
        void clearGrid();
        
        /**
         * @brief Clears the grid and deallocates space
         */
        void clearAndDeallocGrid();

        ///////////////////////////////////////////////////////////////
        ////////////////PublicGetter///////////////////////////////////
        ///////////////////////////////////////////////////////////////

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
         * @brief Get the SE2 transform of the grid in parent frame
         * @return Eigen::Matrix3d SE2 transformation
         * @see easy_grid::MetaData
         */
        [[nodiscard]] Eigen::Matrix3d getMapTransform() const noexcept;

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

        /////////////////////////////////////////////////////////
        ///////////////////////////Conversions///////////////////
        /////////////////////////////////////////////////////////

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

        /////////////////////////////////////////////////////////////////////////
        ///////////////////////////////Checks////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////

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

        /////////////////////////////////////////////////////////////////////////
        ///////////////////////////////Neighborhood//////////////////////////////
        /////////////////////////////////////////////////////////////////////////

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

        //////////////////////////////////////////////////////////////////////////
        /////////////////////////Cell Accessor and Modifiers//////////////////////
        //////////////////////////////////////////////////////////////////////////

        /**
         * @brief Access the grid as a Cell<T>&
         * @param size_t index
         * @return CellT&
         * @warning No bounds checking, use with caution
         * @warning Cell will be mutable
         */
        CellT& getCell(size_t index);
        
        /**
         * @brief Access the grid as a Cell<T>&
         * @param Eigen::Vector2i grid_coord
         * @return CellT&
         * @warning No bounds checking, use with caution
         * @warning Cell will be mutable
         */
        CellT& getCell(Eigen::Vector2i &grid_coord);

        /**
         * @brief Access the grid as a Cell<T>&
         * @param world coordinates
         * @return CellT&
         * @warning No bounds checking, use with caution
         * @warning Cell will be mutable
         */
        CellT& getCell(Eigen::Vector2d &world_coord);

        /**
         * @brief Get values of cells.
         * @param indices Indices of cells to query.
         * @return vector of cell values.
         * @warning No bounds checking
         */
        std::vector<CellT> getCells(const std::vector<size_t>& indices);

        /**
         * @brief Set the value of a cell
         * @param size_t index
         * @param cell
         */
        void setCell(size_t index, const CellT &cell);

        /**
         * @brief Set the value of a cell
         * @param Eigen::Vector2i grid_coord
         * @param cell
         */
        void setCell(Eigen::Vector2i &grid_coord, const CellT &cell);

        /**
         * @brief Set the value of a cell
         * @param Eigen::Vector2d world_coord 
         * @param cell
         */
        void setCell(Eigen::Vector2d &world_coord, const CellT &cell);

        /**
         * @brief Set cells with values.
         * @param indices Indices of cells to set.
         * @param values Values to set, must be same size as indices.
         */
        void setCells(const std::vector<size_t>& indices, const std::vector<CellT> values);

        //////////////////////////////////////////////////////////////////////////
        /////////////////////////Iterators////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////
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
        * @param bool include_end includes the destination/end point if set to true, otherwise does not
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
         */
        template <class VisitorFunction>
        void SubMapIterator(Eigen::Vector2d& top_left_world , size_t submap_width, size_t submap_height, VisitorFunction&& visitor_func);
        
        /**
        * @brief Circle Iterator for the grid that applies VisitorFunction to the points in circle perimeter given a center
        * @param Eigen::Vector2d& key_origin The origin of the ray in world coordinates
        * @param Eigen::Vector2d& key_end The end of the ray in world coordinates
        * @param VisitorFunction&& func with param Eigen::Vector2i &grid_coordc  
        */
        template <class VisitorFunction>
        void CirclePerimeterIterator(Eigen::Vector2d& center_world, double radius, VisitorFunction&& visitor_func);
        
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
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!easy_grid::GridHandler IMPLEMENTATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */

    

}