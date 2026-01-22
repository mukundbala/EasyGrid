# Easy Grid Documentation

## Overview

Easy Grid is a header-only C++ library for handling 2D grids with spatial awareness. It provides a templated `GridHandler` class that manages grid cells in a parent coordinate frame, with utilities for coordinate conversions, iteration patterns, and geometric queries.

**Key Features:**
- Template-based cell storage for flexible data structures
- Parent frame to grid coordinate conversions (SE2/SE3)
- Multiple iteration patterns (line, circle, polygon, submap)
- Geometric queries (point in polygon, bounds checking)
- No external dependencies except Eigen

## Core Components

### MetaData

Defines the spatial properties of a 2D grid relative to a parent coordinate frame.

**Members:**
- `double resolution` - Spatial dimension of each grid cell in meters (default: 0.1)
- `size_t map_width` - Number of cells along width (default: 100)
- `size_t map_height` - Number of cells along height (default: 100)
- `Eigen::Matrix4d map_frame_transform` - SE(3) transform from parent frame to grid origin (default: Identity)

**Methods:**

```cpp
void set_transform(Eigen::Vector3d &translation, Eigen::Quaterniond &rotation)
```
Sets the grid's pose in the parent frame. Normalizes quaternion if needed.

```cpp
Eigen::VectorXd get_transform_as_vec() const
```
Returns transform as 7D vector: [x, y, z, qx, qy, qz, qw]

```cpp
Eigen::Matrix4d get_transform_as_mat() const
```
Returns full 4x4 homogeneous transformation matrix.

```cpp
Eigen::Vector3d get_translation() const
```
Extracts translation component.

```cpp
Eigen::Matrix3d get_rotation_as_matrix() const
```
Extracts 3x3 rotation matrix.

```cpp
Eigen::Quaterniond get_rotation_as_quat() const
```
Returns rotation as quaternion.

---

### GridHandler<CellT>

Main grid management class templated on cell type.

**Template Parameter:**
- `CellT` - User-defined struct containing cell data. Must be default constructible.

## Construction and Assignment

```cpp
GridHandler()
```
Default constructor. Creates empty grid handler.

```cpp
explicit GridHandler(MetaData meta)
```
Constructs grid with specified metadata. Initializes all cells to default `CellT()`.

**Parameters:**
- `meta` - Grid metadata

```cpp
explicit GridHandler(MetaData meta, std::vector<CellT> &other_grid_cells)
```
Constructs grid with metadata and existing cells.

**Parameters:**
- `meta` - Grid metadata
- `other_grid_cells` - Vector of cells (moved, not copied)

**Throws:** `std::runtime_error` if `other_grid_cells.size() != meta.map_width * meta.map_height`

**Warning:** Invalidates `other_grid_cells` via move operation.

```cpp
GridHandler(GridHandler &&other_grid)
GridHandler& operator=(GridHandler &&other_grid)
```
Move constructor and assignment operator.

**Warning:** Invalidates `other_grid`. Do not reuse after move.

**Deleted Operations:**
- Copy constructor
- Copy assignment operator

---

## Grid Management

```cpp
void setGridCells(std::vector<CellT> &grid_cells)
```
Replaces grid cells with new data.

**Parameters:**
- `grid_cells` - New cell vector (moved)

**Throws:** `std::runtime_error` if size doesn't match metadata dimensions.

```cpp
void setMetaData(MetaData meta)
```
Updates metadata and reinitializes grid.

**Parameters:**
- `meta` - New metadata

**Warning:** Deletes all existing cells. Grid is rebuilt with default-initialized cells.

```cpp
GridHandler cloneGrid()
```
Creates deep copy of grid.

**Returns:** Independent copy of the grid.

```cpp
void clearGrid()
```
Clears all cells. Capacity remains allocated.

```cpp
void clearAndDeallocGrid()
```
Clears cells and deallocates memory.

---

## Getters

All getters are `noexcept` and `[[nodiscard]]`.

```cpp
double getResolution() const noexcept
```
Returns cell resolution in meters.

```cpp
size_t getWidth() const noexcept
```
Returns grid width in cells.

```cpp
size_t getHeight() const noexcept
```
Returns grid height in cells.

```cpp
Eigen::Matrix4d getMapTransform() const noexcept
```
Returns SE(3) transform of grid in parent frame.

```cpp
Eigen::Vector3d getMapTranslation() const noexcept
```
Returns translation component (z = 0).

```cpp
Eigen::Quaterniond getMapRotation() const noexcept
```
Returns rotation as quaternion.

```cpp
MetaData getMetaData() const noexcept
```
Returns metadata struct.

```cpp
size_t getTotalCells() const noexcept
```
Returns total number of cells.

```cpp
size_t getCapacity() const noexcept
```
Returns allocated capacity of underlying vector.

**Note:** Capacity >= TotalCells

---

## Coordinate Conversions

```cpp
Eigen::Vector2i parentToGrid(const Eigen::Vector2d &parent_coord) const noexcept
```
Converts parent frame coordinates to grid coordinates.

**Parameters:**
- `parent_coord` - 2D position in parent frame

**Returns:** Grid coordinates (u, v) where u increases right, v increases up, origin at bottom-left.

**Note:** Uses floor operation. Out-of-bounds coordinates may be returned.

```cpp
Eigen::Vector2d gridToParent(const Eigen::Vector2i &grid_coord) const noexcept
```
Converts grid coordinates to parent frame position at cell center.

**Parameters:**
- `grid_coord` - Grid coordinates

**Returns:** Parent frame position at center of cell.

```cpp
size_t gridToIndex(const Eigen::Vector2i &grid_coord) const noexcept
```
Converts grid coordinates to 1D array index.

**Parameters:**
- `grid_coord` - Grid coordinates

**Returns:** Index for accessing cell in internal vector.

**Formula:** `index = v * width + u`

```cpp
Eigen::Vector2i indexToGrid(size_t index) const noexcept
```
Converts 1D index to grid coordinates.

**Parameters:**
- `index` - Array index

**Returns:** Grid coordinates (u, v).

---

## Bounds Checking

```cpp
bool outOfBound(const Eigen::Vector2d &test_coord) const noexcept
bool outOfBound(const Eigen::Vector2i &test_coord) const noexcept
bool outOfBound(const size_t index) const noexcept
```
Checks if coordinate or index is outside grid boundaries.

**Parameters:**
- `test_coord` - Parent or grid coordinates
- `index` - Array index

**Returns:** `true` if out of bounds, `false` otherwise.

---

## Utility Functions

```cpp
template <typename ArithmeticT>
constexpr int whatsign(ArithmeticT value)
```
Determines sign of arithmetic value.

**Returns:** -1 if negative, 0 if zero, 1 if positive.

```cpp
template <typename ArithmeticT>
constexpr bool nearZero(ArithmeticT value, ArithmeticT eps = 1e-12) noexcept
```
Checks if value is near zero within epsilon.

**Parameters:**
- `value` - Value to test
- `eps` - Epsilon threshold (default: 1e-12)

**Returns:** `true` if `abs(value) <= eps`

---

## Geometric Queries

```cpp
bool pointInPolygon(Eigen::Vector2d& point, 
                    std::vector<Eigen::Vector2d>& polygon,
                    bool include_perimeter)
```
Tests if point lies inside polygon using even-odd ray casting rule.

**Parameters:**
- `point` - Point in parent coordinates
- `polygon` - Polygon vertices in parent coordinates (must have >= 3 vertices)
- `include_perimeter` - Whether to include points on polygon boundary

**Returns:** `true` if point is inside polygon (or on perimeter if `include_perimeter = true`)

**Note:** Returns `false` if polygon has fewer than 3 vertices.

```cpp
bool pointOnSegment(Eigen::Vector2d& p, Eigen::Vector2d& a, Eigen::Vector2d& b)
```
Tests if point lies on line segment.

**Parameters:**
- `p` - Point to test
- `a` - Segment start point
- `b` - Segment end point

**Returns:** `true` if point is on segment within numerical tolerance (1e-9).

---

## Neighbor Queries

```cpp
std::array<Eigen::Vector2i,4> getCardinalNeighbors(const Eigen::Vector2i &source_grid_coord)
```
Returns 4-connected neighbors: right, up, left, down.

**Parameters:**
- `source_grid_coord` - Source cell

**Returns:** Array of 4 neighbor coordinates.

**Warning:** No bounds checking performed. Use `outOfBound()` to validate.

```cpp
std::array<Eigen::Vector2i,8> getOctileNeighbors(const Eigen::Vector2i &source_grid_coord)
```
Returns 8-connected neighbors: cardinal + diagonal.

**Parameters:**
- `source_grid_coord` - Source cell

**Returns:** Array of 8 neighbor coordinates.

**Warning:** No bounds checking performed. Use `outOfBound()` to validate.

---

## Cell Access and Modification

```cpp
CellT& getCell(const size_t index)
CellT& getCell(const Eigen::Vector2i &grid_coord)
CellT& getCell(const Eigen::Vector2d &parent_coord)
```
Retrieves reference to cell for reading or modification.

**Parameters:**
- `index` - 1D array index
- `grid_coord` - Grid coordinates
- `parent_coord` - Parent frame coordinates

**Returns:** Mutable reference to cell.

**Throws:** Out-of-range exception if index/coordinates are invalid.

**Warning:** No bounds checking. Caller must validate coordinates.

```cpp
std::vector<CellT> getCells(const std::vector<size_t>& indices)
```
Retrieves multiple cells.

**Parameters:**
- `indices` - Vector of cell indices

**Returns:** Vector of cell copies.

**Throws:** Out-of-range exception if any index is invalid.

```cpp
void setCell(const size_t index, const CellT &cell)
void setCell(const Eigen::Vector2i &grid_coord, const CellT &cell)
void setCell(const Eigen::Vector2d &parent_coord, const CellT &cell)
```
Sets cell value.

**Parameters:**
- `index` - 1D array index
- `grid_coord` - Grid coordinates
- `parent_coord` - Parent frame coordinates
- `cell` - New cell value

**Throws:** Out-of-range exception if index/coordinates are invalid.

**Warning:** No bounds checking.

```cpp
void setCells(const std::vector<size_t>& indices, const std::vector<CellT> &values)
```
Sets multiple cells.

**Parameters:**
- `indices` - Vector of cell indices
- `values` - Vector of cell values

**Throws:** `std::runtime_error` if `indices.size() != values.size()`, or out-of-range if any index is invalid.

---

## Iterator Methods

All iterator methods accept a visitor function with signature: `void(Eigen::Vector2i& grid_coord)`.

### forEachCellDo

```cpp
template<class VisitorFunction>
void forEachCellDo(VisitorFunction&& visitor,
                   std::optional<std::vector<size_t>> selected_indices = std::nullopt)
```
Applies visitor to all cells or selected subset.

**Parameters:**
- `visitor` - Function called for each cell: `void(Eigen::Vector2i& grid_coord)`
- `selected_indices` - Optional list of specific indices to visit

**Warning:** Visitor may modify cells. No bounds checking on selected indices.

### RayIterator

```cpp
template <class VisitorFunction>
void RayIterator(const Eigen::Vector2d& origin_point,
                 const Eigen::Vector2d& end_point, 
                 bool include_end, 
                 VisitorFunction&& visitor_func)
```
Traces line between two points using Bresenham's algorithm.

**Parameters:**
- `origin_point` - Ray start in parent coordinates
- `end_point` - Ray end in parent coordinates
- `include_end` - Whether to visit end cell
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Automatically skips out-of-bounds cells
- If origin == end, visits only that cell (if in bounds)
- Always visits origin cell if in bounds

### SubMapIterator

```cpp
template <class VisitorFunction>
void SubMapIterator(Eigen::Vector2d& top_left_parent, 
                    size_t submap_width, 
                    size_t submap_height, 
                    VisitorFunction&& visitor_func)
```
Iterates over rectangular subregion.

**Parameters:**
- `top_left_parent` - Top-left corner in parent coordinates
- `submap_width` - Width in cells
- `submap_height` - Height in cells
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Iterates top-to-bottom, left-to-right
- Automatically skips out-of-bounds cells

**Warning:** Continues iteration even if top-left is out of bounds.

### CirclePerimeterIterator

```cpp
template <class VisitorFunction>
void CirclePerimeterIterator(Eigen::Vector2d& center_parent, 
                              double radius, 
                              VisitorFunction&& visitor_func)
```
Visits cells on circle boundary using Bresenham's circle algorithm.

**Parameters:**
- `center_parent` - Circle center in parent coordinates
- `radius` - Radius in meters
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Automatically deduplicates visited cells
- Skips out-of-bounds cells
- Returns immediately if `radius < 0`

### CircleAreaIterator

```cpp
template <class VisitorFunction>
void CircleAreaIterator(Eigen::Vector2d& center_parent, 
                        double radius, 
                        VisitorFunction&& visitor_func)
```
Visits all cells within circle (perimeter + interior).

**Parameters:**
- `center_parent` - Circle center in parent coordinates
- `radius` - Radius in meters
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Uses bounding box approach with distance filtering
- Automatically skips out-of-bounds cells
- Returns immediately if `radius < 0`

### PolygonPerimeterIterator

```cpp
template <class VisitorFunction>
void PolygonPerimeterIterator(std::vector<Eigen::Vector2d>& polygon, 
                               VisitorFunction&& visitor_func)
```
Traces polygon edges using line iteration between vertices.

**Parameters:**
- `polygon` - Polygon vertices in parent coordinates (ordered clockwise recommended)
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Assumes edges exist between consecutive vertices and from last to first
- Automatically deduplicates vertices
- Skips out-of-bounds cells
- Returns immediately if fewer than 3 vertices

**Warning:** End points of each edge are not included to avoid double-visiting vertices.

### PolygonAreaIterator

```cpp
template <class VisitorFunction>
void PolygonAreaIterator(std::vector<Eigen::Vector2d>& polygon, 
                         VisitorFunction&& visitor_func)
```
Visits all cells within polygon boundary (perimeter + interior).

**Parameters:**
- `polygon` - Polygon vertices in parent coordinates
- `visitor_func` - Function: `void(Eigen::Vector2i& grid_coord)`

**Behavior:**
- Computes axis-aligned bounding box
- Tests each cell in bbox using `pointInPolygon()`
- Includes perimeter cells
- Skips out-of-bounds cells
- Returns immediately if fewer than 3 vertices

---

## Usage Example

```cpp
// Define custom cell type
struct OccupancyCell {
    float probability = 0.5f;
    bool visited = false;
};

// Create metadata
easy_grid::MetaData meta;
meta.resolution = 0.05;  // 5cm cells
meta.map_width = 200;
meta.map_height = 200;

// Create grid
easy_grid::GridHandler<OccupancyCell> grid(meta);

// Set a cell
Eigen::Vector2d world_pos(1.0, 2.0);
OccupancyCell cell;
cell.probability = 0.9f;
grid.setCell(world_pos, cell);

// Ray tracing example
Eigen::Vector2d start(0.0, 0.0);
Eigen::Vector2d end(5.0, 5.0);
grid.RayIterator(start, end, true, 
    [&](Eigen::Vector2i& coord) {
        auto& cell = grid.getCell(coord);
        cell.visited = true;
    }
);

// Circle clearing example
Eigen::Vector2d robot_pos(2.5, 2.5);
grid.CircleAreaIterator(robot_pos, 1.0,
    [&](Eigen::Vector2i& coord) {
        grid.getCell(coord).probability = 0.0f;
    }
);
```

---

## Important Notes

### Memory Management
- Grid cells stored in contiguous `std::vector`
- Move semantics used throughout to avoid unnecessary copies
- Copy construction and assignment explicitly deleted

### Coordinate Systems
- Grid origin at bottom-left
- Grid u-axis increases right, v-axis increases up
- Vertical translation of grid in parent frame is always 0
- All conversions handle SE(2)/SE(3) transformations

### Thread Safety
- Not thread-safe
- Caller must synchronize concurrent access

### Performance Considerations
- Coordinate conversions involve matrix operations
- Iterator methods may visit many cells; consider performance impact
- No bounds checking in `getCell()`/`setCell()` for performance; validate beforehand

### Numerical Precision
- Epsilon values used for floating-point comparisons
- Default epsilon: 1e-12 for `nearZero()`, 1e-9 for geometry tests
- Adjust if working at very small or large scales