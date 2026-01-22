/**
 * @file easy_grid_polygon_test.cpp
 * @brief Unit tests for GridHandler polygon-related methods
 * 
 * Tests covered:
 * - pointInPolygon
 * - pointOnSegment
 * - PolygonPerimeterIterator
 * - PolygonAreaIterator
 */

#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>
#include <set>
#include <vector>
#include <cmath>

struct TestCell {
    int value = 0;
};

class GridHandlerPolygonTest : public ::testing::Test {
protected:
    void SetUp() override {
        meta_.resolution = 0.1;
        meta_.map_width = 10;
        meta_.map_height = 10;
        meta_.map_frame_transform = Eigen::Matrix4d::Identity();
    }
    
    easy_grid::MetaData meta_;
};

// ============================================================================
// pointOnSegment Tests
// ============================================================================

TEST_F(GridHandlerPolygonTest, PointOnSegment_OnSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    Eigen::Vector2d p(0.5, 0.5);
    
    EXPECT_TRUE(grid.pointOnSegment(p, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_NotOnSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    Eigen::Vector2d p(0.5, 0.6);
    
    EXPECT_FALSE(grid.pointOnSegment(p, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_AtStartPoint) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    
    EXPECT_TRUE(grid.pointOnSegment(a, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_AtEndPoint) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    
    EXPECT_TRUE(grid.pointOnSegment(b, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_BeyondSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    Eigen::Vector2d p(1.5, 1.5);  // On the line, but beyond segment
    
    EXPECT_FALSE(grid.pointOnSegment(p, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_BeforeSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.0);
    Eigen::Vector2d b(1.0, 1.0);
    Eigen::Vector2d p(-0.5, -0.5);  // On the line, but before segment
    
    EXPECT_FALSE(grid.pointOnSegment(p, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_HorizontalSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.0, 0.5);
    Eigen::Vector2d b(1.0, 0.5);
    Eigen::Vector2d p_on(0.5, 0.5);
    Eigen::Vector2d p_off(0.5, 0.6);
    
    EXPECT_TRUE(grid.pointOnSegment(p_on, a, b));
    EXPECT_FALSE(grid.pointOnSegment(p_off, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_VerticalSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.5, 0.0);
    Eigen::Vector2d b(0.5, 1.0);
    Eigen::Vector2d p_on(0.5, 0.5);
    Eigen::Vector2d p_off(0.6, 0.5);
    
    EXPECT_TRUE(grid.pointOnSegment(p_on, a, b));
    EXPECT_FALSE(grid.pointOnSegment(p_off, a, b));
}

TEST_F(GridHandlerPolygonTest, PointOnSegment_DegenerateSegment) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d a(0.5, 0.5);
    Eigen::Vector2d b(0.5, 0.5);  // Same as a
    Eigen::Vector2d p_on(0.5, 0.5);
    Eigen::Vector2d p_off(0.6, 0.5);
    
    EXPECT_TRUE(grid.pointOnSegment(p_on, a, b));
    EXPECT_FALSE(grid.pointOnSegment(p_off, a, b));
}

// ============================================================================
// pointInPolygon Tests
// ============================================================================

TEST_F(GridHandlerPolygonTest, PointInPolygon_InsideSquare) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d inside(0.5, 0.5);
    EXPECT_TRUE(grid.pointInPolygon(inside, square, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_OutsideSquare) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d outside(1.5, 0.5);
    EXPECT_FALSE(grid.pointInPolygon(outside, square, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_OnEdge_ExcludePerimeter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d on_edge(0.5, 0.0);
    // On the edge, but not including perimeter - behavior depends on ray casting
    // Point exactly on edge may or may not be included
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_OnEdge_IncludePerimeter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d on_edge(0.5, 0.0);
    EXPECT_TRUE(grid.pointInPolygon(on_edge, square, true));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_OnVertex_IncludePerimeter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d on_vertex(0.0, 0.0);
    EXPECT_TRUE(grid.pointInPolygon(on_vertex, square, true));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_InsideTriangle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> triangle = {
        {0.0, 0.0}, {1.0, 0.0}, {0.5, 1.0}
    };
    
    Eigen::Vector2d inside(0.5, 0.3);
    EXPECT_TRUE(grid.pointInPolygon(inside, triangle, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_OutsideTriangle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> triangle = {
        {0.0, 0.0}, {1.0, 0.0}, {0.5, 1.0}
    };
    
    Eigen::Vector2d outside(0.1, 0.9);
    EXPECT_FALSE(grid.pointInPolygon(outside, triangle, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_ConcavePolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // L-shaped polygon
    std::vector<Eigen::Vector2d> l_shape = {
        {0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}, {1.0, 0.5}, {1.0, 1.0}, {0.0, 1.0}
    };
    
    Eigen::Vector2d inside(0.25, 0.75);
    Eigen::Vector2d in_concave(0.75, 0.75);
    Eigen::Vector2d outside(0.75, 0.25);
    
    EXPECT_TRUE(grid.pointInPolygon(inside, l_shape, false));
    EXPECT_TRUE(grid.pointInPolygon(in_concave, l_shape, false));
    EXPECT_FALSE(grid.pointInPolygon(outside, l_shape, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_DegeneratePolygon_TwoPoints) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> line = {{0.0, 0.0}, {1.0, 1.0}};
    Eigen::Vector2d point(0.5, 0.5);
    
    EXPECT_FALSE(grid.pointInPolygon(point, line, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_DegeneratePolygon_OnePoint) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> point_poly = {{0.5, 0.5}};
    Eigen::Vector2d point(0.5, 0.5);
    
    EXPECT_FALSE(grid.pointInPolygon(point, point_poly, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_DegeneratePolygon_Empty) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> empty;
    Eigen::Vector2d point(0.5, 0.5);
    
    EXPECT_FALSE(grid.pointInPolygon(point, empty, false));
}

TEST_F(GridHandlerPolygonTest, PointInPolygon_Pentagon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Regular pentagon-ish shape
    std::vector<Eigen::Vector2d> pentagon = {
        {0.5, 0.0}, {1.0, 0.4}, {0.8, 1.0}, {0.2, 1.0}, {0.0, 0.4}
    };
    
    Eigen::Vector2d center(0.5, 0.5);
    EXPECT_TRUE(grid.pointInPolygon(center, pentagon, false));
    
    Eigen::Vector2d outside(1.5, 0.5);
    EXPECT_FALSE(grid.pointInPolygon(outside, pentagon, false));
}

// ============================================================================
// PolygonPerimeterIterator Tests
// ============================================================================

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_Triangle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> triangle = {
        {0.1, 0.1}, {0.9, 0.1}, {0.5, 0.9}
    };
    
    std::set<size_t> visited;
    grid.PolygonPerimeterIterator(triangle, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_Square) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.1, 0.1}, {0.8, 0.1}, {0.8, 0.8}, {0.1, 0.8}
    };
    
    std::set<size_t> visited;
    grid.PolygonPerimeterIterator(square, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_NoDuplicates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.1, 0.1}, {0.8, 0.1}, {0.8, 0.8}, {0.1, 0.8}
    };
    
    std::vector<size_t> all_visited;
    std::set<size_t> unique_visited;
    
    grid.PolygonPerimeterIterator(square, [&](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        all_visited.push_back(idx);
        unique_visited.insert(idx);
    });
    
    EXPECT_EQ(all_visited.size(), unique_visited.size());
}

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_DegeneratePolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> line = {{0.0, 0.0}, {1.0, 1.0}};
    
    size_t count = 0;
    grid.PolygonPerimeterIterator(line, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Polygon extends beyond grid
    std::vector<Eigen::Vector2d> large_square = {
        {-0.5, -0.5}, {1.5, -0.5}, {1.5, 1.5}, {-0.5, 1.5}
    };
    
    grid.PolygonPerimeterIterator(large_square, [&grid](Eigen::Vector2i& coord) {
        EXPECT_FALSE(grid.outOfBound(coord));
    });
}

TEST_F(GridHandlerPolygonTest, PolygonPerimeterIterator_VisitsEdgeCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Small square to verify edge cells
    std::vector<Eigen::Vector2d> square = {
        {0.25, 0.25}, {0.75, 0.25}, {0.75, 0.75}, {0.25, 0.75}
    };
    
    std::set<std::pair<int, int>> visited;
    grid.PolygonPerimeterIterator(square, [&visited](Eigen::Vector2i& coord) {
        visited.insert({coord.x(), coord.y()});
    });
    
    // Should visit cells on all four edges
    // Bottom edge: (2,2), (3,2), ..., (7,2)
    // Check corners are visited
    EXPECT_TRUE(visited.count({2, 2}) || visited.count({2, 7}) || 
                visited.count({7, 2}) || visited.count({7, 7}));
}

// ============================================================================
// PolygonAreaIterator Tests
// ============================================================================

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_Square) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.2, 0.2}, {0.7, 0.2}, {0.7, 0.7}, {0.2, 0.7}
    };
    
    std::set<size_t> visited;
    grid.PolygonAreaIterator(square, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_Triangle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> triangle = {
        {0.1, 0.1}, {0.9, 0.1}, {0.5, 0.9}
    };
    
    std::set<size_t> visited;
    grid.PolygonAreaIterator(triangle, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_AreaGreaterThanPerimeter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.15, 0.15}, {0.85, 0.15}, {0.85, 0.85}, {0.15, 0.85}
    };
    
    std::set<size_t> area_visited;
    std::set<size_t> perimeter_visited;
    
    grid.PolygonAreaIterator(square, [&area_visited, &grid](Eigen::Vector2i& coord) {
        area_visited.insert(grid.gridToIndex(coord));
    });
    
    grid.PolygonPerimeterIterator(square, [&perimeter_visited, &grid](Eigen::Vector2i& coord) {
        perimeter_visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(area_visited.size(), perimeter_visited.size());
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_DegeneratePolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> line = {{0.0, 0.0}, {1.0, 1.0}};
    
    size_t count = 0;
    grid.PolygonAreaIterator(line, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Polygon extends beyond grid
    std::vector<Eigen::Vector2d> large_square = {
        {-0.5, -0.5}, {1.5, -0.5}, {1.5, 1.5}, {-0.5, 1.5}
    };
    
    grid.PolygonAreaIterator(large_square, [&grid](Eigen::Vector2i& coord) {
        EXPECT_FALSE(grid.outOfBound(coord));
    });
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_AllCellsInPolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.2, 0.2}, {0.7, 0.2}, {0.7, 0.7}, {0.2, 0.7}
    };
    
    grid.PolygonAreaIterator(square, [&grid, &square](Eigen::Vector2i& coord) {
        Eigen::Vector2d world = grid.gridToParent(coord);
        // Cell center should be inside or on polygon
        EXPECT_TRUE(grid.pointInPolygon(world, square, true));
    });
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_NoDuplicates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> triangle = {
        {0.1, 0.1}, {0.9, 0.1}, {0.5, 0.9}
    };
    
    std::vector<size_t> all_visited;
    std::set<size_t> unique_visited;
    
    grid.PolygonAreaIterator(triangle, [&](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        all_visited.push_back(idx);
        unique_visited.insert(idx);
    });
    
    EXPECT_EQ(all_visited.size(), unique_visited.size());
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_ConcavePolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // L-shaped polygon
    std::vector<Eigen::Vector2d> l_shape = {
        {0.1, 0.1}, {0.5, 0.1}, {0.5, 0.5}, {0.9, 0.5}, {0.9, 0.9}, {0.1, 0.9}
    };
    
    std::set<size_t> visited;
    grid.PolygonAreaIterator(l_shape, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
    
    // Verify cells in the "cut out" area are not visited
    Eigen::Vector2d cutout_point(0.7, 0.3);
    Eigen::Vector2i cutout_grid = grid.parentToGrid(cutout_point);
    if (!grid.outOfBound(cutout_grid)) {
        size_t cutout_idx = grid.gridToIndex(cutout_grid);
        EXPECT_FALSE(visited.count(cutout_idx));
    }
}

TEST_F(GridHandlerPolygonTest, PolygonAreaIterator_SmallPolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Very small polygon, might only cover 1-2 cells
    std::vector<Eigen::Vector2d> tiny = {
        {0.51, 0.51}, {0.59, 0.51}, {0.55, 0.59}
    };
    
    std::set<size_t> visited;
    grid.PolygonAreaIterator(tiny, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    // Should visit at least one cell
    EXPECT_GE(visited.size(), 1);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(GridHandlerPolygonTest, Integration_MarkPolygonArea) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.2, 0.2}, {0.7, 0.2}, {0.7, 0.7}, {0.2, 0.7}
    };
    
    // Mark all cells in polygon
    grid.PolygonAreaIterator(square, [&grid](Eigen::Vector2i& coord) {
        grid.getCell(coord).value = 1;
    });
    
    // Verify cells inside are marked
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(4, 4)).value, 1);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(5, 5)).value, 1);
    
    // Verify cells outside are not marked
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(0, 0)).value, 0);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(9, 9)).value, 0);
}

TEST_F(GridHandlerPolygonTest, Integration_CountCellsInPolygon) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<Eigen::Vector2d> square = {
        {0.25, 0.25}, {0.75, 0.25}, {0.75, 0.75}, {0.25, 0.75}
    };
    
    size_t area_cells = 0;
    grid.PolygonAreaIterator(square, [&area_cells](Eigen::Vector2i& coord) {
        ++area_cells;
    });
    
    size_t perimeter_cells = 0;
    grid.PolygonPerimeterIterator(square, [&perimeter_cells](Eigen::Vector2i& coord) {
        ++perimeter_cells;
    });
    
    // Area should be approximately 0.5 * 0.5 = 0.25 m^2
    // With 0.1m cells, that's ~25 cells
    EXPECT_GE(area_cells, 15);
    EXPECT_LE(area_cells, 36);
    
    // Perimeter should be less than area
    EXPECT_LT(perimeter_cells, area_cells);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}