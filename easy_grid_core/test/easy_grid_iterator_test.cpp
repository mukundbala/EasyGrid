/**
 * @file easy_grid_iterator_test.cpp
 * @brief Unit tests for GridHandler iterator methods
 * 
 * Tests covered:
 * - forEachCellDo
 * - RayIterator
 * - SubMapIterator
 * - CirclePerimeterIterator
 * - CircleAreaIterator
 */

#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>
#include <set>
#include <vector>
#include <cmath>

struct TestCell {
    int value = 0;
    bool visited = false;
};

class GridHandlerIteratorTest : public ::testing::Test {
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
// forEachCellDo Tests
// ============================================================================

TEST_F(GridHandlerIteratorTest, ForEachCellDo_VisitsAllCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    size_t count = 0;
    grid.forEachCellDo([&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 100);
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_VisitsEachCellOnce) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::set<size_t> visited_indices;
    grid.forEachCellDo([&visited_indices, &grid](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        visited_indices.insert(idx);
    });
    
    EXPECT_EQ(visited_indices.size(), 100);
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_CanModifyCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.forEachCellDo([&grid](Eigen::Vector2i& coord) {
        grid.getCell(coord).value = coord.x() + coord.y();
        grid.getCell(coord).visited = true;
    });
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 7);
    EXPECT_TRUE(grid.getCell(Eigen::Vector2i(5, 5)).visited);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(0, 0)).value, 0);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(9, 9)).value, 18);
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_SelectedIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> selected = {0, 10, 20, 50, 99};
    size_t count = 0;
    
    grid.forEachCellDo([&count](Eigen::Vector2i& coord) {
        ++count;
    }, selected);
    
    EXPECT_EQ(count, 5);
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_SelectedIndices_CorrectCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> selected = {0, 55, 99};
    std::set<size_t> visited;
    
    grid.forEachCellDo([&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    }, selected);
    
    EXPECT_EQ(visited.size(), 3);
    EXPECT_TRUE(visited.count(0));
    EXPECT_TRUE(visited.count(55));
    EXPECT_TRUE(visited.count(99));
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_EmptySelectedIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> selected;
    size_t count = 0;
    
    grid.forEachCellDo([&count](Eigen::Vector2i& coord) {
        ++count;
    }, selected);
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, ForEachCellDo_NulloptVisitsAll) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    size_t count = 0;
    grid.forEachCellDo([&count](Eigen::Vector2i& coord) {
        ++count;
    }, std::nullopt);
    
    EXPECT_EQ(count, 100);
}

// ============================================================================
// RayIterator Tests
// ============================================================================

TEST_F(GridHandlerIteratorTest, RayIterator_HorizontalLine) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.95, 0.05);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.size(), 10);
    for (const auto& v : visited) {
        EXPECT_EQ(v.y(), 0);
    }
}

TEST_F(GridHandlerIteratorTest, RayIterator_VerticalLine) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.05, 0.95);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.size(), 10);
    for (const auto& v : visited) {
        EXPECT_EQ(v.x(), 0);
    }
}

TEST_F(GridHandlerIteratorTest, RayIterator_DiagonalLine) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.95, 0.95);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_GE(visited.size(), 10);
    EXPECT_EQ(visited.front(), Eigen::Vector2i(0, 0));
    EXPECT_EQ(visited.back(), Eigen::Vector2i(9, 9));
}

TEST_F(GridHandlerIteratorTest, RayIterator_IncludeEnd_True) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.35, 0.05);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.back(), Eigen::Vector2i(3, 0));
}

TEST_F(GridHandlerIteratorTest, RayIterator_IncludeEnd_False) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.35, 0.05);
    
    std::vector<Eigen::Vector2i> with_end, without_end;
    
    grid.RayIterator(start, end, true, [&with_end](Eigen::Vector2i& coord) {
        with_end.push_back(coord);
    });
    
    grid.RayIterator(start, end, false, [&without_end](Eigen::Vector2i& coord) {
        without_end.push_back(coord);
    });
    
    EXPECT_EQ(with_end.size(), without_end.size() + 1);
}

TEST_F(GridHandlerIteratorTest, RayIterator_SameStartAndEnd) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d point(0.55, 0.55);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(point, point, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.size(), 1);
    EXPECT_EQ(visited[0], Eigen::Vector2i(5, 5));
}

TEST_F(GridHandlerIteratorTest, RayIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Ray that starts out of bounds
    Eigen::Vector2d start(-0.5, 0.05);
    Eigen::Vector2d end(0.55, 0.05);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    // Should only include in-bounds cells
    for (const auto& v : visited) {
        EXPECT_FALSE(grid.outOfBound(v));
    }
}

TEST_F(GridHandlerIteratorTest, RayIterator_ReversedDirection) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.95, 0.05);
    Eigen::Vector2d end(0.05, 0.05);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.size(), 10);
    EXPECT_EQ(visited.front(), Eigen::Vector2i(9, 0));
    EXPECT_EQ(visited.back(), Eigen::Vector2i(0, 0));
}

TEST_F(GridHandlerIteratorTest, RayIterator_SteepLine) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // More vertical than horizontal
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.25, 0.95);
    
    std::vector<Eigen::Vector2i> visited;
    grid.RayIterator(start, end, true, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.front(), Eigen::Vector2i(0, 0));
    EXPECT_EQ(visited.back(), Eigen::Vector2i(2, 9));
}

TEST_F(GridHandlerIteratorTest, RayIterator_NoDuplicates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d start(0.05, 0.05);
    Eigen::Vector2d end(0.95, 0.75);
    
    std::vector<size_t> all_indices;
    std::set<size_t> unique_indices;
    
    grid.RayIterator(start, end, true, [&](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        all_indices.push_back(idx);
        unique_indices.insert(idx);
    });
    
    EXPECT_EQ(all_indices.size(), unique_indices.size());
}

// ============================================================================
// SubMapIterator Tests
// ============================================================================

TEST_F(GridHandlerIteratorTest, SubMapIterator_FullGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(0.0, 0.95);
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 10, 10, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 100);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_PartialGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(0.25, 0.75);  // Cell (2, 7)
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 5, 5, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 25);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_SingleCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(0.55, 0.55);
    
    std::vector<Eigen::Vector2i> visited;
    grid.SubMapIterator(top_left, 1, 1, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    EXPECT_EQ(visited.size(), 1);
    EXPECT_EQ(visited[0], Eigen::Vector2i(5, 5));
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Top-left near corner, submap extends beyond grid
    Eigen::Vector2d top_left(0.75, 0.95);
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 5, 5, [&count, &grid](Eigen::Vector2i& coord) {
        EXPECT_FALSE(grid.outOfBound(coord));
        ++count;
    });
    
    EXPECT_LT(count, 25);
    EXPECT_GT(count, 0);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_CompletelyOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(2.0, 2.0);  // Completely outside
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 5, 5, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_ZeroWidth) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(0.5, 0.5);
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 0, 5, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_ZeroHeight) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d top_left(0.5, 0.5);
    
    size_t count = 0;
    grid.SubMapIterator(top_left, 5, 0, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, SubMapIterator_CorrectCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Top-left at cell (2, 5), 3x3 submap
    Eigen::Vector2d top_left(0.25, 0.55);
    
    std::set<std::pair<int, int>> visited;
    grid.SubMapIterator(top_left, 3, 3, [&visited](Eigen::Vector2i& coord) {
        visited.insert({coord.x(), coord.y()});
    });
    
    // Should visit cells (2,5), (3,5), (4,5), (2,4), (3,4), (4,4), (2,3), (3,3), (4,3)
    EXPECT_EQ(visited.size(), 9);
    EXPECT_TRUE(visited.count({2, 5}));
    EXPECT_TRUE(visited.count({4, 3}));
}

// ============================================================================
// CirclePerimeterIterator Tests
// ============================================================================

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_SmallCircle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.2;
    
    std::set<size_t> visited;
    grid.CirclePerimeterIterator(center, radius, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_NoDuplicates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.3;
    
    std::vector<size_t> all_visited;
    std::set<size_t> unique_visited;
    
    grid.CirclePerimeterIterator(center, radius, [&](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        all_visited.push_back(idx);
        unique_visited.insert(idx);
    });
    
    EXPECT_EQ(all_visited.size(), unique_visited.size());
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_CellsAtCorrectDistance) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.3;
    
    Eigen::Vector2i center_grid = grid.parentToGrid(center);
    int radius_cells = static_cast<int>(std::round(radius / grid.getResolution()));
    
    grid.CirclePerimeterIterator(center, radius, [&](Eigen::Vector2i& coord) {
        double dist = (coord - center_grid).cast<double>().norm();
        EXPECT_NEAR(dist, radius_cells, 1.5);  // Allow some tolerance
    });
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_NegativeRadius) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = -1.0;
    
    size_t count = 0;
    grid.CirclePerimeterIterator(center, radius, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_ZeroRadius) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.0;
    
    std::vector<Eigen::Vector2i> visited;
    grid.CirclePerimeterIterator(center, radius, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    // Zero radius should visit just the center cell
    EXPECT_EQ(visited.size(), 1);
    EXPECT_EQ(visited[0], Eigen::Vector2i(5, 5));
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Center near edge
    Eigen::Vector2d center(0.95, 0.5);
    double radius = 0.3;
    
    grid.CirclePerimeterIterator(center, radius, [&grid](Eigen::Vector2i& coord) {
        EXPECT_FALSE(grid.outOfBound(coord));
    });
}

TEST_F(GridHandlerIteratorTest, CirclePerimeterIterator_LargeCircle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.4;  // Large relative to grid
    
    std::set<size_t> visited;
    grid.CirclePerimeterIterator(center, radius, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 10);
}

// ============================================================================
// CircleAreaIterator Tests
// ============================================================================

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_SmallCircle) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.2;
    
    std::set<size_t> visited;
    grid.CircleAreaIterator(center, radius, [&visited, &grid](Eigen::Vector2i& coord) {
        visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(visited.size(), 0);
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_CellsWithinRadius) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.25;
    
    Eigen::Vector2i center_grid = grid.parentToGrid(center);
    int radius_cells = static_cast<int>(std::round(radius / grid.getResolution()));
    int radius_sq = radius_cells * radius_cells;
    
    grid.CircleAreaIterator(center, radius, [&](Eigen::Vector2i& coord) {
        Eigen::Vector2i diff = coord - center_grid;
        int dist_sq = diff.x() * diff.x() + diff.y() * diff.y();
        EXPECT_LE(dist_sq, radius_sq);
    });
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_AreaGreaterThanPerimeter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.3;
    
    std::set<size_t> area_visited;
    std::set<size_t> perimeter_visited;
    
    grid.CircleAreaIterator(center, radius, [&area_visited, &grid](Eigen::Vector2i& coord) {
        area_visited.insert(grid.gridToIndex(coord));
    });
    
    grid.CirclePerimeterIterator(center, radius, [&perimeter_visited, &grid](Eigen::Vector2i& coord) {
        perimeter_visited.insert(grid.gridToIndex(coord));
    });
    
    EXPECT_GT(area_visited.size(), perimeter_visited.size());
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_NegativeRadius) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = -1.0;
    
    size_t count = 0;
    grid.CircleAreaIterator(center, radius, [&count](Eigen::Vector2i& coord) {
        ++count;
    });
    
    EXPECT_EQ(count, 0);
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_ZeroRadius) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.0;
    
    std::vector<Eigen::Vector2i> visited;
    grid.CircleAreaIterator(center, radius, [&visited](Eigen::Vector2i& coord) {
        visited.push_back(coord);
    });
    
    // Zero radius should visit just the center
    EXPECT_EQ(visited.size(), 1);
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_SkipsOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.1, 0.1);
    double radius = 0.3;
    
    grid.CircleAreaIterator(center, radius, [&grid](Eigen::Vector2i& coord) {
        EXPECT_FALSE(grid.outOfBound(coord));
    });
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_NoDuplicates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.5, 0.5);
    double radius = 0.35;
    
    std::vector<size_t> all_visited;
    std::set<size_t> unique_visited;
    
    grid.CircleAreaIterator(center, radius, [&](Eigen::Vector2i& coord) {
        size_t idx = grid.gridToIndex(coord);
        all_visited.push_back(idx);
        unique_visited.insert(idx);
    });
    
    EXPECT_EQ(all_visited.size(), unique_visited.size());
}

TEST_F(GridHandlerIteratorTest, CircleAreaIterator_IncludesCenter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2d center(0.55, 0.55);
    double radius = 0.2;
    
    Eigen::Vector2i center_grid = grid.parentToGrid(center);
    bool center_visited = false;
    
    grid.CircleAreaIterator(center, radius, [&](Eigen::Vector2i& coord) {
        if (coord == center_grid) {
            center_visited = true;
        }
    });
    
    EXPECT_TRUE(center_visited);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}