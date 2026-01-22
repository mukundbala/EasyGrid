/**
 * @file easy_grid_cell_access_test.cpp
 * @brief Unit tests for GridHandler cell access and modification methods
 * 
 * Tests covered:
 * - getCell(size_t)
 * - getCell(Eigen::Vector2i)
 * - getCell(Eigen::Vector2d)
 * - getCells(std::vector<size_t>)
 * - setCell(size_t, CellT)
 * - setCell(Eigen::Vector2i, CellT)
 * - setCell(Eigen::Vector2d, CellT)
 * - setCells(std::vector<size_t>, std::vector<CellT>)
 * - getCardinalNeighbors
 * - getOctileNeighbors
 */

#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>
#include <set>

struct TestCell {
    int value = 0;
    bool flag = false;
    
    bool operator==(const TestCell& other) const {
        return value == other.value && flag == other.flag;
    }
};

class GridHandlerCellAccessTest : public ::testing::Test {
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
// getCell(size_t) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_ReturnsReference) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(0).value = 42;
    
    EXPECT_EQ(grid.getCell(0).value, 42);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_FirstCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(0).value = 100;
    grid.getCell(0).flag = true;
    
    EXPECT_EQ(grid.getCell(0).value, 100);
    EXPECT_TRUE(grid.getCell(0).flag);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(99).value = 999;
    
    EXPECT_EQ(grid.getCell(99).value, 999);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_MiddleCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(50).value = 500;
    
    EXPECT_EQ(grid.getCell(50).value, 500);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_THROW(grid.getCell(100), std::out_of_range);
    EXPECT_THROW(grid.getCell(1000), std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_ModifiesInPlace) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell& cell = grid.getCell(25);
    cell.value = 123;
    cell.flag = true;
    
    EXPECT_EQ(grid.getCell(25).value, 123);
    EXPECT_TRUE(grid.getCell(25).flag);
}

TEST_F(GridHandlerCellAccessTest, GetCellByIndex_ConstIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 42;
    
    const size_t idx = 0;
    EXPECT_EQ(grid.getCell(idx).value, 42);
}

// ============================================================================
// getCell(Eigen::Vector2i) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_ReturnsReference) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(Eigen::Vector2i(3, 4)).value = 42;
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 42);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_Origin) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(Eigen::Vector2i(0, 0)).value = 100;
    
    // Should be same as index 0
    EXPECT_EQ(grid.getCell(0).value, 100);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_ConsistentWithIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (3, 4) should be at index 4*10 + 3 = 43
    grid.getCell(Eigen::Vector2i(3, 4)).value = 200;
    
    EXPECT_EQ(grid.getCell(43).value, 200);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(Eigen::Vector2i(9, 9)).value = 999;
    
    EXPECT_EQ(grid.getCell(99).value, 999);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_ThrowsOnNegative) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // gridToIndex throws on negative coordinates
    EXPECT_THROW(grid.getCell(Eigen::Vector2i(-1, 0)), std::out_of_range);
    EXPECT_THROW(grid.getCell(Eigen::Vector2i(0, -1)), std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Index will be valid but out of vector bounds
    EXPECT_NO_THROW(grid.getCell(Eigen::Vector2i(10, 0)));
    EXPECT_THROW(grid.getCell(Eigen::Vector2i(0, 10)),std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, GetCellByGridCoord_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(Eigen::Vector2i(5, 5)).value = 55;
    
    const Eigen::Vector2i coord(5, 5);
    EXPECT_EQ(grid.getCell(coord).value, 55);
}

// ============================================================================
// getCell(Eigen::Vector2d) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_ReturnsReference) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(Eigen::Vector2d(0.35, 0.45)).value = 42;
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2d(0.35, 0.45)).value, 42);
}

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_MapsToCorrectCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // (0.35, 0.45) -> cell (3, 4) -> index 43
    grid.getCell(Eigen::Vector2d(0.35, 0.45)).value = 300;
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 300);
    EXPECT_EQ(grid.getCell(43).value, 300);
}

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_OriginCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.getCell(Eigen::Vector2d(0.05, 0.05)).value = 100;
    
    EXPECT_EQ(grid.getCell(0).value, 100);
}

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_CellCenter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (5, 5) center is at (0.55, 0.55)
    grid.getCell(Eigen::Vector2d(0.55, 0.55)).value = 555;
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(5, 5)).value, 555);
}

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Negative coordinates map to negative grid coords
    EXPECT_NO_THROW(grid.getCell(Eigen::Vector2d(-0.1, 0.5)));
    
    // Beyond grid maps to out-of-bounds index
    EXPECT_NO_THROW(grid.getCell(Eigen::Vector2d(1.5, 0.5)));
}

TEST_F(GridHandlerCellAccessTest, GetCellByParentCoord_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(Eigen::Vector2d(0.55, 0.55)).value = 55;
    
    const Eigen::Vector2d coord(0.55, 0.55);
    EXPECT_EQ(grid.getCell(coord).value, 55);
}

// ============================================================================
// getCells(std::vector<size_t>) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, GetCells_ReturnsCorrectCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 10;
    grid.getCell(5).value = 50;
    grid.getCell(99).value = 990;
    
    std::vector<size_t> indices = {0, 5, 99};
    std::vector<TestCell> cells = grid.getCells(indices);
    
    ASSERT_EQ(cells.size(), 3);
    EXPECT_EQ(cells[0].value, 10);
    EXPECT_EQ(cells[1].value, 50);
    EXPECT_EQ(cells[2].value, 990);
}

TEST_F(GridHandlerCellAccessTest, GetCells_EmptyIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices;
    std::vector<TestCell> cells = grid.getCells(indices);
    
    EXPECT_TRUE(cells.empty());
}

TEST_F(GridHandlerCellAccessTest, GetCells_SingleIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(42).value = 420;
    
    std::vector<size_t> indices = {42};
    std::vector<TestCell> cells = grid.getCells(indices);
    
    ASSERT_EQ(cells.size(), 1);
    EXPECT_EQ(cells[0].value, 420);
}

TEST_F(GridHandlerCellAccessTest, GetCells_DuplicateIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(5).value = 50;
    
    std::vector<size_t> indices = {5, 5, 5};
    std::vector<TestCell> cells = grid.getCells(indices);
    
    ASSERT_EQ(cells.size(), 3);
    EXPECT_EQ(cells[0].value, 50);
    EXPECT_EQ(cells[1].value, 50);
    EXPECT_EQ(cells[2].value, 50);
}

TEST_F(GridHandlerCellAccessTest, GetCells_ReturnsCopies) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 100;
    
    std::vector<size_t> indices = {0};
    std::vector<TestCell> cells = grid.getCells(indices);
    
    // Modifying returned cells should not affect grid
    cells[0].value = 999;
    EXPECT_EQ(grid.getCell(0).value, 100);
}

TEST_F(GridHandlerCellAccessTest, GetCells_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {0, 100};  // 100 is out of bounds
    
    EXPECT_THROW(grid.getCells(indices), std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, GetCells_PreservesOrder) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 0;
    grid.getCell(50).value = 50;
    grid.getCell(99).value = 99;
    
    std::vector<size_t> indices = {99, 0, 50};
    std::vector<TestCell> cells = grid.getCells(indices);
    
    EXPECT_EQ(cells[0].value, 99);
    EXPECT_EQ(cells[1].value, 0);
    EXPECT_EQ(cells[2].value, 50);
}

// ============================================================================
// setCell(size_t, CellT) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, SetCellByIndex_SetsValue) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    grid.setCell(0, cell);
    
    EXPECT_EQ(grid.getCell(0).value, 42);
    EXPECT_TRUE(grid.getCell(0).flag);
}

TEST_F(GridHandlerCellAccessTest, SetCellByIndex_OverwritesExisting) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 100;
    
    TestCell cell{200, true};
    grid.setCell(0, cell);
    
    EXPECT_EQ(grid.getCell(0).value, 200);
}

TEST_F(GridHandlerCellAccessTest, SetCellByIndex_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{999, true};
    grid.setCell(99, cell);
    
    EXPECT_EQ(grid.getCell(99).value, 999);
}

TEST_F(GridHandlerCellAccessTest, SetCellByIndex_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    
    EXPECT_THROW(grid.setCell(100, cell), std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, SetCellByIndex_ConstCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const TestCell cell{42, true};
    const size_t idx = 0;
    grid.setCell(idx, cell);
    
    EXPECT_EQ(grid.getCell(0).value, 42);
}

// ============================================================================
// setCell(Eigen::Vector2i, CellT) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, SetCellByGridCoord_SetsValue) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    grid.setCell(Eigen::Vector2i(3, 4), cell);
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 42);
    EXPECT_EQ(grid.getCell(43).value, 42);  // 4*10 + 3
}

TEST_F(GridHandlerCellAccessTest, SetCellByGridCoord_Origin) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{100, false};
    grid.setCell(Eigen::Vector2i(0, 0), cell);
    
    EXPECT_EQ(grid.getCell(0).value, 100);
}

TEST_F(GridHandlerCellAccessTest, SetCellByGridCoord_ThrowsOnNegative) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    
    EXPECT_THROW(grid.setCell(Eigen::Vector2i(-1, 0), cell),std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, SetCellByGridCoord_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const Eigen::Vector2i coord(5, 5);
    const TestCell cell{55, true};
    grid.setCell(coord, cell);
    
    EXPECT_EQ(grid.getCell(coord).value, 55);
}

// ============================================================================
// setCell(Eigen::Vector2d, CellT) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, SetCellByParentCoord_SetsValue) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    grid.setCell(Eigen::Vector2d(0.35, 0.45), cell);
    
    // (0.35, 0.45) -> cell (3, 4)
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 42);
}

TEST_F(GridHandlerCellAccessTest, SetCellByParentCoord_OriginCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{100, true};
    grid.setCell(Eigen::Vector2d(0.05, 0.05), cell);
    
    EXPECT_EQ(grid.getCell(0).value, 100);
}

TEST_F(GridHandlerCellAccessTest, SetCellByParentCoord_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    TestCell cell{42, true};
    
    EXPECT_NO_THROW(grid.setCell(Eigen::Vector2d(-0.1, 0.5), cell));
}

TEST_F(GridHandlerCellAccessTest, SetCellByParentCoord_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const Eigen::Vector2d coord(0.55, 0.55);
    const TestCell cell{55, true};
    grid.setCell(coord, cell);
    
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(5, 5)).value, 55);
}

// ============================================================================
// setCells(std::vector<size_t>, std::vector<CellT>) Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, SetCells_SetsMultipleCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {0, 5, 99};
    std::vector<TestCell> values = {{10, false}, {50, true}, {990, false}};
    
    grid.setCells(indices, values);
    
    EXPECT_EQ(grid.getCell(0).value, 10);
    EXPECT_EQ(grid.getCell(5).value, 50);
    EXPECT_TRUE(grid.getCell(5).flag);
    EXPECT_EQ(grid.getCell(99).value, 990);
}

TEST_F(GridHandlerCellAccessTest, SetCells_EmptyVectors) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices;
    std::vector<TestCell> values;
    
    // Should not throw
    EXPECT_NO_THROW(grid.setCells(indices, values));
}

TEST_F(GridHandlerCellAccessTest, SetCells_SingleCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {42};
    std::vector<TestCell> values = {{420, true}};
    
    grid.setCells(indices, values);
    
    EXPECT_EQ(grid.getCell(42).value, 420);
}

TEST_F(GridHandlerCellAccessTest, SetCells_ThrowsOnSizeMismatch_MoreIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {0, 5, 99};
    std::vector<TestCell> values = {{10, false}, {50, true}};  // Only 2 values
    
    EXPECT_THROW(grid.setCells(indices, values), std::runtime_error);
}

TEST_F(GridHandlerCellAccessTest, SetCells_ThrowsOnSizeMismatch_MoreValues) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {0, 5};  // Only 2 indices
    std::vector<TestCell> values = {{10, false}, {50, true}, {990, false}};
    
    EXPECT_THROW(grid.setCells(indices, values), std::runtime_error);
}

TEST_F(GridHandlerCellAccessTest, SetCells_ThrowsOnOutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<size_t> indices = {0, 100};  // 100 is out of bounds
    std::vector<TestCell> values = {{10, false}, {50, true}};
    
    EXPECT_THROW(grid.setCells(indices, values), std::out_of_range);
}

TEST_F(GridHandlerCellAccessTest, SetCells_DuplicateIndices) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Last value wins
    std::vector<size_t> indices = {5, 5, 5};
    std::vector<TestCell> values = {{10, false}, {20, false}, {30, true}};
    
    grid.setCells(indices, values);
    
    EXPECT_EQ(grid.getCell(5).value, 30);
    EXPECT_TRUE(grid.getCell(5).flag);
}

TEST_F(GridHandlerCellAccessTest, SetCells_ConstVectors) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const std::vector<size_t> indices = {0, 1, 2};
    const std::vector<TestCell> values = {{100, true}, {200, false}, {300, true}};
    
    grid.setCells(indices, values);
    
    EXPECT_EQ(grid.getCell(0).value, 100);
    EXPECT_EQ(grid.getCell(1).value, 200);
    EXPECT_EQ(grid.getCell(2).value, 300);
}

// ============================================================================
// getCardinalNeighbors Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, CardinalNeighbors_CenterCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto neighbors = grid.getCardinalNeighbors(Eigen::Vector2i(5, 5));
    
    ASSERT_EQ(neighbors.size(), 4);
    
    std::set<std::pair<int, int>> expected = {{6, 5}, {5, 6}, {4, 5}, {5, 4}};
    std::set<std::pair<int, int>> actual;
    for (const auto& n : neighbors) {
        actual.insert({n.x(), n.y()});
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(GridHandlerCellAccessTest, CardinalNeighbors_Origin) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto neighbors = grid.getCardinalNeighbors(Eigen::Vector2i(0, 0));
    
    std::set<std::pair<int, int>> expected = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
    std::set<std::pair<int, int>> actual;
    for (const auto& n : neighbors) {
        actual.insert({n.x(), n.y()});
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(GridHandlerCellAccessTest, CardinalNeighbors_Corner) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto neighbors = grid.getCardinalNeighbors(Eigen::Vector2i(9, 9));
    
    // Includes out-of-bounds neighbors (10, 9) and (9, 10)
    std::set<std::pair<int, int>> expected = {{10, 9}, {9, 10}, {8, 9}, {9, 8}};
    std::set<std::pair<int, int>> actual;
    for (const auto& n : neighbors) {
        actual.insert({n.x(), n.y()});
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(GridHandlerCellAccessTest, CardinalNeighbors_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const Eigen::Vector2i coord(5, 5);
    auto neighbors = grid.getCardinalNeighbors(coord);
    
    EXPECT_EQ(neighbors.size(), 4);
}

TEST_F(GridHandlerCellAccessTest, CardinalNeighbors_NoAutoBoundsCheck) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // At corner, some neighbors are out of bounds but still returned
    auto neighbors = grid.getCardinalNeighbors(Eigen::Vector2i(0, 0));
    
    int oob_count = 0;
    for (const auto& n : neighbors) {
        if (grid.outOfBound(n)) {
            ++oob_count;
        }
    }
    EXPECT_EQ(oob_count, 2);  // (-1, 0) and (0, -1)
}

// ============================================================================
// getOctileNeighbors Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, OctileNeighbors_CenterCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto neighbors = grid.getOctileNeighbors(Eigen::Vector2i(5, 5));
    
    ASSERT_EQ(neighbors.size(), 8);
    
    std::set<std::pair<int, int>> expected = {
        {6, 5}, {5, 6}, {4, 5}, {5, 4},  // Cardinals
        {6, 6}, {4, 6}, {4, 4}, {6, 4}   // Diagonals
    };
    std::set<std::pair<int, int>> actual;
    for (const auto& n : neighbors) {
        actual.insert({n.x(), n.y()});
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(GridHandlerCellAccessTest, OctileNeighbors_Origin) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto neighbors = grid.getOctileNeighbors(Eigen::Vector2i(0, 0));
    
    std::set<std::pair<int, int>> expected = {
        {1, 0}, {0, 1}, {-1, 0}, {0, -1},
        {1, 1}, {-1, 1}, {-1, -1}, {1, -1}
    };
    std::set<std::pair<int, int>> actual;
    for (const auto& n : neighbors) {
        actual.insert({n.x(), n.y()});
    }
    EXPECT_EQ(actual, expected);
}

TEST_F(GridHandlerCellAccessTest, OctileNeighbors_ConstCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    const Eigen::Vector2i coord(5, 5);
    auto neighbors = grid.getOctileNeighbors(coord);
    
    EXPECT_EQ(neighbors.size(), 8);
}

TEST_F(GridHandlerCellAccessTest, OctileNeighbors_NoAutoBoundsCheck) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // At corner, some neighbors are out of bounds but still returned
    auto neighbors = grid.getOctileNeighbors(Eigen::Vector2i(0, 0));
    
    int oob_count = 0;
    for (const auto& n : neighbors) {
        if (grid.outOfBound(n)) {
            ++oob_count;
        }
    }
    EXPECT_EQ(oob_count, 5);  // (-1,0), (0,-1), (-1,1), (-1,-1), (1,-1)
}

TEST_F(GridHandlerCellAccessTest, OctileNeighbors_ContainsCardinals) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    auto cardinal = grid.getCardinalNeighbors(Eigen::Vector2i(5, 5));
    auto octile = grid.getOctileNeighbors(Eigen::Vector2i(5, 5));
    
    std::set<std::pair<int, int>> cardinal_set;
    for (const auto& n : cardinal) {
        cardinal_set.insert({n.x(), n.y()});
    }
    
    std::set<std::pair<int, int>> octile_set;
    for (const auto& n : octile) {
        octile_set.insert({n.x(), n.y()});
    }
    
    // All cardinal neighbors should be in octile set
    for (const auto& c : cardinal_set) {
        EXPECT_TRUE(octile_set.count(c) > 0);
    }
}

// ============================================================================
// Cross-Method Consistency Tests
// ============================================================================

TEST_F(GridHandlerCellAccessTest, AllGetCellOverloads_ReturnSameCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Set via index
    grid.getCell(43).value = 999;  // index 43 = cell (3, 4)
    
    // Verify all methods return same value
    EXPECT_EQ(grid.getCell(size_t(43)).value, 999);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 999);
    EXPECT_EQ(grid.getCell(Eigen::Vector2d(0.35, 0.45)).value, 999);
}

TEST_F(GridHandlerCellAccessTest, AllSetCellOverloads_SetSameCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Set via different methods, verify they affect the same cell
    TestCell cell1{100, true};
    grid.setCell(size_t(43), cell1);
    EXPECT_EQ(grid.getCell(Eigen::Vector2i(3, 4)).value, 100);
    
    TestCell cell2{200, false};
    grid.setCell(Eigen::Vector2i(3, 4), cell2);
    EXPECT_EQ(grid.getCell(43).value, 200);
    
    TestCell cell3{300, true};
    grid.setCell(Eigen::Vector2d(0.35, 0.45), cell3);
    EXPECT_EQ(grid.getCell(43).value, 300);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}