#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>

// Simple test cell type
struct TestCell {
    int value = 0;
    bool flag = false;
    
    bool operator==(const TestCell& other) const {
        return value == other.value && flag == other.flag;
    }
};

class GridHandlerConstructorTest : public ::testing::Test {
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
// Default Constructor Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, DefaultConstructor_CreatesEmptyGrid) {
    easy_grid::GridHandler<TestCell> grid;
    
    EXPECT_EQ(grid.getTotalCells(), 0);
    EXPECT_EQ(grid.getCapacity(), 0);
}

TEST_F(GridHandlerConstructorTest, DefaultConstructor_DefaultMetaData) {
    easy_grid::GridHandler<TestCell> grid;
    
    easy_grid::MetaData meta = grid.getMetaData();
    EXPECT_DOUBLE_EQ(meta.resolution, 0.1);
    EXPECT_EQ(meta.map_width, 100);
    EXPECT_EQ(meta.map_height, 100);
}

// ============================================================================
// Constructor with MetaData Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, MetaDataConstructor_CorrectSize) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_EQ(grid.getTotalCells(), 100);  // 10 x 10
    EXPECT_GE(grid.getCapacity(), 100);
}

TEST_F(GridHandlerConstructorTest, MetaDataConstructor_StoresMetaData) {
    meta_.resolution = 0.05;
    meta_.map_width = 20;
    meta_.map_height = 15;
    
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);
    EXPECT_EQ(grid.getWidth(), 20);
    EXPECT_EQ(grid.getHeight(), 15);
    EXPECT_EQ(grid.getTotalCells(), 300);  // 20 x 15
}

TEST_F(GridHandlerConstructorTest, MetaDataConstructor_DefaultInitializesCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // All cells should be default initialized
    for (size_t i = 0; i < grid.getTotalCells(); ++i) {
        EXPECT_EQ(grid.getCell(i).value, 0);
        EXPECT_FALSE(grid.getCell(i).flag);
    }
}

TEST_F(GridHandlerConstructorTest, MetaDataConstructor_StoresTransform) {
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.getMapTranslation().isApprox(translation));
    EXPECT_TRUE(grid.getMapRotation().isApprox(rotation, 1e-9));
}

// ============================================================================
// Constructor with MetaData and Cells Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, CellsConstructor_MovesCells) {
    std::vector<TestCell> cells(100);
    for (size_t i = 0; i < cells.size(); ++i) {
        cells[i].value = static_cast<int>(i);
    }
    
    easy_grid::GridHandler<TestCell> grid(meta_, cells);
    
    EXPECT_EQ(grid.getTotalCells(), 100);
    EXPECT_EQ(grid.getCell(0).value, 0);
    EXPECT_EQ(grid.getCell(50).value, 50);
    EXPECT_EQ(grid.getCell(99).value, 99);
    
    // Original vector should be empty (moved from)
    EXPECT_TRUE(cells.empty());
}

TEST_F(GridHandlerConstructorTest, CellsConstructor_ThrowsOnSizeMismatch_TooFew) {
    std::vector<TestCell> cells(50);  // Too few
    
    EXPECT_THROW(
        easy_grid::GridHandler<TestCell> grid(meta_, cells),
        std::runtime_error
    );
}

TEST_F(GridHandlerConstructorTest, CellsConstructor_ThrowsOnSizeMismatch_TooMany) {
    std::vector<TestCell> cells(200);  // Too many
    
    EXPECT_THROW(
        easy_grid::GridHandler<TestCell> grid(meta_, cells),
        std::runtime_error
    );
}

TEST_F(GridHandlerConstructorTest, CellsConstructor_EmptyCellsForEmptyGrid) {
    easy_grid::MetaData empty_meta;
    empty_meta.map_width = 0;
    empty_meta.map_height = 0;
    std::vector<TestCell> cells;
    
    easy_grid::GridHandler<TestCell> grid(empty_meta, cells);
    
    EXPECT_EQ(grid.getTotalCells(), 0);
}

// ============================================================================
// Move Constructor Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, MoveConstructor_TransfersOwnership) {
    easy_grid::GridHandler<TestCell> original(meta_);
    original.getCell(0).value = 42;
    original.getCell(99).value = 99;
    
    easy_grid::GridHandler<TestCell> moved(std::move(original));
    
    EXPECT_EQ(moved.getTotalCells(), 100);
    EXPECT_EQ(moved.getCell(0).value, 42);
    EXPECT_EQ(moved.getCell(99).value, 99);
}

TEST_F(GridHandlerConstructorTest, MoveConstructor_InvalidatesSource) {
    easy_grid::GridHandler<TestCell> original(meta_);
    
    easy_grid::GridHandler<TestCell> moved(std::move(original));
    
    EXPECT_EQ(original.getTotalCells(), 0);
}

TEST_F(GridHandlerConstructorTest, MoveConstructor_PreservesMetaData) {
    meta_.resolution = 0.05;
    meta_.map_width = 20;
    meta_.map_height = 15;
    easy_grid::GridHandler<TestCell> original(meta_);
    
    easy_grid::GridHandler<TestCell> moved(std::move(original));
    
    EXPECT_DOUBLE_EQ(moved.getResolution(), 0.05);
    EXPECT_EQ(moved.getWidth(), 20);
    EXPECT_EQ(moved.getHeight(), 15);
}

// ============================================================================
// Move Assignment Operator Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, MoveAssignment_TransfersOwnership) {
    easy_grid::GridHandler<TestCell> original(meta_);
    original.getCell(0).value = 42;
    
    easy_grid::GridHandler<TestCell> assigned;
    assigned = std::move(original);
    
    EXPECT_EQ(assigned.getTotalCells(), 100);
    EXPECT_EQ(assigned.getCell(0).value, 42);
}

TEST_F(GridHandlerConstructorTest, MoveAssignment_InvalidatesSource) {
    easy_grid::GridHandler<TestCell> original(meta_);
    
    easy_grid::GridHandler<TestCell> assigned;
    assigned = std::move(original);
    
    EXPECT_EQ(original.getTotalCells(), 0);
}

TEST_F(GridHandlerConstructorTest, MoveAssignment_OverwritesExistingData) {
    easy_grid::GridHandler<TestCell> original(meta_);
    original.getCell(0).value = 42;
    
    easy_grid::MetaData other_meta;
    other_meta.resolution = 0.2;
    other_meta.map_width = 5;
    other_meta.map_height = 5;
    easy_grid::GridHandler<TestCell> assigned(other_meta);
    assigned.getCell(0).value = 100;
    
    assigned = std::move(original);
    
    EXPECT_EQ(assigned.getTotalCells(), 100);  // Now 10x10
    EXPECT_EQ(assigned.getCell(0).value, 42);
    EXPECT_DOUBLE_EQ(assigned.getResolution(), 0.1);
}

TEST_F(GridHandlerConstructorTest, MoveAssignment_SelfAssignmentIsSafe) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 42;
    grid.getCell(50).value = 100;
    
    grid = std::move(grid);
    
    // Grid should still be valid after self-assignment
    EXPECT_EQ(grid.getTotalCells(), 100);
    EXPECT_EQ(grid.getCell(0).value, 42);
    EXPECT_EQ(grid.getCell(50).value, 100);
}

TEST_F(GridHandlerConstructorTest, MoveAssignment_ReturnsReference) {
    easy_grid::GridHandler<TestCell> original(meta_);
    easy_grid::GridHandler<TestCell> assigned;
    
    easy_grid::GridHandler<TestCell>& ref = (assigned = std::move(original));
    
    EXPECT_EQ(&ref, &assigned);
}

// ============================================================================
// setGridCells Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, SetGridCells_ReplacesCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<TestCell> new_cells(100);
    for (size_t i = 0; i < new_cells.size(); ++i) {
        new_cells[i].value = static_cast<int>(i * 2);
    }
    
    grid.setGridCells(new_cells);
    
    EXPECT_EQ(grid.getCell(0).value, 0);
    EXPECT_EQ(grid.getCell(50).value, 100);
}

TEST_F(GridHandlerConstructorTest, SetGridCells_MovesCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<TestCell> new_cells(100);
    new_cells[0].value = 999;
    
    grid.setGridCells(new_cells);
    
    EXPECT_TRUE(new_cells.empty());  // Should be moved from
}

TEST_F(GridHandlerConstructorTest, SetGridCells_ThrowsOnSizeMismatch) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<TestCell> wrong_size(50);
    
    EXPECT_THROW(grid.setGridCells(wrong_size), std::runtime_error);
}

TEST_F(GridHandlerConstructorTest, SetGridCells_PreservesMetaData) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    std::vector<TestCell> new_cells(100);
    grid.setGridCells(new_cells);
    
    EXPECT_EQ(grid.getWidth(), 10);
    EXPECT_EQ(grid.getHeight(), 10);
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.1);
}

// ============================================================================
// setMetaData Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, SetMetaData_UpdatesMetaData) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    easy_grid::MetaData new_meta;
    new_meta.resolution = 0.05;
    new_meta.map_width = 20;
    new_meta.map_height = 15;
    
    grid.setMetaData(new_meta);
    
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);
    EXPECT_EQ(grid.getWidth(), 20);
    EXPECT_EQ(grid.getHeight(), 15);
}

TEST_F(GridHandlerConstructorTest, SetMetaData_ResizesGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    EXPECT_EQ(grid.getTotalCells(), 100);
    
    easy_grid::MetaData new_meta;
    new_meta.map_width = 20;
    new_meta.map_height = 20;
    
    grid.setMetaData(new_meta);
    
    EXPECT_EQ(grid.getTotalCells(), 400);
}

TEST_F(GridHandlerConstructorTest, SetMetaData_ClearsExistingData) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.getCell(0).value = 42;
    
    easy_grid::MetaData new_meta;
    new_meta.map_width = 10;
    new_meta.map_height = 10;
    
    grid.setMetaData(new_meta);
    
    // Data should be cleared and default-initialized
    EXPECT_EQ(grid.getCell(0).value, 0);
}

TEST_F(GridHandlerConstructorTest, SetMetaData_UpdatesTransform) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    easy_grid::MetaData new_meta;
    new_meta.map_width = 10;
    new_meta.map_height = 10;
    Eigen::Vector3d translation(5.0, 5.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    new_meta.set_transform(translation, rotation);
    
    grid.setMetaData(new_meta);
    
    EXPECT_TRUE(grid.getMapTranslation().isApprox(translation));
}

// ============================================================================
// cloneGrid Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, CloneGrid_CreatesCopy) {
    easy_grid::GridHandler<TestCell> original(meta_);
    original.getCell(0).value = 42;
    original.getCell(50).value = 100;
    original.getCell(99).value = 999;
    
    easy_grid::GridHandler<TestCell> cloned = original.cloneGrid();
    
    EXPECT_EQ(cloned.getTotalCells(), original.getTotalCells());
    EXPECT_EQ(cloned.getCell(0).value, 42);
    EXPECT_EQ(cloned.getCell(50).value, 100);
    EXPECT_EQ(cloned.getCell(99).value, 999);
}

TEST_F(GridHandlerConstructorTest, CloneGrid_IsIndependent) {
    easy_grid::GridHandler<TestCell> original(meta_);
    original.getCell(0).value = 42;
    
    easy_grid::GridHandler<TestCell> cloned = original.cloneGrid();
    cloned.getCell(0).value = 999;
    
    // Original should be unchanged
    EXPECT_EQ(original.getCell(0).value, 42);
    EXPECT_EQ(cloned.getCell(0).value, 999);
}

TEST_F(GridHandlerConstructorTest, CloneGrid_CopiesMetaData) {
    meta_.resolution = 0.05;
    meta_.map_width = 20;
    meta_.map_height = 15;
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    
    easy_grid::GridHandler<TestCell> original(meta_);
    easy_grid::GridHandler<TestCell> cloned = original.cloneGrid();
    
    EXPECT_DOUBLE_EQ(cloned.getResolution(), 0.05);
    EXPECT_EQ(cloned.getWidth(), 20);
    EXPECT_EQ(cloned.getHeight(), 15);
    EXPECT_TRUE(cloned.getMapTranslation().isApprox(translation));
}

TEST_F(GridHandlerConstructorTest, CloneGrid_CopiesAllCells) {
    easy_grid::GridHandler<TestCell> original(meta_);
    for (size_t i = 0; i < original.getTotalCells(); ++i) {
        original.getCell(i).value = static_cast<int>(i);
    }
    
    easy_grid::GridHandler<TestCell> cloned = original.cloneGrid();
    
    for (size_t i = 0; i < cloned.getTotalCells(); ++i) {
        EXPECT_EQ(cloned.getCell(i).value, static_cast<int>(i));
    }
}

// ============================================================================
// clearGrid Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, ClearGrid_RemovesAllCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    EXPECT_EQ(grid.getTotalCells(), 100);
    
    grid.clearGrid();
    
    EXPECT_EQ(grid.getTotalCells(), 0);
}

TEST_F(GridHandlerConstructorTest, ClearGrid_PreservesCapacity) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    size_t original_capacity = grid.getCapacity();
    
    grid.clearGrid();
    
    EXPECT_GE(grid.getCapacity(), original_capacity);
}

TEST_F(GridHandlerConstructorTest, ClearGrid_PreservesMetaData) {
    meta_.resolution = 0.05;
    meta_.map_width = 20;
    meta_.map_height = 15;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.clearGrid();
    
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);
    EXPECT_EQ(grid.getWidth(), 20);
    EXPECT_EQ(grid.getHeight(), 15);
}

// ============================================================================
// clearAndDeallocGrid Tests
// ============================================================================

TEST_F(GridHandlerConstructorTest, ClearAndDeallocGrid_RemovesAllCells) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    EXPECT_EQ(grid.getTotalCells(), 100);
    
    grid.clearAndDeallocGrid();
    
    EXPECT_EQ(grid.getTotalCells(), 0);
}

TEST_F(GridHandlerConstructorTest, ClearAndDeallocGrid_DeallocatesMemory) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    EXPECT_GE(grid.getCapacity(), 100);
    
    grid.clearAndDeallocGrid();
    
    EXPECT_EQ(grid.getCapacity(), 0);
}

TEST_F(GridHandlerConstructorTest, ClearAndDeallocGrid_PreservesMetaData) {
    meta_.resolution = 0.05;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    grid.clearAndDeallocGrid();
    
    EXPECT_DOUBLE_EQ(grid.getResolution(), 0.05);
}

TEST_F(GridHandlerConstructorTest, ClearAndDeallocGrid_AllowsReuse) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    grid.clearAndDeallocGrid();
    
    // Should be able to set new metadata and use the grid again
    easy_grid::MetaData new_meta;
    new_meta.map_width = 5;
    new_meta.map_height = 5;
    grid.setMetaData(new_meta);
    
    EXPECT_EQ(grid.getTotalCells(), 25);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}