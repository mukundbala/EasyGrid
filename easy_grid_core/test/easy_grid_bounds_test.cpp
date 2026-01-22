
#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>
#include <cmath>

struct TestCell {
    int value = 0;
};

class GridHandlerBoundsTest : public ::testing::Test {
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
// outOfBound(Eigen::Vector2i) - Grid Coordinate Tests
// ============================================================================

TEST_F(GridHandlerBoundsTest, GridCoord_OriginInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(0, 0)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_AllCornersInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(0, 0)));  // Bottom-left
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(9, 0)));  // Bottom-right
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(0, 9)));  // Top-left
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(9, 9)));  // Top-right
}

TEST_F(GridHandlerBoundsTest, GridCoord_MiddleInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(5, 5)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(3, 7)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(8, 2)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_NegativeU_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-1, 0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-1, 5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-100, 0)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_NegativeV_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, -1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(5, -1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, -100)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_BothNegative_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-1, -1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-5, -5)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_U_AtWidth_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(10, 0)));  // Width is 10, valid range [0,9]
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(10, 5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(10, 9)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_V_AtHeight_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, 10)));  // Height is 10, valid range [0,9]
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(5, 10)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(9, 10)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_BothExceedBounds_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(10, 10)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(100, 100)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_LargePositive_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(1000, 0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, 1000)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(1000, 1000)));
}

TEST_F(GridHandlerBoundsTest, GridCoord_BoundaryValues) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Last valid
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(9, 9)));
    // First invalid
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(10, 9)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(9, 10)));
}

// ============================================================================
// outOfBound(size_t) - Index Tests
// ============================================================================

TEST_F(GridHandlerBoundsTest, Index_ZeroInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(size_t(0)));
}

TEST_F(GridHandlerBoundsTest, Index_LastValidInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(size_t(99)));  // 10*10 - 1
}

TEST_F(GridHandlerBoundsTest, Index_MiddleInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(size_t(50)));
    EXPECT_FALSE(grid.outOfBound(size_t(1)));
    EXPECT_FALSE(grid.outOfBound(size_t(98)));
}

TEST_F(GridHandlerBoundsTest, Index_AtTotalCells_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(size_t(100)));  // Total cells = 100
}

TEST_F(GridHandlerBoundsTest, Index_BeyondTotal_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(size_t(101)));
    EXPECT_TRUE(grid.outOfBound(size_t(1000)));
    EXPECT_TRUE(grid.outOfBound(size_t(1000000)));
}

TEST_F(GridHandlerBoundsTest, Index_MaxSizeT_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(std::numeric_limits<size_t>::max()));
}

TEST_F(GridHandlerBoundsTest, Index_BoundaryValues) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Last valid
    EXPECT_FALSE(grid.outOfBound(size_t(99)));
    // First invalid
    EXPECT_TRUE(grid.outOfBound(size_t(100)));
}

// ============================================================================
// outOfBound(Eigen::Vector2d) - Parent Coordinate Tests
// ============================================================================

TEST_F(GridHandlerBoundsTest, ParentCoord_CenterInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.5, 0.5)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_OriginCellInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.05, 0.05)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.0, 0.0)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.099, 0.099)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_LastCellInBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.95, 0.95)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.999, 0.999)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_NegativeX_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(-0.01, 0.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(-0.1, 0.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(-1.0, 0.5)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_NegativeY_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, -0.01)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, -0.1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, -1.0)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_BothNegative_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(-0.1, -0.1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(-1.0, -1.0)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_BeyondWidth_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Grid spans [0, 1.0) in x (10 cells * 0.1 resolution)
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(1.0, 0.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(1.5, 0.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(10.0, 0.5)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_BeyondHeight_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Grid spans [0, 1.0) in y (10 cells * 0.1 resolution)
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, 1.0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, 1.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, 10.0)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_BeyondBoth_OutOfBounds) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(1.0, 1.0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(2.0, 2.0)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_JustInsideBoundary) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    double eps = 1e-10;
    // Just inside upper boundary
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(1.0 - eps, 1.0 - eps)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_ExactlyOnUpperBoundary) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Exactly at x=1.0 maps to cell 10, which is out of bounds
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(1.0, 0.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, 1.0)));
}

// ============================================================================
// outOfBound with Transforms
// ============================================================================

TEST_F(GridHandlerBoundsTest, ParentCoord_WithTranslation_InBounds) {
    Eigen::Vector3d translation(5.0, 5.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Grid now spans [5.0, 6.0) x [5.0, 6.0)
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(5.5, 5.5)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(5.0, 5.0)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(5.99, 5.99)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_WithTranslation_OutOfBounds) {
    Eigen::Vector3d translation(5.0, 5.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Original (0,0) is now out of bounds
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.0, 0.0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.5, 0.5)));
    
    // Beyond translated grid
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(6.0, 5.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(5.5, 6.0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(4.9, 5.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(5.5, 4.9)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_WithNegativeTranslation_InBounds) {
    Eigen::Vector3d translation(-5.0, -5.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Grid now spans [-5.0, -4.0) x [-5.0, -4.0)
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(-4.5, -4.5)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(-5.0, -5.0)));
}

TEST_F(GridHandlerBoundsTest, ParentCoord_WithRotation_InBounds) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Center of cell (5,5) in rotated frame
    Eigen::Vector2d cell_center = grid.gridToParent(Eigen::Vector2i(5, 5));
    EXPECT_FALSE(grid.outOfBound(cell_center));
}

// ============================================================================
// Different Grid Sizes
// ============================================================================

TEST_F(GridHandlerBoundsTest, SingleCellGrid_InBounds) {
    meta_.map_width = 1;
    meta_.map_height = 1;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(0, 0)));
    EXPECT_FALSE(grid.outOfBound(size_t(0)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(0.05, 0.05)));
}

TEST_F(GridHandlerBoundsTest, SingleCellGrid_OutOfBounds) {
    meta_.map_width = 1;
    meta_.map_height = 1;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(1, 0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, 1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(-1, 0)));
    EXPECT_TRUE(grid.outOfBound(size_t(1)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(0.1, 0.05)));
}

TEST_F(GridHandlerBoundsTest, LargeGrid_Bounds) {
    meta_.map_width = 1000;
    meta_.map_height = 1000;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // In bounds
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(999, 999)));
    EXPECT_FALSE(grid.outOfBound(size_t(999999)));
    
    // Out of bounds
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(1000, 999)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(999, 1000)));
    EXPECT_TRUE(grid.outOfBound(size_t(1000000)));
}

TEST_F(GridHandlerBoundsTest, NonSquareGrid_Bounds) {
    meta_.map_width = 20;
    meta_.map_height = 5;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // In bounds
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(19, 4)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2i(0, 0)));
    EXPECT_FALSE(grid.outOfBound(size_t(99)));  // 20*5 - 1
    
    // Out of bounds
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(20, 0)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2i(0, 5)));
    EXPECT_TRUE(grid.outOfBound(size_t(100)));
}

TEST_F(GridHandlerBoundsTest, DifferentResolution_Bounds) {
    meta_.resolution = 0.5;
    meta_.map_width = 10;
    meta_.map_height = 10;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Grid spans [0, 5.0) x [0, 5.0) with 0.5 resolution
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(2.5, 2.5)));
    EXPECT_FALSE(grid.outOfBound(Eigen::Vector2d(4.99, 4.99)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(5.0, 2.5)));
    EXPECT_TRUE(grid.outOfBound(Eigen::Vector2d(2.5, 5.0)));
}

// ============================================================================
// Const Correctness Tests
// ============================================================================

TEST_F(GridHandlerBoundsTest, ConstCorrectness_GridCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    const Eigen::Vector2i coord(5, 5);
    
    EXPECT_FALSE(grid.outOfBound(coord));
}

TEST_F(GridHandlerBoundsTest, ConstCorrectness_ParentCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    const Eigen::Vector2d coord(0.5, 0.5);
    
    EXPECT_FALSE(grid.outOfBound(coord));
}

TEST_F(GridHandlerBoundsTest, ConstCorrectness_Index) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    const size_t index = 50;
    
    EXPECT_FALSE(grid.outOfBound(index));
}

// ============================================================================
// Consistency Tests
// ============================================================================

TEST_F(GridHandlerBoundsTest, Consistency_GridAndIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // All valid grid coords should have valid indices
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i coord(u, v);
            EXPECT_FALSE(grid.outOfBound(coord));
            
            size_t index = grid.gridToIndex(coord);
            EXPECT_FALSE(grid.outOfBound(index));
        }
    }
}

TEST_F(GridHandlerBoundsTest, Consistency_ParentAndGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // All cell centers should be in bounds
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i grid_coord(u, v);
            Eigen::Vector2d parent_coord = grid.gridToParent(grid_coord);
            
            EXPECT_FALSE(grid.outOfBound(grid_coord));
            EXPECT_FALSE(grid.outOfBound(parent_coord));
        }
    }
}

TEST_F(GridHandlerBoundsTest, Consistency_AllThreeOverloads) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Test a point that's in bounds
    Eigen::Vector2d parent_in(0.55, 0.35);
    Eigen::Vector2i grid_in = grid.parentToGrid(parent_in);
    size_t index_in = grid.gridToIndex(grid_in);
    
    EXPECT_FALSE(grid.outOfBound(parent_in));
    EXPECT_FALSE(grid.outOfBound(grid_in));
    EXPECT_FALSE(grid.outOfBound(index_in));
    
    // Test a point that's out of bounds
    Eigen::Vector2d parent_out(-0.1, 0.5);
    Eigen::Vector2i grid_out = grid.parentToGrid(parent_out);  // Will be (-1, 5)
    
    EXPECT_TRUE(grid.outOfBound(parent_out));
    EXPECT_TRUE(grid.outOfBound(grid_out));
    // Note: Can't test index for negative grid coords as gridToIndex throws
}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}