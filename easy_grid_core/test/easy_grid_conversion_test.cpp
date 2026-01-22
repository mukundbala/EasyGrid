#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>
#include <cmath>


struct TestCell {
    int value = 0;
};

class GridHandlerConversionTest : public ::testing::Test {
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
// parentToGrid Tests
// ============================================================================

TEST_F(GridHandlerConversionTest, ParentToGrid_OriginCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point in first cell (0,0) - cell spans [0, 0.1) x [0, 0.1)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.05, 0.05));
    
    EXPECT_EQ(result.x(), 0);
    EXPECT_EQ(result.y(), 0);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_CellBoundary) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point exactly at cell boundary should go to next cell (floor behavior)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.1, 0.1));
    
    EXPECT_EQ(result.x(), 1);
    EXPECT_EQ(result.y(), 1);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_MiddleOfGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point at (0.55, 0.35) with 0.1 resolution -> cell (5, 3)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.55, 0.35));
    
    EXPECT_EQ(result.x(), 5);
    EXPECT_EQ(result.y(), 3);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point in last cell (9, 9) - cell spans [0.9, 1.0) x [0.9, 1.0)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.95, 0.95));
    
    EXPECT_EQ(result.x(), 9);
    EXPECT_EQ(result.y(), 9);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_NegativeCoordinates) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Negative coordinates should return negative grid coords
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(-0.15, -0.25));
    
    EXPECT_EQ(result.x(), -2);  // floor(-0.15 / 0.1) = floor(-1.5) = -2
    EXPECT_EQ(result.y(), -3);  // floor(-0.25 / 0.1) = floor(-2.5) = -3
}

TEST_F(GridHandlerConversionTest, ParentToGrid_WithTranslation) {
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point at (1.05, 2.05) should map to cell (0, 0) due to offset
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(1.05, 2.05));
    
    EXPECT_EQ(result.x(), 0);
    EXPECT_EQ(result.y(), 0);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_WithTranslation_OffsetPoint) {
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point at (1.55, 2.35) should map to cell (5, 3) due to offset
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(1.55, 2.35));
    
    EXPECT_EQ(result.x(), 5);
    EXPECT_EQ(result.y(), 3);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_With90DegreeRotation) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // 90 degree rotation about Z: parent x -> map -y, parent y -> map x
    // Point at (0.0, 0.5) in parent frame -> (0.5, 0.0) in map frame -> cell (5, 0)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.0, 0.55));
    
    EXPECT_EQ(result.x(), 5);
    EXPECT_EQ(result.y(), 0);
}

TEST_F(GridHandlerConversionTest, ParentToGrid_DifferentResolution) {
    meta_.resolution = 0.5;
    meta_.map_width = 20;
    meta_.map_height = 20;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point at (2.25, 3.75) with 0.5 resolution -> cell (4, 7)
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(2.25, 3.75));
    
    EXPECT_EQ(result.x(), 4);   // floor(2.25 / 0.5) = 4
    EXPECT_EQ(result.y(), 7);   // floor(3.75 / 0.5) = 7
}

TEST_F(GridHandlerConversionTest, ParentToGrid_VerySmallResolution) {
    meta_.resolution = 0.001;  // 1mm
    meta_.map_width = 1000;
    meta_.map_height = 1000;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.5005, 0.5005));
    
    EXPECT_EQ(result.x(), 500);
    EXPECT_EQ(result.y(), 500);
}

// ============================================================================
// gridToParent Tests
// ============================================================================

TEST_F(GridHandlerConversionTest, GridToParent_OriginCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (0, 0) center should be at (0.05, 0.05)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(0, 0));
    
    EXPECT_NEAR(result.x(), 0.05, 1e-9);
    EXPECT_NEAR(result.y(), 0.05, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_ReturnsCellCenter) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (5, 3) center should be at (0.55, 0.35)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(5, 3));
    
    EXPECT_NEAR(result.x(), 0.55, 1e-9);
    EXPECT_NEAR(result.y(), 0.35, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (9, 9) center should be at (0.95, 0.95)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(9, 9));
    
    EXPECT_NEAR(result.x(), 0.95, 1e-9);
    EXPECT_NEAR(result.y(), 0.95, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_NegativeGridCoord) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (-1, -1) center should be at (-0.05, -0.05)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(-1, -1));
    
    EXPECT_NEAR(result.x(), -0.05, 1e-9);
    EXPECT_NEAR(result.y(), -0.05, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_WithTranslation) {
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (0, 0) center should be at (1.05, 2.05)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(0, 0));
    
    EXPECT_NEAR(result.x(), 1.05, 1e-9);
    EXPECT_NEAR(result.y(), 2.05, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_With90DegreeRotation) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (5, 0) in map frame -> rotated to parent frame
    // Map (0.55, 0.05) rotated 90 deg -> parent (-0.05, 0.55)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(5, 0));
    
    EXPECT_NEAR(result.x(), -0.05, 1e-9);
    EXPECT_NEAR(result.y(), 0.55, 1e-9);
}

TEST_F(GridHandlerConversionTest, GridToParent_DifferentResolution) {
    meta_.resolution = 0.5;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Cell (4, 7) center with 0.5 resolution -> (2.25, 3.75)
    Eigen::Vector2d result = grid.gridToParent(Eigen::Vector2i(4, 7));
    
    EXPECT_NEAR(result.x(), 2.25, 1e-9);
    EXPECT_NEAR(result.y(), 3.75, 1e-9);
}

// ============================================================================
// gridToIndex Tests
// ============================================================================

TEST_F(GridHandlerConversionTest, GridToIndex_Origin) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(0, 0)), 0);
}

TEST_F(GridHandlerConversionTest, GridToIndex_RowMajorOrder) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Row-major: index = v * width + u
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(0, 0)), 0);
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(1, 0)), 1);
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(9, 0)), 9);
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(0, 1)), 10);
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(5, 3)), 35);  // 3 * 10 + 5
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(9, 9)), 99);
}

TEST_F(GridHandlerConversionTest, GridToIndex_LastCell) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(9, 9)), 99);
}

TEST_F(GridHandlerConversionTest, GridToIndex_DifferentGridSize) {
    meta_.map_width = 20;
    meta_.map_height = 15;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // index = v * width + u = 5 * 20 + 10 = 110
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(10, 5)), 110);
}

// ============================================================================
// indexToGrid Tests
// ============================================================================

TEST_F(GridHandlerConversionTest, IndexToGrid_Zero) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2i result = grid.indexToGrid(0);
    
    EXPECT_EQ(result.x(), 0);
    EXPECT_EQ(result.y(), 0);
}

TEST_F(GridHandlerConversionTest, IndexToGrid_RowMajorOrder) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_EQ(grid.indexToGrid(0), Eigen::Vector2i(0, 0));
    EXPECT_EQ(grid.indexToGrid(1), Eigen::Vector2i(1, 0));
    EXPECT_EQ(grid.indexToGrid(9), Eigen::Vector2i(9, 0));
    EXPECT_EQ(grid.indexToGrid(10), Eigen::Vector2i(0, 1));
    EXPECT_EQ(grid.indexToGrid(35), Eigen::Vector2i(5, 3));
    EXPECT_EQ(grid.indexToGrid(99), Eigen::Vector2i(9, 9));
}

TEST_F(GridHandlerConversionTest, IndexToGrid_LastIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2i result = grid.indexToGrid(99);
    
    EXPECT_EQ(result.x(), 9);
    EXPECT_EQ(result.y(), 9);
}

TEST_F(GridHandlerConversionTest, IndexToGrid_DifferentGridSize) {
    meta_.map_width = 20;
    meta_.map_height = 15;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // index 110 = 5 * 20 + 10 -> (10, 5)
    Eigen::Vector2i result = grid.indexToGrid(110);
    
    EXPECT_EQ(result.x(), 10);
    EXPECT_EQ(result.y(), 5);
}

// ============================================================================
// Round-Trip Conversion Tests
// ============================================================================

TEST_F(GridHandlerConversionTest, RoundTrip_GridToParentToGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i original(u, v);
            Eigen::Vector2d parent = grid.gridToParent(original);
            Eigen::Vector2i recovered = grid.parentToGrid(parent);
            
            EXPECT_EQ(recovered.x(), original.x()) 
                << "Failed at grid coord (" << u << ", " << v << ")";
            EXPECT_EQ(recovered.y(), original.y()) 
                << "Failed at grid coord (" << u << ", " << v << ")";
        }
    }
}

TEST_F(GridHandlerConversionTest, RoundTrip_IndexToGridToIndex) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (size_t i = 0; i < 100; ++i) {
        Eigen::Vector2i grid_coord = grid.indexToGrid(i);
        size_t recovered = grid.gridToIndex(grid_coord);
        
        EXPECT_EQ(recovered, i) << "Failed at index " << i;
    }
}

TEST_F(GridHandlerConversionTest, RoundTrip_GridToIndexToGrid) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i original(u, v);
            size_t index = grid.gridToIndex(original);
            Eigen::Vector2i recovered = grid.indexToGrid(index);
            
            EXPECT_EQ(recovered.x(), original.x());
            EXPECT_EQ(recovered.y(), original.y());
        }
    }
}

TEST_F(GridHandlerConversionTest, RoundTrip_WithTranslation) {
    Eigen::Vector3d translation(5.0, -3.0, 0.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i original(u, v);
            Eigen::Vector2d parent = grid.gridToParent(original);
            Eigen::Vector2i recovered = grid.parentToGrid(parent);
            
            EXPECT_EQ(recovered, original);
        }
    }
}

TEST_F(GridHandlerConversionTest, RoundTrip_WithRotation) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i original(u, v);
            Eigen::Vector2d parent = grid.gridToParent(original);
            Eigen::Vector2i recovered = grid.parentToGrid(parent);
            
            EXPECT_EQ(recovered, original);
        }
    }
}

TEST_F(GridHandlerConversionTest, RoundTrip_WithTranslationAndRotation) {
    Eigen::Vector3d translation(10.0, -5.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    for (int u = 0; u < 10; ++u) {
        for (int v = 0; v < 10; ++v) {
            Eigen::Vector2i original(u, v);
            Eigen::Vector2d parent = grid.gridToParent(original);
            Eigen::Vector2i recovered = grid.parentToGrid(parent);
            
            EXPECT_EQ(recovered, original);
        }
    }
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(GridHandlerConversionTest, EdgeCase_VeryLargeCoordinates) {
    meta_.map_width = 10000;
    meta_.map_height = 10000;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    Eigen::Vector2i large_coord(9999, 9999);
    size_t index = grid.gridToIndex(large_coord);
    Eigen::Vector2i recovered = grid.indexToGrid(index);
    
    EXPECT_EQ(recovered, large_coord);
}

TEST_F(GridHandlerConversionTest, EdgeCase_SingleCellGrid) {
    meta_.map_width = 1;
    meta_.map_height = 1;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(0, 0)), 0);
    EXPECT_EQ(grid.indexToGrid(0), Eigen::Vector2i(0, 0));
    
    Eigen::Vector2d center = grid.gridToParent(Eigen::Vector2i(0, 0));
    EXPECT_NEAR(center.x(), 0.05, 1e-9);
    EXPECT_NEAR(center.y(), 0.05, 1e-9);
}

TEST_F(GridHandlerConversionTest, EdgeCase_NonSquareGrid) {
    meta_.map_width = 20;
    meta_.map_height = 5;
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Last cell
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(19, 4)), 99);
    EXPECT_EQ(grid.indexToGrid(99), Eigen::Vector2i(19, 4));
    
    // First cell of last row
    EXPECT_EQ(grid.gridToIndex(Eigen::Vector2i(0, 4)), 80);
    EXPECT_EQ(grid.indexToGrid(80), Eigen::Vector2i(0, 4));
}

TEST_F(GridHandlerConversionTest, EdgeCase_PointExactlyOnCellCorner) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point exactly at (0.5, 0.5) - corner of cells (4,4), (5,4), (4,5), (5,5)
    // floor(0.5 / 0.1) = floor(5.0) = 5
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.5, 0.5));
    
    EXPECT_EQ(result.x(), 5);
    EXPECT_EQ(result.y(), 5);
}

TEST_F(GridHandlerConversionTest, EdgeCase_PointJustInsideBoundary) {
    easy_grid::GridHandler<TestCell> grid(meta_);
    
    // Point just inside (0.1 - epsilon)
    double eps = 1e-10;
    Eigen::Vector2i result = grid.parentToGrid(Eigen::Vector2d(0.1 - eps, 0.1 - eps));
    
    EXPECT_EQ(result.x(), 0);
    EXPECT_EQ(result.y(), 0);
}


int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
