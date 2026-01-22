#include <gtest/gtest.h>
#include <easy_grid_core/easy_grid.hpp>



class MetaDataTest : public ::testing::Test {
protected:
    easy_grid::MetaData meta_;
};

// ============================================================================
// Default Initialization Tests
// ============================================================================

TEST_F(MetaDataTest, DefaultValues) {
    easy_grid::MetaData m;
    EXPECT_DOUBLE_EQ(m.resolution, 0.1);
    EXPECT_EQ(m.map_width, 100);
    EXPECT_EQ(m.map_height, 100);
    EXPECT_TRUE(m.map_frame_transform.isApprox(Eigen::Matrix4d::Identity()));
}

TEST_F(MetaDataTest, DefaultTransformIsIdentity) {
    easy_grid::MetaData m;
    EXPECT_TRUE(m.get_transform_as_mat().isApprox(Eigen::Matrix4d::Identity()));
    EXPECT_TRUE(m.get_translation().isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(m.get_rotation_as_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

// ============================================================================
// set_transform Tests
// ============================================================================

TEST_F(MetaDataTest, SetTransform_TranslationOnly) {
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    
    meta_.set_transform(translation, rotation);
    
    EXPECT_TRUE(meta_.get_translation().isApprox(translation));
    EXPECT_TRUE(meta_.get_rotation_as_matrix().isApprox(Eigen::Matrix3d::Identity()));
}

TEST_F(MetaDataTest, SetTransform_RotationOnly) {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    
    meta_.set_transform(translation, rotation);
    
    EXPECT_TRUE(meta_.get_translation().isApprox(Eigen::Vector3d::Zero()));
    EXPECT_TRUE(meta_.get_rotation_as_quat().isApprox(rotation, 1e-9));
}

TEST_F(MetaDataTest, SetTransform_TranslationAndRotation) {
    Eigen::Vector3d translation(1.0, 2.0, 0.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    
    meta_.set_transform(translation, rotation);
    
    EXPECT_TRUE(meta_.get_translation().isApprox(translation));
    EXPECT_TRUE(meta_.get_rotation_as_quat().isApprox(rotation, 1e-9));
}

TEST_F(MetaDataTest, SetTransform_NormalizesUnnormalizedQuaternion) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond unnormalized(2.0, 0.0, 0.0, 0.0);  // Not unit length
    
    meta_.set_transform(translation, unnormalized);
    
    Eigen::Quaterniond result = meta_.get_rotation_as_quat();
    EXPECT_NEAR(result.norm(), 1.0, 1e-9);
}

TEST_F(MetaDataTest, SetTransform_PreservesAlreadyNormalizedQuaternion) {
    Eigen::Vector3d translation(0.0, 0.0, 0.0);
    Eigen::Quaterniond normalized(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()));
    
    meta_.set_transform(translation, normalized);
    
    EXPECT_TRUE(meta_.get_rotation_as_quat().isApprox(normalized, 1e-9));
}

// ============================================================================
// get_transform_as_vec Tests
// ============================================================================

TEST_F(MetaDataTest, GetTransformAsVec_ReturnsCorrectSize) {
    Eigen::VectorXd vec = meta_.get_transform_as_vec();
    EXPECT_EQ(vec.size(), 7);
}

TEST_F(MetaDataTest, GetTransformAsVec_IdentityTransform) {
    Eigen::VectorXd vec = meta_.get_transform_as_vec();
    
    // Translation should be zero
    EXPECT_DOUBLE_EQ(vec(0), 0.0);
    EXPECT_DOUBLE_EQ(vec(1), 0.0);
    EXPECT_DOUBLE_EQ(vec(2), 0.0);
    
    // Identity quaternion: (0, 0, 0, 1)
    EXPECT_NEAR(vec(3), 0.0, 1e-9);  // qx
    EXPECT_NEAR(vec(4), 0.0, 1e-9);  // qy
    EXPECT_NEAR(vec(5), 0.0, 1e-9);  // qz
    EXPECT_NEAR(vec(6), 1.0, 1e-9);  // qw
}

TEST_F(MetaDataTest, GetTransformAsVec_WithTranslation) {
    Eigen::Vector3d translation(1.5, 2.5, 3.5);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(translation, rotation);
    
    Eigen::VectorXd vec = meta_.get_transform_as_vec();
    
    EXPECT_DOUBLE_EQ(vec(0), 1.5);
    EXPECT_DOUBLE_EQ(vec(1), 2.5);
    EXPECT_DOUBLE_EQ(vec(2), 3.5);
}

// ============================================================================
// get_transform_as_mat Tests
// ============================================================================

TEST_F(MetaDataTest, GetTransformAsMat_ReturnsHomogeneousMatrix) {
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    
    Eigen::Matrix4d mat = meta_.get_transform_as_mat();
    
    // Check bottom row is [0, 0, 0, 1]
    EXPECT_DOUBLE_EQ(mat(3, 0), 0.0);
    EXPECT_DOUBLE_EQ(mat(3, 1), 0.0);
    EXPECT_DOUBLE_EQ(mat(3, 2), 0.0);
    EXPECT_DOUBLE_EQ(mat(3, 3), 1.0);
    
    // Check translation column
    bool to_check = mat.topRightCorner<3, 1>().isApprox(translation);
    EXPECT_TRUE(to_check);
}

// ============================================================================
// get_translation Tests
// ============================================================================

TEST_F(MetaDataTest, GetTranslation_ReturnsCorrectVector) {
    Eigen::Vector3d expected(5.0, -3.0, 1.5);
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    meta_.set_transform(expected, rotation);
    
    Eigen::Vector3d actual = meta_.get_translation();
    
    EXPECT_TRUE(actual.isApprox(expected));
}

// ============================================================================
// get_rotation_as_matrix Tests
// ============================================================================

TEST_F(MetaDataTest, GetRotationAsMatrix_Returns3x3) {
    Eigen::Matrix3d rot = meta_.get_rotation_as_matrix();
    EXPECT_EQ(rot.rows(), 3);
    EXPECT_EQ(rot.cols(), 3);
}

TEST_F(MetaDataTest, GetRotationAsMatrix_IsOrthogonal) {
    Eigen::Vector3d translation(0, 0, 0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
    meta_.set_transform(translation, rotation);
    
    Eigen::Matrix3d R = meta_.get_rotation_as_matrix();
    
    // R * R^T should be identity
    EXPECT_TRUE((R * R.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-9));
    // Determinant should be 1
    EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

// ============================================================================
// get_rotation_as_quat Tests
// ============================================================================

TEST_F(MetaDataTest, GetRotationAsQuat_IsNormalized) {
    Eigen::Vector3d translation(0, 0, 0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d(1, 1, 1).normalized()));
    meta_.set_transform(translation, rotation);
    
    Eigen::Quaterniond result = meta_.get_rotation_as_quat();
    
    EXPECT_NEAR(result.norm(), 1.0, 1e-9);
}

TEST_F(MetaDataTest, GetRotationAsQuat_ConsistentWithMatrix) {
    Eigen::Vector3d translation(0, 0, 0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 5, Eigen::Vector3d::UnitX()));
    meta_.set_transform(translation, rotation);
    
    Eigen::Matrix3d R_from_mat = meta_.get_rotation_as_matrix();
    Eigen::Quaterniond q = meta_.get_rotation_as_quat();
    Eigen::Matrix3d R_from_quat = q.toRotationMatrix();
    
    EXPECT_TRUE(R_from_mat.isApprox(R_from_quat, 1e-9));
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(MetaDataTest, SetTransform_180DegreeRotation) {
    Eigen::Vector3d translation(0, 0, 0);
    Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    meta_.set_transform(translation, rotation);
    
    Eigen::Matrix3d R = meta_.get_rotation_as_matrix();
    
    // 180 degree rotation about Z: x -> -x, y -> -y, z -> z
    EXPECT_NEAR(R(0, 0), -1.0, 1e-9);
    EXPECT_NEAR(R(1, 1), -1.0, 1e-9);
    EXPECT_NEAR(R(2, 2), 1.0, 1e-9);
}

TEST_F(MetaDataTest, CustomResolutionAndDimensions) {
    easy_grid::MetaData custom;
    custom.resolution = 0.05;
    custom.map_width = 200;
    custom.map_height = 150;
    
    EXPECT_DOUBLE_EQ(custom.resolution, 0.05);
    EXPECT_EQ(custom.map_width, 200);
    EXPECT_EQ(custom.map_height, 150);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

