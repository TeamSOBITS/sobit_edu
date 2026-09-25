#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>


namespace sobit_edu
{

using namespace Eigen;

// FK/IKで使う関節ベクトルの添字は，q[6]がベース旋回(yaw)，q[7]がベース前進量(slide)であり，
// JointIds::HandJoint / HeadCameraPanJoint とは異なる
enum JointIds
{
  ArmShoulderRollJoint,
  ArmShoulderPitchJoint,
  ArmElbowPitchJoint,
  ArmForearmRollJoint,
  ArmWristPitchJoint,
  ArmWristRollJoint,
  HandJoint,
  HeadCameraPanJoint,
  HeadCameraTiltJoint,
  JointNum
};

//修正DHパラメータ
struct DH_modified
{
  double A;
  double Alpha;
  double D;
  double Theta;
  Matrix4d trans_coord;

  DH_modified(double a, double alpha, double d, double zero) : A(a), Alpha(alpha), D(d), Theta(zero), trans_coord(( Eigen::Matrix4d()<<
    std::cos(Theta), -std::sin(Theta), 0.0, A,
    std::cos(Alpha)*std::sin(Theta), std::cos(Alpha)*std::cos(Theta), -std::sin(Alpha), -std::sin(Alpha)*D,
    std::sin(Alpha)*std::sin(Theta), std::sin(Alpha)*std::cos(Theta), std::cos(Alpha), std::cos(Alpha)*D,
    0.0, 0.0, 0.0, 1.0).finished()){}
    // Matrix3d getRotation() const { return matrix.block<3, 3>(0, 0); }
    // Vector3d getTranslation() const { return matrix.block<3, 1>(0, 3); }

};

// 手先の姿勢(位置 + クォータニオン)。ROSのメッセージ型に依存しないので，
// ROSメッセージ(TransformStamped等)との変換は呼び出し側(ノード)で行う
struct Pose3d
{
  Vector3d position = Vector3d::Zero();
  Quaterniond orientation = Quaterniond::Identity();
};

class IKLibrary
{
public:
  IKLibrary();

  Pose3d forward_kinematics(
    const VectorXd &target_joint_rad_eigen);  // target_yaw should be eliminated in the future.
  std::vector<double> inverse_kinematics(
    const Pose3d &goal_coord,
    const std::vector<double> &current_joint_rad);  // target_yaw should be eliminated in the future.

private:
  static constexpr double base_to_shoulder_x = 0.12561;
  static constexpr double base_to_shoulder_z = 0.55505;
  static constexpr double arm_upper_link = 0.128;
  static constexpr double arm_lower_link = 0.146;
  static constexpr double arm_gripper_link = 0.1834;
  static constexpr double arm_shoulder_derr = 0.022;
  static constexpr double arm_elbow_to_lower = arm_lower_link - arm_shoulder_derr;

  //基礎ヤコビ行列を計算する関数
  Matrix<double, 6,8> cal_Jv(const VectorXd &current_joint_rad);
  //x, y, z, row, pitch, yawの誤差重み行列の対角要素
  VectorXd b{{0.1, 0.1, 1000.0, 10.0, 10.0, 10.0}};
  //更新則の減衰行列
  double bias = 0.1*arm_upper_link*arm_upper_link;
  VectorXd b_bar{{bias, bias, bias, bias, bias, bias, bias*10, 0.001*arm_upper_link*arm_upper_link}};

  VectorXd q_d {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  MatrixXd W_E;
  MatrixXd W_N_bar;

  // eの収束条件
  double threshold_e = 1e-10;
  // qの収束条件
  double threshold_q = 1e-10;
  // 最大反復回数
  int maxIterations = 15000;
  // 反復回数
  int iteration = 0;

  //誤差を計算
  void cal_e(VectorXd& e, VectorXd& q, const Pose3d &goal_coord);

  // main勾配kを計算する関数
  void cal_k(VectorXd& k, const MatrixXd& J, const VectorXd& e);

  // ヌル空間射影オペレータを計算する関数
  void cal_P(MatrixXd& P, const MatrixXd& I,const MatrixXd& J);

  // 勾配ksを計算する関数
  void cal_ks(VectorXd& ks, VectorXd& q);

  // 減衰因子行列を計算する関数
  void cal_W_N(MatrixXd& W_N, const VectorXd& e);

  // 疑似ヤコビ行列を計算する関数
  void cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N);

}; // class IKLibrary

} // namespace sobit_edu
