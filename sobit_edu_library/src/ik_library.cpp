#include "sobit_edu_library/ik_library.hpp"

#include <cmath>
#include <iostream>


namespace sobit_edu{

IKLibrary::IKLibrary()
: //誤差重み行列
  W_E (b.asDiagonal()),//bを対角行列として誤差重み行列とする
  //減衰因子行列
  W_N_bar (b_bar.asDiagonal())//(MatrixXd::Identity(8, 8)*0.001*arm_upper_link*arm_upper_link)
{
}

// ここは，もしも今後逆運動学が3次元に発展したときに，それに対応させるために3次元での順運動学を算出
Pose3d IKLibrary::forward_kinematics(const VectorXd& target_joint_rad_eigen){

  Pose3d final_coord;

  //SOBIT_EDU
  DH_modified Trans_base_Y(0.0, 0.0, 0.0, target_joint_rad_eigen(6));

  DH_modified Trans_base_x((target_joint_rad_eigen(7) + base_to_shoulder_x), 0.0, base_to_shoulder_z, 0.0);

  DH_modified Trans_1(0.0, 0.0, 0.0, target_joint_rad_eigen(JointIds::ArmShoulderRollJoint));
  // std::cout <<"0T1"<< std::endl<< Trans_1.Trans << std::endl;
  DH_modified Trans_2(0.0, -M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmShoulderPitchJoint));
  // std::cout <<"1T2"<< std::endl<< Trans_2.Trans << std::endl;
  DH_modified Trans_2_1(arm_shoulder_derr, 0.0, 0.0, -M_PI_2);
  // std::cout << "2T2_1"<< std::endl << Trans_2_1.Trans << std::endl;
  DH_modified Trans_3(arm_upper_link, 0.0, 0.0, target_joint_rad_eigen(JointIds::ArmElbowPitchJoint));
  // std::cout << "2_1T3"<< std::endl << Trans_3.Trans << std::endl;
  DH_modified Trans_4(0.0, -M_PI_2, arm_elbow_to_lower, target_joint_rad_eigen(JointIds::ArmForearmRollJoint));
  // std::cout << "3T4"<< std::endl << Trans_4.Trans << std::endl;
  DH_modified Trans_5(0.0, M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmWristPitchJoint));
  // std::cout << "4T5"<< std::endl << Trans_5.Trans << std::endl;
  DH_modified Trans_6(0.0, -M_PI_2, 0.0, target_joint_rad_eigen(JointIds::ArmWristRollJoint));
  // std::cout << "5T6"<< std::endl << Trans_6.Trans << std::endl;
  DH_modified Trans_7(0.0, 0.0, arm_gripper_link, 0.0);
  // std::cout << "6TE"<< std::endl << Trans_7.Trans << std::endl;
  DH_modified Trans_8(0.0, -M_PI_2, 0.0, -M_PI_2);
  // std::cout << "E8"<< std::endl << Trans_8.Trans << std::endl;
  DH_modified Trans_9(0.0, -M_PI_2, 0.0, 0.0);
  // std::cout << "88"<< std::endl << Trans_8.Trans << std::endl;

  Matrix4d result = Trans_base_Y.trans_coord*Trans_base_x.trans_coord;
  result = result*Trans_1.trans_coord;
  result = result*Trans_2.trans_coord;
  result = result*Trans_2_1.trans_coord;
  result = result*Trans_3.trans_coord;
  result = result*Trans_4.trans_coord;
  result = result*Trans_5.trans_coord;
  result = result*Trans_6.trans_coord;
  result = result*Trans_7.trans_coord;
  result = result*Trans_8.trans_coord;
  result = result*Trans_9.trans_coord;

  final_coord.position.x() = result(0,3);
  final_coord.position.y() = result(1,3);
  final_coord.position.z() = result(2,3);

  Quaterniond qua_result(result.block<3,3>(0,0));
  // std::cout << qua_result << std::endl;

  final_coord.orientation = qua_result;

  return final_coord;
}

std::vector<double> IKLibrary::inverse_kinematics(
  const Pose3d &goal_coord,  // 'goal_coord' is the coordinates of robot base.
  const std::vector<double> &current_joint_rad)
{
  MatrixXd W_N(8, 8), P_arm(8, 8), H_arm(8, 8), dr(3,3), R(3,3);
  MatrixXd I_p = MatrixXd::Identity(8,8);
  VectorXd e(6), k_p(8), k_s(8), q(8), delta_q(8);

  q << current_joint_rad[0], current_joint_rad[1], current_joint_rad[2], current_joint_rad[3], current_joint_rad[4], current_joint_rad[5], 0.0, 0.0;

  std::cout << "q" << std::endl << q.transpose() << std::endl;

  do{
  cal_e(e, q, goal_coord);//残差計算
  Matrix<double, 6, 8> J_v = cal_Jv(q);
  cal_P(P_arm, I_p, J_v);
  cal_k(k_p, J_v, e);
  cal_ks(k_s, q);
  cal_W_N(W_N, e);
  cal_H(H_arm, J_v, W_N);

  delta_q = H_arm.inverse() * k_p + (P_arm * k_s) ;

  q += delta_q;
  q[3] = std::remainder(q[3], 2*M_PI);
  q[5] = std::remainder(q[5], 2*M_PI);

  if (q[0] >= M_PI/4) q[0] = M_PI/4;
  if (q[0] <= -M_PI/4) q[0] = -M_PI/4;
  if (q[1] >= 3*M_PI/4) q[1] = 3*M_PI/4;
  if (q[1] <= -M_PI_2) q[1] = -M_PI_2;
  if (q[2] >= M_PI_2) q[2] = M_PI_2;
  if (q[2] <= -M_PI_2) q[2] = -M_PI_2;
  if (q[3] >= M_PI) q[3] = M_PI;
  if (q[3] <= -M_PI) q[3] = -M_PI;
  if (q[4] >= M_PI_2) q[4] = M_PI_2;
  if (q[4] <= -M_PI_2) q[4] = -M_PI_2;
  if (q[5] >= M_PI) q[5] = M_PI;
  if (q[5] <= -M_PI) q[5] = -M_PI;
  if (q[6] >= M_PI_2) q[6] = M_PI_2;
  if (q[6] <= -M_PI_2) q[6] = -M_PI_2;

  if (q[7] <= -0.5) q[7] = -0.5;

  iteration++;
  // std::cout << "iteration:" << iteration <<std::endl;
  std::cout << "eの値" << std::endl << e.transpose() << std::endl;
  std::cout << "J(I-P_arm)の値" << std::endl << (J_v* P_arm) << std::endl;

  }while((fabs(e(0)) > threshold_e || fabs(e(1)) > threshold_e || fabs(e(2)) > threshold_e  || fabs(e(3)) > threshold_e  || fabs(e(4)) > threshold_e  || fabs(e(5)) > threshold_e) && (fabs(delta_q(0)) > threshold_q || fabs(delta_q(1)) > threshold_q || fabs(delta_q(2)) > threshold_q || fabs(delta_q(3)) > threshold_q || fabs(delta_q(4)) > threshold_q || fabs(delta_q(5)) > threshold_q || fabs(delta_q(6)) > threshold_q || fabs(delta_q(7)) > threshold_q) && iteration < maxIterations);
  // }while((fabs(e(0)) > threshold_e || fabs(e(1)) > threshold_e || fabs(e(2)) > threshold_e  || fabs(e(3)) > threshold_e  || fabs(e(4)) > threshold_e  || fabs(e(5)) > threshold_e) && iteration < maxIterations);
  // }while(iteration < maxIterations);
  iteration = 0;
  std::vector<double> result_q(q.data(), q.data() + q.size());
  if (iteration >= maxIterations) result_q.clear();

  // std::cout << "result_q" << std::endl << result_q[0] << std::endl
  //                          << std::endl << result_q[1] << std::endl
  //                           << std::endl << result_q[2] << std::endl
  //                            << std::endl << result_q[3] << std::endl
  //                             << std::endl << result_q[4] << std::endl
  //                              << std::endl << result_q[5] << std::endl;
  // q << 0.0,0.0,0.0,0.0,0.0,0.0;

  return result_q;
}

//位置誤差
void IKLibrary::cal_e(VectorXd& e, VectorXd& q_bar, const Pose3d &goal_coord){
  // std::cout << "暫定解q"<< std::endl << q_bar.transpose() << std::endl << "--------------" <<std::endl;

  Pose3d fk = forward_kinematics(q_bar);
  //目標位置と現在の位置の差分を計算
  e(0) = goal_coord.position.x() - fk.position.x();//(arm_shoulder_derr*std::cos(q_bar(0))*std::cos(q_bar(1))+arm_upper_link*std::cos(q_bar(0))*std::sin(q_bar(1))+arm_lower_link*std::cos(q_bar(0))*std::cos(q_bar(1)+q_bar(2)));
  e(1) = goal_coord.position.y() - fk.position.y();//(arm_shoulder_derr*std::sin(q_bar(0))*std::cos(q_bar(1))+arm_upper_link*std::sin(q_bar(0))*std::sin(q_bar(1))+arm_lower_link*std::sin(q_bar(0))*std::cos(q_bar(1)+q_bar(2)));
  e(2) = goal_coord.position.z() - fk.position.z();//(-arm_shoulder_derr*std::sin(q_bar(0))+arm_upper_link*std::cos(q_bar(1))-arm_lower_link*std::sin(q_bar(1)+q_bar(2)));
// 姿勢誤差
  //目標位置と現在野市と差分を計算
  MatrixXd dR(3, 3), R(3, 3), rrq(3, 3);
  VectorXd l(3), m(3), a(3);

  Quaterniond q_dR = fk.orientation;

  Quaterniond q_R = goal_coord.orientation;

  dR = q_dR.toRotationMatrix();

  R = q_R.toRotationMatrix();

  rrq = R * dR.transpose();//姿勢行列

  l(0) = rrq(2,1)-rrq(1,2);
  l(1) = rrq(0,2)-rrq(2,0);
  l(2) = rrq(1,0)-rrq(0,1);

  m(0) = rrq(0,0) + 1;
  m(1) = rrq(1,1) + 1;
  m(2) = rrq(2,2) + 1;

  bool tani = true;
  bool taikaku = true;

  // Rが対角行列
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      if(i != j && fabs(rrq(i, j)) > 1e-12){ 
        // std::cout << "単位行列でも対角行列でもない" <<  rrq(i, j) << std::endl;
        tani = false;
        taikaku = false;
      }
      if(i == j && rrq(i, j) != 1){
        // std::cout << "対角行列" << std::endl; 
        tani = false;
      }
    }
  }

  if(tani){
        // std::cout << "単位行列" << a << std::endl;
            a.setZero();

  }else if(taikaku){
        a = M_PI_2 * m;
        // std::cout << "対角行列" << std::endl << a.transpose() << std::endl;
  }else{
    // std::cout << "lの値" << std::endl << l << std::endl;
    // std::cout << "l.squaredNorm()" << l.squaredNorm() << std::endl;
        a = (atan2(l.squaredNorm(), rrq(0,0)+rrq(1,1)+rrq(2,2)-1) / l.squaredNorm())* l;
        // a = a ;
        // std::cout << "単位行列でも対角行列でもない" <<std::endl << a << std::endl;
  }

  // Rが対角行列でない
  //回転行列の一致性を評価
  e(3) = a(0);
  e(4) = a(1);
  e(5) = a(2);
  return;
}

//基礎ヤコビ行列を計算する関数
Matrix<double, 6,8> IKLibrary::cal_Jv(const VectorXd &current_joint_rad) {
  Matrix<double, 6,8> J;

  Vector3d e_z;
  e_z << 0.0, 0.0, 1.0;
  
    //SOBIT_EDU 手先
          DH_modified Trans_coord_base_Y(0.0, 0.0, 0.0, current_joint_rad(6));

          DH_modified Trans_coord_base_x((current_joint_rad(7) + base_to_shoulder_x), 0.0, base_to_shoulder_z, 0.0);

          DH_modified Trans_coord_1(0.0, 0.0, 0.0, current_joint_rad(JointIds::ArmShoulderRollJoint));

          DH_modified Trans_coord_2(0.0, -M_PI_2, 0.0, current_joint_rad(JointIds::ArmShoulderPitchJoint));

          DH_modified Trans_coord_2_1(arm_shoulder_derr, 0.0, 0.0, -M_PI_2);

          DH_modified Trans_coord_3(arm_upper_link, 0.0, 0.0, current_joint_rad(JointIds::ArmElbowPitchJoint));

          DH_modified Trans_coord_4(0.0, -M_PI_2, arm_elbow_to_lower, current_joint_rad(JointIds::ArmForearmRollJoint));

          DH_modified Trans_coord_5(0.0, M_PI_2, 0.0, current_joint_rad(JointIds::ArmWristPitchJoint));

          DH_modified Trans_coord_6(0.0, -M_PI_2, 0.0, current_joint_rad(JointIds::ArmWristRollJoint));
         
          DH_modified Trans_coord_7(0.0, 0.0, arm_gripper_link, 0.0);
     
          DH_modified Trans_coord_8(0.0, -M_PI_2, 0.0, -M_PI_2);
  
          DH_modified Trans_coord_9(0.0, -M_PI_2, 0.0, 0.0);
  
          Matrix4d result_base_Y = Trans_coord_base_Y.trans_coord;
          Matrix4d result_base_x = result_base_Y*Trans_coord_base_x.trans_coord;
          Matrix4d result_1 = result_base_x*Trans_coord_1.trans_coord;
          Matrix4d result_2 = result_1*Trans_coord_2.trans_coord;
          Matrix4d result_2_1 = result_2*Trans_coord_2_1.trans_coord;
          Matrix4d result_3 = result_2_1*Trans_coord_3.trans_coord;
          Matrix4d result_4 = result_3*Trans_coord_4.trans_coord;
          Matrix4d result_5 = result_4*Trans_coord_5.trans_coord;
          Matrix4d result_6 = result_5*Trans_coord_6.trans_coord;
          Matrix4d result_7 = result_6*Trans_coord_7.trans_coord;
          Matrix4d result_8 = result_7*Trans_coord_8.trans_coord;
          Matrix4d result_9 = result_8*Trans_coord_9.trans_coord;          

          Vector3d p_6_E = result_9.block<3,1>(0,3) - result_6.block<3,1>(0,3);
          Vector3d p_5_E = result_9.block<3,1>(0,3) - result_5.block<3,1>(0,3);
          Vector3d p_4_E = result_9.block<3,1>(0,3) - result_4.block<3,1>(0,3);
          Vector3d p_3_E = result_9.block<3,1>(0,3) - result_3.block<3,1>(0,3);
          Vector3d p_2_E = result_9.block<3,1>(0,3) - result_2.block<3,1>(0,3);
          Vector3d p_1_E = result_9.block<3,1>(0,3) - result_1.block<3,1>(0,3);
          Vector3d p_base_Y_E = result_9.block<3,1>(0,3) - result_base_Y.block<3,1>(0,3);


          
          Vector3d z_base_Y = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_base_x(std::cos(current_joint_rad(6)), std::sin(current_joint_rad(6)), 0.0);

          Vector3d z_1 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_2 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_3 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_4 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_5 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * Trans_coord_5.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d z_6 = Trans_coord_base_Y.trans_coord.block<3,3>(0,0) * Trans_coord_base_x.trans_coord.block<3,3>(0,0) * Trans_coord_1.trans_coord.block<3,3>(0,0) * Trans_coord_2.trans_coord.block<3,3>(0,0) * Trans_coord_2_1.trans_coord.block<3,3>(0,0) * Trans_coord_3.trans_coord.block<3,3>(0,0) * Trans_coord_4.trans_coord.block<3,3>(0,0) * Trans_coord_5.trans_coord.block<3,3>(0,0) * Trans_coord_6.trans_coord.block<3,3>(0,0) * e_z;

          Vector3d p_base_Y = z_base_Y.cross(p_base_Y_E);

          Vector3d p_base_x = z_base_x;

          Vector3d p_1 = z_1.cross(p_1_E);

          Vector3d p_2 = z_2.cross(p_2_E);

          Vector3d p_3 = z_3.cross(p_3_E);

          Vector3d p_4 = z_4.cross(p_4_E);

          Vector3d p_5 = z_5.cross(p_5_E);

          Vector3d p_6 = z_6.cross(p_6_E);

          Matrix<double, 3, 8> J_pos;
          J_pos.col(0) = p_1;
          J_pos.col(1) = p_2;
          J_pos.col(2) = p_3; 
          J_pos.col(3) = p_4;
          J_pos.col(4) = p_5;
          J_pos.col(5) = p_6;
          J_pos.col(6) = p_base_Y;
          J_pos.col(7) = p_base_x;


          Matrix<double, 3, 8> J_rot;
          J_rot.col(0) = z_1;
          J_rot.col(1) = z_2;
          J_rot.col(2) = z_3;
          J_rot.col(3) = z_4;
          J_rot.col(4) = z_5;
          J_rot.col(5) = z_6;
          J_rot.col(6) = z_base_Y;
          J_rot.col(7) = Vector3d::Zero();

          Matrix<double, 6, 8> J_prot;
          J << J_pos, J_rot;

    return J;
}

//ヌル空間射影オペレータを計算する関数
void IKLibrary::cal_P(MatrixXd& P, const MatrixXd& I,const MatrixXd& J) {
    P = I - (J.transpose() * (J * J.transpose()).inverse() * J);
    return;
}

//勾配k(疑似行列の任意ベクトル)を計算する関数
void IKLibrary::cal_k(VectorXd& k, const MatrixXd& J, const VectorXd& e) {
    k = J.transpose() * W_E * e;
    return;
}

//
void IKLibrary::cal_ks(VectorXd& ks, VectorXd& q){
    ks = 1.0 * (q_d - q);
}

//減衰因子行列を計算する関数
void IKLibrary::cal_W_N(MatrixXd &W_N, const VectorXd &e) {
    W_N = 0.5 * (e.transpose() * W_E * e)(0,0) * MatrixXd::Identity(8,8)+W_N_bar;
    return;
}

void IKLibrary::cal_H(MatrixXd &H, const MatrixXd &J, const MatrixXd &W_N){
    H = J.transpose() * W_E * J + W_N;
    return;
}

} // namespace sobit_edu
