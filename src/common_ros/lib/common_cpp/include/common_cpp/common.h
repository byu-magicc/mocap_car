// Name: Jerel Nielsen
// Date: 15 June 2017
// Desc: Container for common functions.

#ifndef COMMON_H
#define COMMON_H


#include <iostream>
#include <eigen3/Eigen/Eigen>

namespace common
{


class Quaternion
{

public:

  Quaternion();
  ~Quaternion();
  Quaternion(double _w, double _x, double _y, double _z);
  Quaternion(double roll, double pitch, double yaw);
  Quaternion(Eigen::Vector4d v);
  Quaternion(Eigen::Vector3d fz);

  double x;
  double y;
  double z;
  double w;

  Quaternion operator*(const Quaternion &q2);
  friend std::ostream& operator<<(std::ostream &os, const Quaternion &q);
  Quaternion inv();
  double mag();
  void normalize();
  double roll();
  double pitch();
  double yaw();
  void convertFromEigen(Eigen::Vector4d q);
  Eigen::Vector4d convertToEigen();
  Eigen::Matrix3d rot();
  Eigen::Vector3d rotateVector(Eigen::Vector3d v);
  Eigen::Vector3d unitVector();
  Eigen::MatrixXd projection();

};

Eigen::VectorXd rk5(Eigen::VectorXd state, Eigen::VectorXd input, std::function<Eigen::VectorXd(Eigen::VectorXd, Eigen::VectorXd)> ode, double h);
Quaternion exp_q(const Eigen::Vector3d delta);
Eigen::Vector3d log_q(const Quaternion q);
Eigen::Vector3d log_R(const Eigen::Matrix3d R);
common::Quaternion vec2quat(const Eigen::Vector3d v);
Eigen::Vector3d vex(const Eigen::Matrix3d mat);
Eigen::Matrix3d skew(const Eigen::Vector3d vec);
Eigen::Matrix3d R_v2_to_b(double phi);
Eigen::Matrix3d R_v1_to_v2(double theta);
Eigen::Matrix3d R_v_to_v1(double psi);
Eigen::Matrix3d R_v_to_b(double phi, double theta, double psi);
Eigen::Matrix3d R_cb2c();

// Removes elements from Eigen vectors.
// idx: index of first element to remove
// num: number of elements to remove including element located at idx
template<typename ScalarType>
void removeElements(Eigen::Matrix<ScalarType,-1,1,0,-1,1>& vector, unsigned idx, unsigned num)
{
  unsigned new_elements = vector.size()-num; // number of elements in resulting vector
  if (idx < new_elements)
    vector.segment(idx,new_elements-idx) = vector.segment(idx+num,new_elements-idx);
  else
    std::cout << "ERROR: cannot remove requested elements in function 'removeElements()'!\n";
  vector.conservativeResize(new_elements);
}

// Removes rows from Eigen matrices.
// idx: index of first row to remove
// num: number of rows to remove including row located at idx
template<typename ScalarType>
void removeRows(Eigen::Matrix<ScalarType,-1,-1,0,-1,-1>& matrix, unsigned idx, unsigned num)
{
  unsigned new_rows = matrix.rows()-num; // number of rows in resulting matrix
  if (idx < new_rows)
    matrix.block(idx,0,new_rows-idx,matrix.cols()) = matrix.block(idx+num,0,new_rows-idx,matrix.cols());
  else
    std::cout << "ERROR: cannot remove requested rows in function 'removeRows()'!\n";
  matrix.conservativeResize(new_rows,matrix.cols());
}

// Removes columns from Eigen matrices.
// idx: index of first column to remove
// num: number of columns to remove including column located at idx
template<typename ScalarType>
void removeCols(Eigen::Matrix<ScalarType,-1,-1,0,-1,-1>& matrix, unsigned idx, unsigned num)
{
  unsigned new_cols = matrix.cols()-num; // number of columns in resulting matrix
  if (idx < new_cols)
    matrix.block(0,idx,matrix.rows(),new_cols-idx) = matrix.block(0,idx+num,matrix.rows(),new_cols-idx);
  else
    std::cout << "ERROR: cannot remove requested columns in function 'removeCols()'!\n";
  matrix.conservativeResize(matrix.rows(),new_cols);
}

} // namespace common

#endif // COMMON_H
