// Common.hpp
// structures defined for RaiSim and ROS data transmission
// Du Jianrui   2023.2.4

#pragma once

#include <Eigen/Dense>
#include "raisim/World.hpp"

#include <random>
#include <chrono>
#include <cmath>

struct RobotState
{
  raisim::Vec<3> velocity;
  raisim::Vec<3> angularVelocity;
  raisim::Vec<3> position;
  raisim::Vec<4> quaternion;
};

struct QuadrotorCommand
{
  double thrust;
  raisim::Vec<3> moment;
};

struct AvaRobotState
{
  Eigen::Matrix3d rotation0_w; // rotation matrix from base to world
  Eigen::Matrix3d rotation1_0; // rotation matrix from child drone 1 to joint1
  Eigen::Matrix3d rotation2_0; // rotation matrix from child drone 2 to joint2
  Eigen::Matrix3d rotation3_0; // rotation matrix from child drone 3 to joint3

  Eigen::Vector3d eulerAngle0; // Euler angle of the center's
  Eigen::Vector3d eulerAngle1; // Euler angle of child drone 1's
  Eigen::Vector3d eulerAngle2; // Euler angle of child drone 2's
  Eigen::Vector3d eulerAngle3; // Euler angle of child drone 3's

  Eigen::Vector3d position0; // position of the center's in NED
  Eigen::Vector3d position1; // position of child drone 1's in NED
  Eigen::Vector3d position2; // position of child drone 2's in NED
  Eigen::Vector3d position3; // position of child drone 3's in NED

  Eigen::Vector3d bodyRate0; // body rate of the center's
  Eigen::Vector3d bodyRate1; // body rate of child drone 1's
  Eigen::Vector3d bodyRate2; // body rate of child drone 2's
  Eigen::Vector3d bodyRate3; // body rate of child drone 3's

  Eigen::Vector3d velocity0; // velocity of the center's in body-fixed front-right-down
  Eigen::Vector3d velocity1; // velocity of child drone 1's in body-fixed front-right-down
  Eigen::Vector3d velocity2; // velocity of child drone 2's in body-fixed front-right-down
  Eigen::Vector3d velocity3; // velocity of child drone 3's in body-fixed front-right-down

  Eigen::Vector3d linearAcceleration0; // linear acceleration of the center's in body-fixed front-right-down
  Eigen::Vector3d linearAcceleration1; // linear acceleration of child drone 1's in body-fixed front-right-down
  Eigen::Vector3d linearAcceleration2; // linear acceleration of child drone 1's in body-fixed front-right-down
  Eigen::Vector3d linearAcceleration3; // linear acceleration of child drone 1's in body-fixed front-right-down

  size_t toolIndex; // the tool index acquired by getBodyIdx("base_link")
};

struct AvaRobotCommand
{
  double thrust1; // thrust command of child drone 1's, a positive variable
  double thrust2; // thrust command of child drone 2's, a positive variable
  double thrust3; // thrust command of child drone 3's, a positive variable
  Eigen::Vector3d moment1; // moment command of child drone 1's in body-fixed front-right-down
  Eigen::Vector3d moment2; // moment command of child drone 2's in body-fixed front-right-down
  Eigen::Vector3d moment3; // moment command of child drone 3's in body-fixed front-right-down
};

struct Imu
{
  double deltaTime;
  double sqrtDt;

  // random number generator parameters
  double meanOmega = 0.0;
  double sdOmega = 1.0;
  double meanAcc = 0.0;
  double sdAcc = 1.0;

  // noise parameters
  double arw;
  double sigmaArw;
  double arn;
  double sigmaBiasA;
  double vrw;
  double sigmaVrw;
  double vrn;
  double sigmaBiasV;

  Eigen::Vector3d gyrBias;
  Eigen::Vector3d accBias;
  Eigen::Vector3d dataOmega;
  Eigen::Vector3d dataAcc;

  Imu(double _deltaTime,
      double _arw = 0.0015, double _arn = 1.0e-6, double _vrw = 0.02, double _vrn = 0.0001,
      double _meanOmega = 0.0, double _sdOmega = 1.0, double _meanAcc = 0.0, double _sdAcc = 1.0)
    : deltaTime(_deltaTime), arw(_arw), arn(_arn), vrw(_vrw), vrn(_vrn),
      meanOmega(_meanOmega), sdOmega(_sdOmega), meanAcc(_meanAcc), sdAcc(_sdAcc)
  {
    sqrtDt = sqrt(_deltaTime);
    sigmaArw = arw / sqrtDt;
    sigmaBiasA = arn * sqrtDt;
    sigmaVrw = vrw / sqrtDt;
    sigmaBiasV = vrn * sqrtDt;
  }

  void setParameters(double _arw, double _arn, double _vrw, double _vrn,
                     double _meanOmega, double _sdOmega, double _meanAcc, double _sdAcc)
  {
    arw = _arw;
    arn = _arn;
    vrw = _vrw;
    vrn = _vrn;
    meanOmega = _meanOmega;
    sdOmega = _sdOmega;
    meanAcc = _meanAcc;
    sdAcc = _sdAcc;

    sigmaArw = arw / sqrtDt;
    sigmaBiasA = arn * sqrtDt;
    sigmaVrw = vrw / sqrtDt;
    sigmaBiasV = vrn * sqrtDt;
  }

  void updateDataWithNoise(const Eigen::Vector3d& _rawOmega, const Eigen::Vector3d& _rawAcc)
  {
    updateOmegaWithNoise(_rawOmega);
    updateAccWithNoise(_rawAcc);
  }

  void updateOmegaWithNoise(const Eigen::Vector3d& _rawOmega)
  {
    dataOmega.x() = _rawOmega.x() + sigmaArw * getGaussianNumber(meanOmega, sdOmega) + gyrBias.x();
    dataOmega.y() = _rawOmega.y() + sigmaArw * getGaussianNumber(meanOmega, sdOmega) + gyrBias.y();
    dataOmega.z() = _rawOmega.z() + sigmaArw * getGaussianNumber(meanOmega, sdOmega) + gyrBias.z();

    gyrBias.x() += sigmaBiasA * getGaussianNumber(meanOmega, sdOmega);
    gyrBias.y() += sigmaBiasA * getGaussianNumber(meanOmega, sdOmega);
    gyrBias.z() += sigmaBiasA * getGaussianNumber(meanOmega, sdOmega);
  }

  void updateAccWithNoise(const Eigen::Vector3d& _rawAcc)
  {
    dataAcc.x() = _rawAcc.x() + sigmaVrw * getGaussianNumber(meanAcc, sdAcc) + accBias.x();
    dataAcc.y() = _rawAcc.y() + sigmaVrw * getGaussianNumber(meanAcc, sdAcc) + accBias.y();
    dataAcc.z() = _rawAcc.z() + sigmaVrw * getGaussianNumber(meanAcc, sdAcc) + accBias.z();

    accBias.x() += sigmaBiasV * getGaussianNumber(meanAcc, sdAcc);
    accBias.y() += sigmaBiasV * getGaussianNumber(meanAcc, sdAcc);
    accBias.z() += sigmaBiasV * getGaussianNumber(meanAcc, sdAcc);
  }

  double getGaussianNumber(double _mean, double _standardDeviation)
  {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(_mean, _standardDeviation);

    return distribution(generator);
  }
};
