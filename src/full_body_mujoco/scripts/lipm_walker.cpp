// lipm_walker.cpp
// LIPM walking controller for the Angad full-body humanoid running in MuJoCo.
// Architecture ported entirely from:
//   humanoid_ws/src/lipm_control  (LIPMControl, ZMPDistributor, postureStabilizer, LeakyIntegrator)
//   humanoid_ws/src/lipm_motion   (LIPMDynamics, LIPMPlanner, zmpPlanner)
// The two-node pipeline (lipm_ros + control) is collapsed into one ROS2 node.
// boost::circular_buffer replaced with std::deque; KMath replaced with Eigen.

#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace Eigen;

// ═══════════════════════════════════════════════════════════════════════════════
// LeakyIntegrator  — humanoid_ws/lipm_control/LeakyIntegrator.h + .cpp
// ═══════════════════════════════════════════════════════════════════════════════
class LeakyIntegrator {
public:
  LeakyIntegrator() : integral_(Vector3d::Zero()), rate_(0.0), saturation_(false) {}

  void add(const Vector3d &value, double dt) {
    integral_ = (1.0 - rate_ * dt) * integral_ + value * dt;
    if (saturation_) saturate();
  }
  const Vector3d &eval() const { return integral_; }
  void setZero() { integral_.setZero(); }
  void setInitialState(const Vector3d &v) { integral_ = v; }
  void setRate(double rate) { rate_ = rate; }
  double rate() const { return rate_; }
  void setSaturation(const Vector3d &u_sat, const Vector3d &l_sat) {
    saturation_ = true; u_sat_ = u_sat; l_sat_ = l_sat;
  }

private:
  void saturate() {
    integral_ = integral_.cwiseMax(l_sat_).cwiseMin(u_sat_);
  }
  Vector3d integral_, u_sat_, l_sat_;
  double rate_;
  bool saturation_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// LIPMControl  — humanoid_ws/lipm_control/LIPMControl.h
// ═══════════════════════════════════════════════════════════════════════════════
class LIPMControl {
  Vector3d CoM_c, dCoM_c, ddCoM_c, dCoM_c_;
  Vector3d ZMP_d, ZMP_c, ZMPFeedback;
  Vector3d DCM_ref, dDCM_ref, DCM, dDCM;
  Vector3d sumDCM;
  LeakyIntegrator liZMP;
  double g, h, dt;
  bool initialized;

public:
  Matrix3d Kzmp, Kdcm, Kcom, Kddcm, Kidcm;
  double omega;

  LIPMControl(double h_, double dt_,
              double Pcom_x=0.02, double Pcom_y=0.01,
              double Pzmp_x=7,    double Pzmp_y=7,
              double Pdcm_x=40,   double Pdcm_y=40,
              double Idcm_x=10,   double Idcm_y=10)
      : h(h_), dt(dt_), g(9.81), initialized(false) {
    omega = std::sqrt(g / h);

    Kzmp.setZero(); Kcom.setZero(); Kdcm.setZero(); Kidcm.setZero(); Kddcm.setZero();
    sumDCM.setZero(); ZMPFeedback.setZero();
    ddCoM_c.setZero(); dCoM_c_.setZero(); DCM.setZero();
    DCM_ref.setZero(); dDCM_ref.setZero(); dDCM.setZero();

    Kzmp(0,0) = Pzmp_x / omega;
    Kzmp(1,1) = Pzmp_y / omega;
    Kcom(0,0) = Pcom_x;
    Kcom(1,1) = Pcom_y;
    Kdcm(0,0) = 1.0 + Pdcm_x / omega;
    Kdcm(1,1) = 1.0 + Pdcm_y / omega;
    Kddcm(0,0) = 1.0 / omega;
    Kddcm(1,1) = 1.0 / omega;
    Kidcm(0,0) = Idcm_x / omega;
    Kidcm(1,1) = Idcm_y / omega;
  }

  void Control(Vector3d ZMP, Vector3d CoM, Vector3d dCoM, Vector3d ddCoM,
               Vector3d CoM_d, Vector3d dCoM_d, Vector3d ddCoM_d) {
    if (!initialized) {
      CoM_c = CoM_d; dCoM_c = dCoM_d; ddCoM_c = ddCoM_d;
      initialized = true;
      return;
    }
    ZMP_d = CoM_d - ddCoM_d / (omega * omega);
    ZMP_d(2) = ZMP(2);

    DCM_ref = computeDCM(CoM_d, dCoM_d);
    dDCM_ref = computeDCM(dCoM_d, ddCoM_d);
    DCM = computeDCM(CoM, dCoM);
    dDCM = computeDCM(dCoM, ddCoM);

    ZMP_c = ZMP_d - Kdcm * (DCM_ref - DCM) + Kzmp * (ZMP_d - ZMP) - Kidcm * sumDCM;
    sumDCM += (DCM_ref - DCM) * dt;

    ZMPFeedback = Kcom * (ZMP - ZMP_c);
    dCoM_c = dCoM_d + ZMPFeedback;
    ddCoM_c = (dCoM_c - dCoM_c_) / dt;
    dCoM_c_ = dCoM_c;
    liZMP.add(ZMPFeedback, dt);
    CoM_c = liZMP.eval() + CoM_d;
  }

  Vector3d computeDCM(Vector3d c, Vector3d dc) const { return c + (1.0 / omega) * dc; }
  Vector3d getDesiredCoMPosition()     const { return CoM_c; }
  Vector3d getDesiredCoMVelocity()     const { return dCoM_c; }
  Vector3d getDesiredCoMAcceleration() const { return ddCoM_c; }
  Vector3d getDesiredZMP()             const { return ZMP_c; }
};

// ═══════════════════════════════════════════════════════════════════════════════
// LIPMDynamics  — humanoid_ws/lipm_motion/LIPMDynamics.cpp
// ═══════════════════════════════════════════════════════════════════════════════
class LIPMDynamics {
  Vector4d x;
  double omega_, dt_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Matrix4d A, Ad, I;
  Vector4d B, Bd, C, Cd;
  double com, zmp, vcom, acom;

  LIPMDynamics() {
    x.setZero(); A.setZero(); B.setZero(); C.setZero();
    I.setIdentity(); Ad.setZero(); Bd.setZero(); Cd.setZero();
  }
  void setParams(double omega, double dt) { omega_ = omega; dt_ = dt; }
  void init() {
    A(0,1) = 1; A(1,2) = 1; A(3,1) = 1;
    Ad = A * dt_ + I;
    B(2) = 1.0; B(3) = -1.0 / (omega_ * omega_);
    Bd = B * dt_;
    C(3) = 1.0; Cd = C;
  }
  void setState(const Vector4d &x_) { x = x_; }
  void integrate(double u) {
    x = Ad * x + Bd * u;
    com = x(0); vcom = x(1); acom = x(2); zmp = x(3);
  }
  Vector4d getState() const { return x; }
};

// ═══════════════════════════════════════════════════════════════════════════════
// LIPMPlanner  — humanoid_ws/lipm_motion/LIPMPlanner.cpp
//   boost::circular_buffer replaced with std::deque
// ═══════════════════════════════════════════════════════════════════════════════
class LIPMPlanner {
  LIPMDynamics LIPMDynamicsX, LIPMDynamicsY;
  Vector4d x, y, x_, y_;
  Matrix4d A;
  Vector4d B, C;
  MatrixXd Fv, Fvu, R, Qv, H, tmpb, H_inv, I_np, Gx, Gp;
  VectorXd ZMPRefX, ZMPRefY, temp, U_x, U_y;
  double comZ_, g_, dt_, omega_, rv_, qv_, u_x, u_y;
  int Np_;
  bool planAvailable_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double ZMPx_d, comx_d, comdx_d, dcmx_d, dcmdx_d;
  double ZMPy_d, comy_d, comdy_d, dcmy_d, dcmdy_d;
  double comddx_d, comddy_d;

  std::deque<VectorXd> CoMBuffer, DCMBuffer, VRPBuffer;

  LIPMPlanner() : u_x(0), u_y(0), planAvailable_(false) {
    x.setZero(); y.setZero(); x_.setZero(); y_.setZero();
  }

  void setParams(double comZ, double g, double dt, double q, double r, int Np) {
    comZ_ = comZ; g_ = g; dt_ = dt; qv_ = q; rv_ = r; Np_ = Np;
    omega_ = std::sqrt(g / comZ);
    LIPMDynamicsX.setParams(omega_, dt);
    LIPMDynamicsY.setParams(omega_, dt);
  }

  void init() {
    Fv.resize(Np_, 4);   Fv.setZero();
    Fvu.resize(Np_, Np_); Fvu.setZero();
    tmpb.resize(1, Np_-1);
    temp.resize(4);
    R.resize(Np_, Np_);   Qv.resize(Np_, Np_);
    H.resize(Np_, Np_);   H_inv.resize(Np_, Np_);
    I_np.resize(Np_, Np_); I_np.setIdentity();
    U_x.resize(Np_); U_y.resize(Np_);
    Gx.resize(Np_, 4); Gp.resize(Np_, Np_);
    ZMPRefX.resize(Np_); ZMPRefY.resize(Np_);
    ZMPRefX.setZero(); ZMPRefY.setZero();

    LIPMDynamicsX.init(); LIPMDynamicsY.init();

    A = LIPMDynamicsX.Ad; B = LIPMDynamicsX.Bd; C = LIPMDynamicsX.Cd;

    Fv.block(0,0,1,4) = C.transpose() * A;
    Fvu(0,0) = C.transpose() * B;
    temp = B;

    for (int i = 1; i < Np_; i++) {
      Fv.block(i,0,1,4) = Fv.block(i-1,0,1,4) * A;
      tmpb = Fvu.block(i-1,0,1,Np_-1);
      Fvu.block(i,1,1,Np_-1) = tmpb;
      temp = A * temp;
      Fvu(i,0) = C.transpose() * temp;
    }

    R  = I_np * rv_;
    Qv = I_np * qv_;
    H  = R * Qv.llt().solve(I_np);
    H.noalias() += Fvu.transpose() * Fvu;
    H_inv = H.llt().solve(I_np);
    Gp = -H_inv * Fvu.transpose();
    Gx = Gp * Fv;
    planAvailable_ = false;
  }

  void setState(Vector2d CoM, Vector2d vCoM, Vector2d ZMP) {
    u_x = u_y = 0.0;
    LIPMDynamicsX.setState(Vector4d(CoM(0), vCoM(0), 0, ZMP(0)));
    LIPMDynamicsY.setState(Vector4d(CoM(1), vCoM(1), 0, ZMP(1)));
  }

  void plan(std::deque<VectorXd> &ZMPRef) {
    CoMBuffer.clear(); DCMBuffer.clear(); VRPBuffer.clear();
    VectorXd CoM_d(9), DCM_d(6), ZMP_d(3);

    while (!ZMPRef.empty()) {
      for (int i = 0; i < Np_; i++) {
        size_t idx = std::min((size_t)i+1, ZMPRef.size()-1);
        ZMPRefX(i) = ZMPRef[idx](0);
        ZMPRefY(i) = ZMPRef[idx](1);
      }
      x = LIPMDynamicsX.getState();
      y = LIPMDynamicsY.getState();

      U_x = Gx * x - Gp * ZMPRefX;
      U_y = Gx * y - Gp * ZMPRefY;
      u_x = U_x(0); u_y = U_y(0);

      LIPMDynamicsX.integrate(u_x);
      LIPMDynamicsY.integrate(u_y);
      x = LIPMDynamicsX.getState();
      y = LIPMDynamicsY.getState();

      comx_d  = x(0); comy_d  = y(0);
      comdx_d = x(1); comdy_d = y(1);
      ZMPx_d  = x(3); ZMPy_d  = y(3);

      dcmx_d  = comx_d  + comdx_d  / omega_;
      dcmy_d  = comy_d  + comdy_d  / omega_;
      comddx_d = omega_ * omega_ * (comx_d  - ZMPx_d);
      comddy_d = omega_ * omega_ * (comy_d  - ZMPy_d);
      dcmdx_d = comdx_d  + comddx_d / omega_;
      dcmdy_d = comdy_d  + comddy_d / omega_;

      CoM_d << comx_d,comy_d, comZ_+ZMPRef.front()(2),
               comdx_d,comdy_d,0.0,
               comddx_d,comddy_d,0.0;
      DCM_d << dcmx_d,dcmy_d,CoM_d(2),
               dcmdx_d,dcmdy_d,0.0;
      ZMP_d << ZMPx_d,ZMPy_d,ZMPRef.front()(2);

      CoMBuffer.push_back(CoM_d);
      DCMBuffer.push_back(DCM_d);
      VRPBuffer.push_back(ZMP_d);
      ZMPRef.pop_front();
    }
    planAvailable_ = true;
  }

  bool planAvailable() const { return planAvailable_; }
  void emptyPlan() {
    CoMBuffer.clear(); DCMBuffer.clear(); VRPBuffer.clear();
    planAvailable_ = false;
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// ZMPDistributor  — humanoid_ws/lipm_control/ZMPDistributor.cpp
// ═══════════════════════════════════════════════════════════════════════════════
class ZMPDistributor {
  double maxForceL_, maxForceR_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3d frd, fld, tauld, taurd, tau0;
  double a;

  ZMPDistributor(double mass, double g) : a(0.0) {
    frd.setZero(); fld.setZero();
    tauld.setZero(); taurd.setZero(); tau0.setZero();
    maxForceL_ = maxForceR_ = mass * g;
  }

  void computeDistribution(Vector3d pZMPd, Vector3d pZMP,
                           Vector3d pL, Vector3d pR,
                           bool right_support, bool double_support) {
    tauld.setZero(); taurd.setZero(); tau0.setZero();
    fld.setZero();   frd.setZero();

    if (!double_support) {
      a = right_support ? 1.0 : 0.0;
    } else {
      Vector3d vLR   = pR - pL;
      Vector3d vLZMPd = pZMPd - pL;
      double denom = vLR.dot(vLR);
      if (denom < 1e-9) { a = 0.5; }
      else {
        Vector3d pa = pL + vLR * (vLZMPd.dot(vLR) / denom);
        a = (pa - pL).norm() / ((pL - pR).norm() + 1e-9);
        a = std::min(1.0, std::max(0.0, a));
      }
    }

    frd(2) = a * maxForceR_;
    fld(2) = (1.0 - a) * maxForceL_;
    tau0   = -(pR - pZMP).cross(frd) - (pL - pZMP).cross(fld);
    taurd  = a * tau0;
    tauld  = (1.0 - a) * tau0;
    if (tau0(0) < 0) { taurd(0) = tau0(0); tauld(0) = 0; }
    else             { tauld(0) = tau0(0); taurd(0) = 0; }
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// postureStabilizer  — humanoid_ws/lipm_control/postureStabilizer.cpp
// ═══════════════════════════════════════════════════════════════════════════════
class postureStabilizer {
  double dbase_Roll, dbase_Pitch;
  double dL_Roll, dL_Pitch, dR_Roll, dR_Pitch;
  double dLz, dRz, dz;
  Matrix3d dbase;
  double dt_, Tc_, Kc_, Ta_, Ka_, Tn_, Kn_;

  static double sinc_inv(double x) {
    constexpr double eps  = std::numeric_limits<double>::epsilon();
    constexpr double eps2 = std::sqrt(std::numeric_limits<double>::epsilon());
    constexpr double eps4 = std::sqrt(eps2);
    if (std::abs(x) >= eps4) return x / std::sin(x);
    double r = 1.0;
    if (std::abs(x) >= eps)  { double x2 = x*x; r += x2/6.0;
      if (std::abs(x) >= eps2) r += 7.0*x2*x2/360.0; }
    return r;
  }

  static Vector3d logMap(const Matrix3d &R) {
    double acosV = (R(0,0)+R(1,1)+R(2,2)-1.0)*0.5;
    double theta = std::acos(std::min(1.0,std::max(-1.0,acosV)));
    Vector3d w(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
    return w * sinc_inv(theta) * 0.5;
  }

  static Matrix3d expMap(const Vector3d &v) {
    Matrix3d res = Matrix3d::Identity();
    double n = v.norm();
    if (n > std::numeric_limits<double>::epsilon()) {
      Matrix3d sk; sk.setZero();
      sk(0,1)=-v(2); sk(0,2)=v(1); sk(1,2)=-v(0);
      sk(1,0)=v(2);  sk(2,0)=-v(1); sk(2,1)=v(0);
      res += sk*(std::sin(n)/n) + sk*sk*((1.0-std::cos(n))/(n*n));
    }
    return res;
  }

  static Matrix3d rotFromEuler(double roll, double pitch, double yaw) {
    double su=std::sin(roll),cu=std::cos(roll);
    double sv=std::sin(pitch),cv=std::cos(pitch);
    double sw=std::sin(yaw),cw=std::cos(yaw);
    Matrix3d R;
    R << cv*cw, su*sv*cw-cu*sw, su*sw+cu*sv*cw,
         cv*sw, cu*cw+su*sv*sw, cu*sv*sw-su*cw,
         -sv,   su*cv,          cu*cv;
    return R;
  }

public:
  postureStabilizer(double dt,double Kc,double Tc,double Ka,double Ta,double Kn,double Tn)
      : dt_(dt),Tc_(Tc),Kc_(Kc),Ta_(Ta),Ka_(Ka),Tn_(Tn),Kn_(Kn) {
    dbase_Roll=dbase_Pitch=dL_Roll=dL_Pitch=dR_Roll=dR_Pitch=dLz=dRz=dz=0.0;
    dbase.setIdentity();
  }

  void footTorqueStabilizer(Vector3d tauld, Vector3d taurd,
                             Vector3d taul,  Vector3d taur,
                             bool right_contact, bool left_contact) {
    if (left_contact) {
      dL_Roll  = Ka_*dt_*(tauld(0)-taul(0)) + (1.0-dt_/Ta_)*dL_Roll;
      dL_Pitch = Ka_*dt_*(tauld(1)-taul(1)) + (1.0-dt_/Ta_)*dL_Pitch;
    } else { dL_Roll=dL_Pitch=0.0; }
    if (right_contact) {
      dR_Roll  = Ka_*dt_*(taurd(0)-taur(0)) + (1.0-dt_/Ta_)*dR_Roll;
      dR_Pitch = Ka_*dt_*(taurd(1)-taur(1)) + (1.0-dt_/Ta_)*dR_Pitch;
    } else { dR_Roll=dR_Pitch=0.0; }
  }

  void footForceStabilizer(double flz, double frz, double flz_d, double frz_d) {
    dz = Kn_*dt_*((flz-frz)-(flz_d-frz_d)) + (1.0-dt_/Tn_)*dz;
    dLz = -0.5*dz; dRz = 0.5*dz;
  }

  void baseOrientationStabilizer(Quaterniond qm, Quaterniond qref) {
    Vector3d err = logMap((qm.inverse()*qref).toRotationMatrix());
    dbase_Roll  = Kc_*dt_*err(0) + (1.0-dt_/Tc_)*dbase_Roll;
    dbase_Pitch = Kc_*dt_*err(1) + (1.0-dt_/Tc_)*dbase_Pitch;
    dbase = expMap(Vector3d(dbase_Roll, dbase_Pitch, 0));
  }

  Quaterniond getBaseOrientation()      { return Quaterniond(dbase); }
  Quaterniond getRightFootOrientation() { return Quaterniond(rotFromEuler(dR_Roll,dR_Pitch,0)); }
  Quaterniond getLeftFootOrientation()  { return Quaterniond(rotFromEuler(dL_Roll,dL_Pitch,0)); }
  Vector3d getRightFootVerticalPosition() { return Vector3d(0,0,dRz); }
  Vector3d getLeftFootVerticalPosition()  { return Vector3d(0,0,dLz); }
};

// ═══════════════════════════════════════════════════════════════════════════════
// Walk planning types  — humanoid_ws/lipm_motion/motionDefines.h
// ═══════════════════════════════════════════════════════════════════════════════
enum SupportLeg { SUPPORT_LEG_NONE=0, SUPPORT_LEG_LEFT, SUPPORT_LEG_RIGHT, SUPPORT_LEG_BOTH };

struct WalkInstruction {
  VectorXd target;
  SupportLeg targetSupport, targetZMP;
  unsigned steps;
  int step_id;
  WalkInstruction() : targetSupport(SUPPORT_LEG_NONE), targetZMP(SUPPORT_LEG_NONE), steps(0), step_id(-1) {
    target.resize(13); target.setZero();
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// Foot trajectory interpolation  — humanoid_ws/lipm_motion/KWalkMat.h
//   (KMath/KMat dependency replaced with Eigen + standard math)
// ═══════════════════════════════════════════════════════════════════════════════
struct FootInterp {
  // Linear interpolation (scalar, 0-indexed step over N total steps)
  static double linear(double step, double end, double start, double N) {
    double s = step / N;
    return s * end + (1.0 - s) * start;
  }

  // Smooth (min-jerk quintic) XY interpolation — replaces planFeetTrajectoryXY
  static double planFeetXY(double step, double end, double start, double N) {
    double s = std::min(1.0, step / N);
    double s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
    double h = 6*s5 - 15*s4 + 10*s3;   // smooth step (0→1)
    return start + h*(end - start);
  }

  // Cubic arc for foot Z (rise to MaxZ at mid-step, back to 0)
  static double planFeetZ(double step, double MaxZ, double startZ, double endZ, double N) {
    double s = step / N;
    double arc = 4.0 * s * (1.0 - s);  // parabolic, peaks at s=0.5
    return startZ + arc * MaxZ + (endZ - startZ) * s;
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// zmpPlanner  — humanoid_ws/lipm_motion/zmpPlanner.cpp
//   boost::circular_buffer replaced with std::deque
// ═══════════════════════════════════════════════════════════════════════════════
class zmpPlanner {
  WalkInstruction planned_;
  double HX_, HY_, HZ_;
  int DS_Instructions_;
  double MaxStepX_, MinStepX_, MaxStepY_, MinStepY_, MaxStepZ_, dt_;

  VectorXd start_, target_, startL_, startR_, targetL_, targetR_;
  VectorXd footR_, footL_, footR_prev_, footL_prev_;
  VectorXd ZMPref_, v_, omega_;
  Quaterniond qR_, qR_prev_, qL_, qL_prev_;

  static double cropStep(double f, double hi, double lo) {
    return std::max(lo, std::min(f, hi));
  }

  static Vector3d logMap(const Matrix3d &R) {
    double acosV = (R(0,0)+R(1,1)+R(2,2)-1.0)*0.5;
    double theta = std::acos(std::min(1.0,std::max(-1.0,acosV)));
    Vector3d w(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
    constexpr double eps = std::numeric_limits<double>::epsilon();
    return (std::abs(std::sin(theta)) > eps) ? w * (theta/(2.0*std::sin(theta))) : w * 0.5;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::deque<VectorXd> ZMPbuffer, footRbuffer, footLbuffer;
  std::queue<WalkInstruction> stepAnkleQ;
  bool planAvailable;

  zmpPlanner() : planAvailable(false) {
    start_.resize(3); start_.setZero();
    target_.resize(3); target_.setZero();
    startL_.resize(13); startL_.setZero();
    startR_.resize(13); startR_.setZero();
    targetL_.resize(13); targetL_.setZero();
    targetR_.resize(13); targetR_.setZero();
    footR_.resize(13); footR_.setZero();
    footL_.resize(13); footL_.setZero();
    footR_prev_.resize(13); footR_prev_.setZero();
    footL_prev_.resize(13); footL_prev_.setZero();
    ZMPref_.resize(3); ZMPref_.setZero();
    v_.resize(3); v_.setZero();
    omega_.resize(3); omega_.setZero();
  }

  void setParams(double HX, double HY, double HZ, int DS_instr,
                 double MaxStepX, double MinStepX, double MaxStepY, double MinStepY,
                 double MaxStepZ, double dt) {
    HX_=HX; HY_=HY; HZ_=HZ; DS_Instructions_=DS_instr;
    MaxStepX_=MaxStepX; MinStepX_=MinStepX;
    MaxStepY_=MaxStepY; MinStepY_=MinStepY;
    MaxStepZ_=MaxStepZ; dt_=dt;
  }

  void emptyPlan() {
    ZMPbuffer.clear(); footRbuffer.clear(); footLbuffer.clear();
    while (!stepAnkleQ.empty()) stepAnkleQ.pop();
    planAvailable = false;
    start_.setZero(); target_.setZero();
    startL_.setZero(); startR_.setZero();
    targetL_.setZero(); targetR_.setZero();
    footR_.setZero(); footL_.setZero();
  }

  void plan(const Vector3d &actual_COP, const VectorXd &actual_footL, const VectorXd &actual_footR) {
    if (stepAnkleQ.empty()) { return; }
    bool first = true;
    start_  = actual_COP;
    startR_ = actual_footR;
    startL_ = actual_footL;

    while (!stepAnkleQ.empty()) {
      WalkInstruction instr = stepAnkleQ.front(); stepAnkleQ.pop();

      // Initial DS transition
      if (first) {
        Quaterniond ql(startL_(3),startL_(4),startL_(5),startL_(6));
        Quaterniond qr(startR_(3),startR_(4),startR_(5),startR_(6));
        if (instr.targetZMP == SUPPORT_LEG_RIGHT)
          target_ = startR_.head(3) + qr.toRotationMatrix()*Vector3d(HX_,-HY_,HZ_);
        else if (instr.targetZMP == SUPPORT_LEG_LEFT)
          target_ = startL_.head(3) + ql.toRotationMatrix()*Vector3d(HX_, HY_,HZ_);

        footR_ = startR_; footL_ = startL_;
        for (int p = 0; p < 2*DS_Instructions_; p++) {
          ZMPref_(0) = FootInterp::planFeetXY(p, target_(0), start_(0), 2*DS_Instructions_-1.0);
          ZMPref_(1) = FootInterp::planFeetXY(p, target_(1), start_(1), 2*DS_Instructions_-1.0);
          ZMPref_(2) = FootInterp::linear(p, target_(2), start_(2), 2*DS_Instructions_-1.0);
          ZMPbuffer.push_back(ZMPref_); footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        first = false; start_ = target_;
      }

      // Single support: left foot is support, right swings
      if (instr.targetSupport == SUPPORT_LEG_LEFT && instr.targetZMP != SUPPORT_LEG_BOTH) {
        targetR_ = instr.target;
        Quaterniond tqR(targetR_(3),targetR_(4),targetR_(5),targetR_(6));
        Quaterniond sqR(startR_(3),startR_(4),startR_(5),startR_(6));
        // kinematic bounds
        Quaterniond sqL(startL_(3),startL_(4),startL_(5),startL_(6));
        Matrix3d rotL = sqL.toRotationMatrix();
        targetR_(0) = cropStep(targetR_(0), (rotL*Vector3d(MaxStepX_,0,0)+startL_.head(3))(0),
                                            (rotL*Vector3d(MinStepX_,0,0)+startL_.head(3))(0));
        targetR_(1) = cropStep(targetR_(1), (rotL*Vector3d(0,-MinStepY_,0)+startL_.head(3))(1),
                                            (rotL*Vector3d(0,-MaxStepY_,0)+startL_.head(3))(1));

        ZMPref_ = start_; footL_ = startL_;
        for (unsigned p = 0; p < instr.steps; p++) {
          ZMPbuffer.push_back(ZMPref_);
          footR_(0) = FootInterp::planFeetXY(p, targetR_(0), startR_(0), instr.steps-1.0);
          footR_(1) = FootInterp::planFeetXY(p, targetR_(1), startR_(1), instr.steps-1.0);
          footR_(2) = FootInterp::planFeetZ(p, MaxStepZ_, startR_(2), targetR_(2), instr.steps-1.0);
          qR_ = sqR.slerp((double)p/(instr.steps-1.0), tqR);
          footR_(3)=qR_.w(); footR_(4)=qR_.x(); footR_(5)=qR_.y(); footR_(6)=qR_.z();
          if (p==0) v_ = (footR_.head(3)-startR_.head(3))/dt_;
          else      v_ = (footR_.head(3)-footR_prev_.head(3))/dt_;
          footR_.segment(7,3) = v_;
          footR_prev_=footR_; qR_prev_=qR_;
          footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        int ds = DS_Instructions_;
        target_ = targetR_.head(3) + tqR.toRotationMatrix()*Vector3d(HX_, HY_, HZ_);
        footR_ = targetR_; footL_ = startL_;
        for (int p = 0; p < ds; p++) {
          ZMPref_(0) = FootInterp::planFeetXY(p, target_(0), start_(0), ds-1.0);
          ZMPref_(1) = FootInterp::planFeetXY(p, target_(1), start_(1), ds-1.0);
          ZMPref_(2) = FootInterp::linear(p, target_(2), start_(2), ds-1.0);
          ZMPbuffer.push_back(ZMPref_); footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        start_ = target_; startR_ = targetR_;
      }
      // Single support: right foot is support, left swings
      else if (instr.targetSupport == SUPPORT_LEG_RIGHT && instr.targetZMP != SUPPORT_LEG_BOTH) {
        targetL_ = instr.target;
        Quaterniond tqL(targetL_(3),targetL_(4),targetL_(5),targetL_(6));
        Quaterniond sqL(startL_(3),startL_(4),startL_(5),startL_(6));
        Quaterniond sqR(startR_(3),startR_(4),startR_(5),startR_(6));
        Matrix3d rotR = sqR.toRotationMatrix();
        targetL_(0) = cropStep(targetL_(0), (rotR*Vector3d(MaxStepX_,0,0)+startR_.head(3))(0),
                                            (rotR*Vector3d(MinStepX_,0,0)+startR_.head(3))(0));
        targetL_(1) = cropStep(targetL_(1), (rotR*Vector3d(0,MaxStepY_,0)+startR_.head(3))(1),
                                            (rotR*Vector3d(0,MinStepY_,0)+startR_.head(3))(1));

        ZMPref_ = start_; footR_ = startR_;
        for (unsigned p = 0; p < instr.steps; p++) {
          ZMPbuffer.push_back(ZMPref_);
          footL_(0) = FootInterp::planFeetXY(p, targetL_(0), startL_(0), instr.steps-1.0);
          footL_(1) = FootInterp::planFeetXY(p, targetL_(1), startL_(1), instr.steps-1.0);
          footL_(2) = FootInterp::planFeetZ(p, MaxStepZ_, startL_(2), targetL_(2), instr.steps-1.0);
          qL_ = sqL.slerp((double)p/(instr.steps-1.0), tqL);
          footL_(3)=qL_.w(); footL_(4)=qL_.x(); footL_(5)=qL_.y(); footL_(6)=qL_.z();
          if (p==0) v_ = (footL_.head(3)-startL_.head(3))/dt_;
          else      v_ = (footL_.head(3)-footL_prev_.head(3))/dt_;
          footL_.segment(7,3) = v_;
          footL_prev_=footL_; qL_prev_=qL_;
          footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        int ds = DS_Instructions_;
        target_ = targetL_.head(3) + tqL.toRotationMatrix()*Vector3d(HX_, -HY_, HZ_);
        footR_ = startR_; footL_ = targetL_;
        for (int p = 0; p < ds; p++) {
          ZMPref_(0) = FootInterp::planFeetXY(p, target_(0), start_(0), ds-1.0);
          ZMPref_(1) = FootInterp::planFeetXY(p, target_(1), start_(1), ds-1.0);
          ZMPref_(2) = FootInterp::linear(p, target_(2), start_(2), ds-1.0);
          ZMPbuffer.push_back(ZMPref_); footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        start_ = target_; startL_ = targetL_;
      }
      // Double support hold
      else {
        footR_ = startR_; footL_ = startL_;
        Quaterniond sqL(startL_(3),startL_(4),startL_(5),startL_(6));
        Quaterniond sqR(startR_(3),startR_(4),startR_(5),startR_(6));
        target_ = 0.5*(startR_.head(3)+sqR.toRotationMatrix()*Vector3d(HX_, HY_,HZ_)
                      +startL_.head(3)+sqL.toRotationMatrix()*Vector3d(HX_,-HY_,HZ_));
        for (int p = 0; p < DS_Instructions_; p++) {
          ZMPref_(0) = FootInterp::planFeetXY(p, target_(0), start_(0), DS_Instructions_-1.0);
          ZMPref_(1) = FootInterp::planFeetXY(p, target_(1), start_(1), DS_Instructions_-1.0);
          ZMPref_(2) = FootInterp::linear(p, target_(2), start_(2), DS_Instructions_-1.0);
          ZMPbuffer.push_back(ZMPref_); footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        for (unsigned p = 0; p < instr.steps; p++) {
          ZMPbuffer.push_back(target_); footRbuffer.push_back(footR_); footLbuffer.push_back(footL_);
        }
        start_ = target_;
      }
    }
    planAvailable = true;
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// LipmWalkerNode — combines lipm_ros + control into one self-contained node
// ═══════════════════════════════════════════════════════════════════════════════
class LipmWalkerNode : public rclcpp::Node {
public:
  LipmWalkerNode() : Node("lipm_walker") {
    // ── parameters ──────────────────────────────────────────────────────────
    hc_        = declare_parameter("hc",        0.55);
    mass_      = declare_parameter("mass",      25.0);
    g_         = declare_parameter("gravity",   9.80665);
    dt_        = declare_parameter("dt",        0.02);
    freq_      = declare_parameter("control_frequency", 50.0);
    plan_freq_ = declare_parameter("plan_frequency",    50.0);

    step_length_    = declare_parameter("step_length",    0.05);
    foot_separation_= declare_parameter("foot_separation",0.10);
    n_steps_        = declare_parameter("n_steps",        6);
    Tss_            = declare_parameter("Tss",            0.8);
    Tds_            = declare_parameter("Tds",            0.3);
    MaxStepZ_       = declare_parameter("MaxStepZ",       0.02);
    mpc_q_          = declare_parameter("mpc_q",          1.0);
    mpc_r_          = declare_parameter("mpc_r",          1e-6);
    mpc_Np_         = declare_parameter("mpc_Np",         100);

    double Pcom_x = declare_parameter("CoM_admittance_gain_x",  0.8);
    double Pcom_y = declare_parameter("CoM_admittance_gain_y",  0.8);
    double Pzmp_x = declare_parameter("ZMP_proportional_gain_x",2.0);
    double Pzmp_y = declare_parameter("ZMP_proportional_gain_y",2.0);
    double Pdcm_x = declare_parameter("DCM_proportional_gain_x",7.0);
    double Pdcm_y = declare_parameter("DCM_proportional_gain_y",7.0);
    double Idcm_x = declare_parameter("DCM_Integral_gain_x",    0.0);
    double Idcm_y = declare_parameter("DCM_Integral_gain_y",    0.0);
    com_ad_enabled_ = declare_parameter("com_admittance_enabled", true);

    double Ka=declare_parameter("foot_damping_gain",0.01);
    double Ta=declare_parameter("foot_damping_tc",  0.001);
    double Kc=declare_parameter("base_damping_gain",0.01);
    double Tc=declare_parameter("base_damping_tc",  0.001);

    // ── control modules ──────────────────────────────────────────────────────
    lc_ = std::make_unique<LIPMControl>(hc_, dt_, Pcom_x, Pcom_y, Pzmp_x, Pzmp_y, Pdcm_x, Pdcm_y, Idcm_x, Idcm_y);
    zd_ = std::make_unique<ZMPDistributor>(mass_, g_);
    ps_ = std::make_unique<postureStabilizer>(dt_, Kc, Tc, Ka, Ta, 0.0, 1.0);
    dp_ = std::make_unique<LIPMPlanner>();
    zp_ = std::make_unique<zmpPlanner>();

    int SS_instr = (int)std::ceil(Tss_ / dt_);
    int DS_instr = (int)std::ceil(Tds_ / dt_);
    zp_->setParams(0.0, 0.0, 0.0, DS_instr, step_length_, -0.5*step_length_,
                   foot_separation_, foot_separation_, MaxStepZ_, dt_);
    dp_->setParams(hc_, g_, dt_, mpc_q_, mpc_r_, mpc_Np_);
    dp_->init();
    SS_Instructions_ = SS_instr;

    // ── state init ────────────────────────────────────────────────────────────
    CoM_.setZero(); vCoM_.setZero(); aCoM_.setZero(); vCoM_prev_.setZero();
    ZMP_.setZero();
    pL_.setZero(); pR_.setZero();
    qL_.setIdentity(); qR_.setIdentity(); qwb_.setIdentity();
    LLeg_GRF_.setZero(); RLeg_GRF_.setZero();
    LLeg_GRT_.setZero(); RLeg_GRT_.setZero();
    CoM_ref_.setZero(); vCoM_ref_.setZero(); aCoM_ref_.setZero();
    ZMP_ref_.setZero();
    lf_pos_ref_.setZero(); rf_pos_ref_.setZero();
    lf_orient_ref_.setIdentity(); rf_orient_ref_.setIdentity();
    right_support_=false; double_support_=true;
    right_contact_=true;  left_contact_=true;
    first_com_vel_=true; initialized_=false;
    traj_idx_=0; trajectorySize_=0;
    state_ = SETTLE;

    // ── ROS interfaces ────────────────────────────────────────────────────────
    auto qos_transient = rclcpp::QoS(1).transient_local();
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", qos_transient,
        [this](std_msgs::msg::String::SharedPtr m){ urdfCallback(m); });

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 100,
        [this](sensor_msgs::msg::JointState::SharedPtr m){ jointCallback(m); });

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/lower_body_controller/joint_trajectory", 10);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lipm_walker/markers", 10);

    tf_buf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_lis_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, this);

    startup_time_ = now();
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0/freq_),
        [this](){ controlCallback(); });

    RCLCPP_INFO(get_logger(), "lipm_walker initialised - SETTLE for 3 s");
  }

private:
  // ──────────────────────────────────────────────────────────────────────────
  // State machine
  // ──────────────────────────────────────────────────────────────────────────
  enum State { SETTLE, PLAN, WALK, DONE };
  State state_;

  // ──────────────────────────────────────────────────────────────────────────
  // URDF callback — extract link masses and foot y-positions
  // ──────────────────────────────────────────────────────────────────────────
  void urdfCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (!link_masses_.empty()) return;
    urdf::Model model;
    if (!model.initString(msg->data)) return;
    for (auto &[name, lnk] : model.links_) {
      if (lnk->inertial) {
        link_masses_[name] = lnk->inertial->mass;
        link_com_offsets_[name] = Eigen::Vector3d(
            lnk->inertial->origin.position.x,
            lnk->inertial->origin.position.y,
            lnk->inertial->origin.position.z);
        total_mass_ += lnk->inertial->mass;
      }
    }
    RCLCPP_INFO(get_logger(), "URDF loaded: total_mass=%.2f kg", total_mass_);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Joint state callback
  // ──────────────────────────────────────────────────────────────────────────
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    joint_state_ = *msg;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // CoM estimation in pelvis frame (matches backup.cpp approach)
  // ──────────────────────────────────────────────────────────────────────────
  bool estimateCoM() {
    if (link_masses_.empty() || total_mass_ < 0.1) return false;
    double cx=0, cy=0, cz=0;
    for (auto &[name, mass] : link_masses_) {
      try {
        geometry_msgs::msg::PointStamped local_pt, pelvis_pt;
        local_pt.header.frame_id = name;
        local_pt.point.x = link_com_offsets_.at(name)(0);
        local_pt.point.y = link_com_offsets_.at(name)(1);
        local_pt.point.z = link_com_offsets_.at(name)(2);
        pelvis_pt = tf_buf_->transform(local_pt, "pelvis", tf2::durationFromSec(0.05));
        cx += mass * pelvis_pt.point.x;
        cy += mass * pelvis_pt.point.y;
        cz += mass * pelvis_pt.point.z;
      } catch (...) { return false; }
    }
    Eigen::Vector3d com_new(cx/total_mass_, cy/total_mass_, cz/total_mass_);
    if (first_com_vel_) {
      vCoM_.setZero(); aCoM_.setZero(); first_com_vel_=false;
    } else {
      Eigen::Vector3d v_new = (com_new - CoM_) * freq_;
      aCoM_ = (v_new - vCoM_) * freq_;
      vCoM_ = v_new;
    }
    CoM_ = com_new;
    ZMP_ = CoM_ - aCoM_ / (lc_->omega * lc_->omega);
    ZMP_(2) = h_foot_pelvis_;  // foot z level in pelvis frame
    return true;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Foot pose in pelvis frame (matches backup.cpp approach)
  // ──────────────────────────────────────────────────────────────────────────
  bool lookupFeet() {
    try {
      auto tfR = tf_buf_->lookupTransform("pelvis","foot_r",tf2::TimePointZero);
      auto tfL = tf_buf_->lookupTransform("pelvis","foot_l",tf2::TimePointZero);
      pR_ = Vector3d(tfR.transform.translation.x,
                     tfR.transform.translation.y,
                     tfR.transform.translation.z);
      pL_ = Vector3d(tfL.transform.translation.x,
                     tfL.transform.translation.y,
                     tfL.transform.translation.z);
      qR_ = Quaterniond(tfR.transform.rotation.w,tfR.transform.rotation.x,
                        tfR.transform.rotation.y,tfR.transform.rotation.z);
      qL_ = Quaterniond(tfL.transform.rotation.w,tfL.transform.rotation.x,
                        tfL.transform.rotation.y,tfL.transform.rotation.z);
      return true;
    } catch (...) { return false; }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Build footstep sequence and send to zmpPlanner + LIPMPlanner (lipm_ros pattern)
  // ──────────────────────────────────────────────────────────────────────────
  void plan() {
    if (!lookupFeet() || !estimateCoM()) {
      RCLCPP_WARN(get_logger(), "Plan: TF/CoM not ready yet");
      return;
    }

    // ── Capture foot z level and measure actual CoM height above ground ──────
    // All values are in pelvis frame.
    // Walk frame convention (from backup.cpp):
    //   walk_x (forward) = -pelvis_y
    //   walk_y (lateral) = +pelvis_x
    h_foot_pelvis_ = (pR_(2) + pL_(2)) * 0.5;
    double hc_measured = CoM_(2) - h_foot_pelvis_;
    // Reinit planner with the real CoM height so trajectory z lines up
    dp_->setParams(hc_measured, g_, dt_, mpc_q_, mpc_r_, mpc_Np_);
    dp_->init();
    lc_->omega = std::sqrt(g_ / hc_measured);

    // Walk frame origin in pelvis frame (midpoint of both feet)
    double px_mid = (pR_(0) + pL_(0)) * 0.5;
    double py_mid = (pR_(1) + pL_(1)) * 0.5;

    // Right foot lateral offset in walk frame: walk_y_r = pelvis_x_r - px_mid
    double wy_r = pR_(0) - px_mid;   // walk-y of right foot  (negative)
    double wy_l = pL_(0) - px_mid;   // walk-y of left foot   (positive)

    RCLCPP_INFO(get_logger(),
      "Plan: feet pelvis R=(%.3f %.3f %.3f) L=(%.3f %.3f %.3f)\n"
      "  h_foot=%.3f  hc_measured=%.3f  omega=%.3f\n"
      "  walk origin pelvis (px_mid=%.3f py_mid=%.3f)  wy_r=%.3f wy_l=%.3f",
      pR_(0),pR_(1),pR_(2), pL_(0),pL_(1),pL_(2),
      h_foot_pelvis_, hc_measured, lc_->omega, px_mid, py_mid, wy_r, wy_l);

    // Build step sequence in pelvis frame using walk-frame rotation
    zp_->emptyPlan(); dp_->emptyPlan();

    VectorXd lfoot(13), rfoot(13);
    lfoot.setZero(); rfoot.setZero();
    lfoot.head(3) = pL_; lfoot(3)=qL_.w(); lfoot(4)=qL_.x(); lfoot(5)=qL_.y(); lfoot(6)=qL_.z();
    rfoot.head(3) = pR_; rfoot(3)=qR_.w(); rfoot(4)=qR_.x(); rfoot(5)=qR_.y(); rfoot(6)=qR_.z();

    for (int k = 0; k < n_steps_; k++) {
      WalkInstruction instr;
      instr.steps = SS_Instructions_;
      instr.target.setZero();
      bool right_swing = (k % 2 == 0);

      // Walk frame step: forward wx = (k+1)*step_length, lateral wy = ±offset
      double wx = (k + 1) * step_length_;
      double wy = right_swing ? wy_r : wy_l;

      // Convert walk frame → pelvis frame:  px = wy + px_mid,  py = py_mid - wx
      double tpx = px_mid + wy;
      double tpy = py_mid - wx;

      instr.target(0) = tpx; instr.target(1) = tpy; instr.target(2) = h_foot_pelvis_;
      instr.target(3) = 1.0; // identity quaternion w
      if (right_swing) {
        instr.targetSupport = SUPPORT_LEG_LEFT;
        instr.targetZMP     = SUPPORT_LEG_LEFT;
      } else {
        instr.targetSupport = SUPPORT_LEG_RIGHT;
        instr.targetZMP     = SUPPORT_LEG_RIGHT;
      }
      instr.step_id = k;
      zp_->stepAnkleQ.push(instr);
      if (k == n_steps_-1) {
        WalkInstruction ds = instr;
        ds.targetZMP = SUPPORT_LEG_BOTH;
        zp_->stepAnkleQ.push(ds);
      }
    }

    Vector3d cop(ZMP_(0), ZMP_(1), h_foot_pelvis_);
    zp_->plan(cop, lfoot, rfoot);

    if (zp_->ZMPbuffer.empty()) { RCLCPP_ERROR(get_logger(),"ZMP plan empty!"); return; }

    Vector2d CoM2(CoM_(0), CoM_(1));
    Vector2d vCoM2(vCoM_(0), vCoM_(1));
    Vector2d ZMP2(ZMP_(0), ZMP_(1));
    dp_->setState(CoM2, vCoM2, ZMP2);
    dp_->plan(zp_->ZMPbuffer);

    if (dp_->CoMBuffer.empty()) { RCLCPP_ERROR(get_logger(),"CoM plan empty!"); return; }

    trajectorySize_ = (int)dp_->CoMBuffer.size();
    traj_idx_ = 0;

    // Set initial references
    CoM_ref_  = Vector3d(dp_->CoMBuffer[0](0), dp_->CoMBuffer[0](1), dp_->CoMBuffer[0](2));
    vCoM_ref_ = Vector3d(dp_->CoMBuffer[0](3), dp_->CoMBuffer[0](4), dp_->CoMBuffer[0](5));
    aCoM_ref_ = Vector3d(dp_->CoMBuffer[0](6), dp_->CoMBuffer[0](7), dp_->CoMBuffer[0](8));
    ZMP_ref_  = Vector3d(dp_->VRPBuffer[0](0), dp_->VRPBuffer[0](1), dp_->VRPBuffer[0](2));
    lf_pos_ref_ = pL_; rf_pos_ref_ = pR_;
    lf_orient_ref_ = qL_; rf_orient_ref_ = qR_;

    RCLCPP_INFO(get_logger(), "Plan ready: %d trajectory points, %zu ZMP points",
                trajectorySize_, dp_->CoMBuffer.size());
    state_ = WALK;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Leg IK — analytical 6-DOF (hip_pitch/roll/yaw + knee + ankle_pitch/roll)
  //   Robot geometry: hip at (hx, ±hy, hz) from pelvis; thigh L1; shank L2.
  //   Joint angle mapping matches the lower_body_controller joint order.
  // ──────────────────────────────────────────────────────────────────────────
  struct LegAngles {
    double hip_roll, hip_pitch, hip_yaw, knee_pitch, ankle_pitch, ankle_roll;
  };

  LegAngles legIK(double fx, double fy, double fz, bool is_right) {
    const double hx = -0.149;
    const double hy = is_right ? -0.122 : 0.122;
    const double hz = -0.103;
    const double afo_x = 0.017;
    const double afo_y = is_right ? -0.023 : 0.023;
    const double afo_z = -0.030;
    const double L1 = 0.2517, L2 = 0.3472;

    double ax = fx - afo_x, ay = fy - afo_y, az = fz - afo_z;
    double dx = ax - hx, dy = ay - hy, dz = az - hz;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    double cos_knee = (dist*dist - L1*L1 - L2*L2) / (2.0*L1*L2);
    cos_knee = std::max(-1.0, std::min(1.0, cos_knee));
    double knee_pitch = std::acos(cos_knee);
    double hip_roll   = std::atan2(dy, -dz);
    double hip_pitch  = std::atan2(dx, std::sqrt(dy*dy + dz*dz));
    double ankle_pitch = -(hip_pitch + knee_pitch);
    double ankle_roll  = -hip_roll;
    double ankle_roll_cmd = is_right ? ankle_roll : -ankle_roll;

    // Joint offsets: map geometric angles to actuator angles
    constexpr double off_hip_roll    =  0.615;
    constexpr double off_hip_pitch   =  0.063;
    constexpr double off_knee_pitch  =  0.389;
    constexpr double off_ankle_pitch = -0.753;

    return { hip_roll  - off_hip_roll,
            -(hip_pitch  - off_hip_pitch),
             0.0,
            -(knee_pitch - off_knee_pitch),
             ankle_pitch  - off_ankle_pitch,
             ankle_roll_cmd };
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Standpose trajectory message
  // ──────────────────────────────────────────────────────────────────────────
  trajectory_msgs::msg::JointTrajectory makeStandpose() {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp = rclcpp::Time(0);
    msg.joint_names = { "hip_pitch_r","hip_roll_r","thigh_yaw_r",
                        "knee_pitch_r","ankle_pitch_r","ankle_roll_r",
                        "hip_pitch_l","hip_roll_l","thigh_yaw_l",
                        "knee_pitch_l","ankle_pitch_l","ankle_roll_l" };
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = { -0.615, 0.0, 0.0, -0.615, -0.351, 0.0,
                     -0.615, 0.0, 0.0, -0.615, -0.351, 0.0 };
    pt.time_from_start = rclcpp::Duration::from_seconds(1.2);
    msg.points.push_back(pt);
    return msg;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Publish visualization markers in pelvis frame (matches backup.cpp style)
  //   0  : CoM desired — blue sphere
  //   1  : ZMP desired — red sphere
  //   2  : DCM        — orange sphere
  //   3  : CoM trajectory precomputed — green line strip
  //   10+: footstep cubes (orange=right, cyan=left)
  // ──────────────────────────────────────────────────────────────────────────
  void publishMarkers(const Vector3d &com, const Vector3d &zmp, const Vector3d &dcm) {
    visualization_msgs::msg::MarkerArray ma;
    auto t = now();

    auto sphere = [&](int id, Vector3d p, float r, float g, float b, float sz=0.05f) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pelvis";
      m.header.stamp = t;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x=p(0); m.pose.position.y=p(1); m.pose.position.z=p(2);
      m.pose.orientation.w=1.0;
      m.scale.x=m.scale.y=m.scale.z=sz;
      m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=0.9f;
      return m;
    };

    ma.markers.push_back(sphere(0, com, 0.0f,0.4f,1.0f));        // blue = CoM
    ma.markers.push_back(sphere(1, zmp, 1.0f,0.1f,0.0f, 0.04f)); // red  = ZMP
    ma.markers.push_back(sphere(2, dcm, 1.0f,0.5f,0.0f, 0.04f)); // orange = DCM

    // CoM trajectory preview — green line strip (from CoMBuffer)
    {
      visualization_msgs::msg::Marker ls;
      ls.header.frame_id = "pelvis";
      ls.header.stamp = t;
      ls.id = 3;
      ls.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ls.action = visualization_msgs::msg::Marker::ADD;
      ls.pose.orientation.w = 1.0;
      ls.scale.x = 0.01;
      ls.color.r=0.0f; ls.color.g=1.0f; ls.color.b=0.0f; ls.color.a=0.8f;
      for (const auto &cb : dp_->CoMBuffer) {
        geometry_msgs::msg::Point p;
        p.x=cb(0); p.y=cb(1); p.z=cb(2);
        ls.points.push_back(p);
      }
      ma.markers.push_back(ls);
    }

    // Footstep cubes — right=orange, left=cyan
    int id = 10;
    bool is_right = true;
    for (const auto &fb : zp_->footRbuffer) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pelvis";
      m.header.stamp = t;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x=fb(0); m.pose.position.y=fb(1); m.pose.position.z=fb(2);
      m.pose.orientation.w=1.0;
      m.scale.x=0.08; m.scale.y=0.04; m.scale.z=0.01;
      m.color.r=1.0f; m.color.g=0.5f; m.color.b=0.0f; m.color.a=0.7f; // orange
      ma.markers.push_back(m);
    }
    for (const auto &fb : zp_->footLbuffer) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "pelvis";
      m.header.stamp = t;
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x=fb(0); m.pose.position.y=fb(1); m.pose.position.z=fb(2);
      m.pose.orientation.w=1.0;
      m.scale.x=0.08; m.scale.y=0.04; m.scale.z=0.01;
      m.color.r=0.0f; m.color.g=0.9f; m.color.b=0.9f; m.color.a=0.7f; // cyan
      ma.markers.push_back(m);
    }

    marker_pub_->publish(ma);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Main control callback  (lipm_ros + control.cpp combined)
  // ──────────────────────────────────────────────────────────────────────────
  void controlCallback() {
    double elapsed = (now() - startup_time_).seconds();

    // ── SETTLE ──────────────────────────────────────────────────────────────
    if (state_ == SETTLE) {
      traj_pub_->publish(makeStandpose());
      if (elapsed > 3.0 && !link_masses_.empty()) {
        RCLCPP_INFO(get_logger(), "Settling complete — planning trajectory");
        state_ = PLAN;
      }
      return;
    }

    // ── PLAN ────────────────────────────────────────────────────────────────
    if (state_ == PLAN) {
      plan();
      return;
    }

    // ── DONE ────────────────────────────────────────────────────────────────
    if (state_ == DONE) {
      traj_pub_->publish(makeStandpose());
      return;
    }

    // ── WALK ────────────────────────────────────────────────────────────────
    if (!estimateCoM() || !lookupFeet()) return;

    // Advance trajectory index (plan_freq ratio)
    static int jj = 0;
    if (traj_idx_ < (size_t)trajectorySize_) {
      int ratio = std::max(1, (int)std::round(freq_ / plan_freq_));
      if (jj >= ratio-1) { traj_idx_++; jj=0; }
      else { jj++; }
    }

    if (traj_idx_ < (size_t)trajectorySize_) {
      const auto &cb = dp_->CoMBuffer[traj_idx_];
      const auto &vb = dp_->VRPBuffer[traj_idx_];
      const auto &lb = zp_->footLbuffer[std::min(traj_idx_, zp_->footLbuffer.size()-1)];
      const auto &rb = zp_->footRbuffer[std::min(traj_idx_, zp_->footRbuffer.size()-1)];

      CoM_ref_  = Vector3d(cb(0), cb(1), cb(2));
      vCoM_ref_ = Vector3d(cb(3), cb(4), cb(5));
      aCoM_ref_ = Vector3d(cb(6), cb(7), cb(8));
      ZMP_ref_  = Vector3d(vb(0), vb(1), vb(2));

      lf_pos_ref_ = Vector3d(lb(0), lb(1), lb(2));
      rf_pos_ref_ = Vector3d(rb(0), rb(1), rb(2));
      lf_orient_ref_ = Quaterniond(lb(3),lb(4),lb(5),lb(6));
      rf_orient_ref_ = Quaterniond(rb(3),rb(4),rb(5),rb(6));

      // Determine support phase
      double alpha = (double)traj_idx_ / trajectorySize_;
      double_support_ = (alpha < 0.15 || alpha > 0.85);
      right_support_  = !double_support_ && (traj_idx_ % (2*SS_Instructions_) < SS_Instructions_);
      right_contact_  = true; left_contact_ = true;
    } else {
      RCLCPP_INFO_ONCE(get_logger(), "Walk complete");
      state_ = DONE;
      return;
    }

    // ── DCM Admittance Control (LIPMControl from humanoid_ws/control.cpp) ──
    Vector3d tempC = CoM_ref_, tempV = vCoM_ref_;
    if (com_ad_enabled_) {
      lc_->Control(ZMP_, CoM_, vCoM_, aCoM_, CoM_ref_, vCoM_ref_, aCoM_ref_);
      tempC = lc_->getDesiredCoMPosition();
      tempV = lc_->getDesiredCoMVelocity();
    }

    // ── ZMP Distribution (ZMPDistributor from humanoid_ws) ──────────────────
    zd_->computeDistribution(ZMP_ref_, ZMP_, pL_, pR_, right_support_, double_support_);

    // ── Posture Stabilizer (postureStabilizer from humanoid_ws) ─────────────
    ps_->footTorqueStabilizer(zd_->tauld, zd_->taurd, LLeg_GRT_, RLeg_GRT_,
                               right_contact_, left_contact_);
    Quaterniond qrr = ps_->getRightFootOrientation();
    Quaterniond qll = ps_->getLeftFootOrientation();
    Quaterniond lf_corr = lf_orient_ref_ * qll;
    Quaterniond rf_corr = rf_orient_ref_ * qrr;

    // ── Leg IK ───────────────────────────────────────────────────────────────
    // Foot refs are already in pelvis frame — pass directly to IK
    double r_fx = rf_pos_ref_(0);
    double r_fy = rf_pos_ref_(1);
    double r_fz = rf_pos_ref_(2) + ps_->getRightFootVerticalPosition()(2);

    double l_fx = lf_pos_ref_(0);
    double l_fy = lf_pos_ref_(1);
    double l_fz = lf_pos_ref_(2) + ps_->getLeftFootVerticalPosition()(2);

    auto ar = legIK(r_fx, r_fy, r_fz, true);
    auto al = legIK(l_fx, l_fy, l_fz, false);

    // ── Diagnostics ─────────────────────────────────────────────────────────
    RCLCPP_INFO(get_logger(),
      "[%zu/%d] step=%s ds=%d\n"
      "  CoM  pelvis (%.3f %.3f %.3f)  ref (%.3f %.3f %.3f)\n"
      "  ZMP  meas  (%.3f %.3f)  ref  (%.3f %.3f)\n"
      "  Foot R pelvis (%.3f %.3f %.3f)\n"
      "  Foot L pelvis (%.3f %.3f %.3f)\n"
      "  IK R  hp=%.3f hr=%.3f hy=%.3f  kp=%.3f  ap=%.3f ar=%.3f\n"
      "  IK L  hp=%.3f hr=%.3f hy=%.3f  kp=%.3f  ap=%.3f ar=%.3f",
      traj_idx_, trajectorySize_,
      right_support_ ? "R-supp" : (double_support_ ? "DS" : "L-supp"),
      (int)double_support_,
      CoM_(0),CoM_(1),CoM_(2), tempC(0),tempC(1),tempC(2),
      ZMP_(0),ZMP_(1), ZMP_ref_(0),ZMP_ref_(1),
      r_fx,r_fy,r_fz,
      l_fx,l_fy,l_fz,
      ar.hip_pitch, ar.hip_roll, ar.hip_yaw, ar.knee_pitch, ar.ankle_pitch, ar.ankle_roll,
      al.hip_pitch, al.hip_roll, al.hip_yaw, al.knee_pitch, al.ankle_pitch, al.ankle_roll);

    // ── Publish joint trajectory ─────────────────────────────────────────────
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0);
    traj.joint_names = { "hip_pitch_r","hip_roll_r","thigh_yaw_r",
                         "knee_pitch_r","ankle_pitch_r","ankle_roll_r",
                         "hip_pitch_l","hip_roll_l","thigh_yaw_l",
                         "knee_pitch_l","ankle_pitch_l","ankle_roll_l" };
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = { ar.hip_roll, ar.hip_pitch, ar.hip_yaw,
                     ar.knee_pitch, ar.ankle_pitch, ar.ankle_roll,
                     al.hip_roll, al.hip_pitch, al.hip_yaw,
                     al.knee_pitch, al.ankle_pitch, al.ankle_roll };
    pt.time_from_start = rclcpp::Duration::from_seconds(1.0/freq_);
    traj.points.push_back(pt);
    traj_pub_->publish(traj);

    // ── Markers ──────────────────────────────────────────────────────────────
    Vector3d dcm = lc_->computeDCM(tempC, tempV);
    publishMarkers(tempC, ZMP_, dcm);
  }

  // ── Members ─────────────────────────────────────────────────────────────────
  // Control modules
  std::unique_ptr<LIPMControl>      lc_;
  std::unique_ptr<ZMPDistributor>   zd_;
  std::unique_ptr<postureStabilizer> ps_;
  std::unique_ptr<LIPMPlanner>      dp_;
  std::unique_ptr<zmpPlanner>       zp_;

  // Parameters
  double hc_, mass_, g_, dt_, freq_, plan_freq_;
  double step_length_, foot_separation_, Tss_, Tds_, MaxStepZ_;
  double mpc_q_, mpc_r_;
  int mpc_Np_, n_steps_, SS_Instructions_;
  bool com_ad_enabled_;

  // Sensor state
  Vector3d CoM_, vCoM_, vCoM_prev_, aCoM_;
  Vector3d ZMP_, pL_, pR_;
  double h_foot_pelvis_ = 0.0;  // foot z level in pelvis frame (captured at plan time)
  Quaterniond qL_, qR_, qwb_;
  Vector3d LLeg_GRF_, RLeg_GRF_, LLeg_GRT_, RLeg_GRT_;
  bool right_support_, double_support_, right_contact_, left_contact_;
  bool first_com_vel_, initialized_;

  // Desired references
  Vector3d CoM_ref_, vCoM_ref_, aCoM_ref_, ZMP_ref_;
  Vector3d lf_pos_ref_, rf_pos_ref_;
  Quaterniond lf_orient_ref_, rf_orient_ref_;

  // Trajectory tracking
  size_t traj_idx_;
  int trajectorySize_;

  // URDF mass map
  std::map<std::string, double> link_masses_;
  std::map<std::string, Eigen::Vector3d> link_com_offsets_;
  double total_mass_ = 0.0;

  // ROS
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_lis_;
  sensor_msgs::msg::JointState joint_state_;
  rclcpp::Time startup_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LipmWalkerNode>());
  rclcpp::shutdown();
  return 0;
}
