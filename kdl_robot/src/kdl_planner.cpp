#include "kdl_ros_control/kdl_planner.h"
#include "math.h"
#define pi 3.14

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd, int _type)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    type_ = _type;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, double _trajRadius, int _type)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
    type_ = _type;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* time = current time
     trajDuration_ = final time
     accDuration_ = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_ = trajectory final point */

  trajectory_point traj;
  
  double s, ds, dds;
  if (type_ == 1 || type_ == 3)
    cubic_polinomial(s, ds, dds, time);
  else
    trapezoidal_vel(s, ds, dds, time, accDuration_);
  
  if (type_ == 1 || type_ == 2){  
    traj.pos = trajInit_ + s*(trajEnd_ - trajInit_);
    traj.vel = ds*(trajEnd_ - trajInit_);
    traj.acc = dds*(trajEnd_ - trajInit_);
    return traj;
  }  
  else{
    traj.pos(0) = trajInit_(0);
    traj.pos(1) = trajInit_(1) - trajRadius_*std::cos(2*pi*s);
    traj.pos(2) = trajInit_(2) - trajRadius_*std::sin(2*pi*s);
  
    traj.vel(0) = 0;
    traj.vel(1) = trajRadius_*2*pi*std::sin(2*pi*s)*ds;
    traj.vel(2) = -trajRadius_*2*pi*std::cos(2*pi*s)*ds;
  
    traj.acc(0) = 0;
    traj.acc(1) = trajRadius_*std::pow(2*pi,2)*std::cos(2*pi*s)*std::pow(ds,2) + trajRadius_*2*pi*std::sin(2*pi*s)*dds;
    traj.acc(2) = trajRadius_*std::pow(2*pi,2)*std::sin(2*pi*s)*std::pow(ds,2) - trajRadius_*2*pi*std::cos(2*pi*s)*dds;
    
    return traj;
 }
}

void KDLPlanner::trapezoidal_vel(double& s, double& ds, double& dds, double t, double tc)
{
  /* trapezoidal velocity profile with tc acceleration time period and trajDuration_ total duration.
     t = current time
     trajDuration_ = final time
     tc = acceleration time
     trajInit = trajectory initial point = 0
     trajEnd = trajectory final point = 1 */

  
  //Curvilinear abscissa between 0 and 1
  
  double ddot_s_c = -1.0/(std::pow(tc,2) - trajDuration_*tc);

  if(t <= tc)
  {
    s = 0.5*ddot_s_c*std::pow(t,2);
    ds = ddot_s_c*t;
    dds = ddot_s_c;
  }
  else if(t <= trajDuration_ - tc)
  {
    s = ddot_s_c*tc*(t - tc/2);
    ds = ddot_s_c*tc;
    dds = 0;
    
  }
  else
  {
    s = 1 - 0.5*ddot_s_c*std::pow(trajDuration_ - t,2);
    ds = ddot_s_c*(trajDuration_ - t);
    dds = -ddot_s_c;
  }
}


void KDLPlanner::cubic_polinomial(double& s, double& ds, double& dds, double t)
{
  /* cubic polinomial profile with a0,a1,a2 and a3 as coefficients to be determined
     t = current time
     trajDuration_ = final time
     trajInit_ = trajectory initial point
     trajEnd_ = trajectory final point */
  
  //si = 0; dsi = 0; sf = 1; dsf = 0
  
  double a0, a1, a2, a3, sf, dsf;
  
  sf = 1;
  dsf = 0;
  
  a0 = 0;
  a1 = 0;
  a2 = -3*a0/std::pow(trajDuration_,2) - 2*a1/trajDuration_ + 3*sf/std::pow(trajDuration_,2) - dsf/trajDuration_;
  a3 = 2*a0/std::pow(trajDuration_,3) + a1/std::pow(trajDuration_,2) - 2*sf/std::pow(trajDuration_,3) + dsf/std::pow(trajDuration_,2);
  
  s = a3*std::pow(t,3) + a2*std::pow(t,2) + a1*t + a0;
  ds = 3*a3*std::pow(t,2) + 2*a2*t + a1;
  dds = 6*a3*t + 2*a2;
}
