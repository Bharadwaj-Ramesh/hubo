#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/PID.hh"

namespace gazebo
{
  class PIDJoints : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model_ = _model;
      // initialize a PID class
// left hand shoulder(p,y,r)
      this->target_position_ = 0.0;
      this->lshoulder_target_position_ = 0.0;
      this->pid_lsp.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_lsp.SetCmd(this->target_position_);
      this->lsp_joint_ = this->model_->GetJoint("LSP");

      this->pid_lsr.Init(25, 0, 0.1, 0, 0, 25, -25);
      this->pid_lsr.SetCmd(this->rshoulder_target_position_);
      this->lsr_joint_ = this->model_->GetJoint("LSR");

      this->pid_lsy.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_lsy.SetCmd(this->target_position_);
      this->lsy_joint_ = this->model_->GetJoint("LSY");

// right hand shoulder(p,y,r)
      this->rshoulder_target_position_ = 0.0;
      this->pid_rsp.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_rsp.SetCmd(this->target_position_);
      this->rsp_joint_ = this->model_->GetJoint("RSP");

      this->pid_rsr.Init(25, 0, 0.1, 0, 0, 25, -25);
      this->pid_rsr.SetCmd(this->lshoulder_target_position_);
      this->rsr_joint_ = this->model_->GetJoint("RSR");

      this->pid_rsy.Init(100, 0, 0, 0, 0, 25, -25);
      this->pid_rsy.SetCmd(this->target_position_);
      this->rsy_joint_ = this->model_->GetJoint("RSY");
// left elbow 

      this->pid_lep.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_lep.SetCmd(this->target_position_);
      this->lep_joint_ = this->model_->GetJoint("LEP");
// right elbow

      this->pid_rep.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_rep.SetCmd(this->target_position_);
      this->rep_joint_ = this->model_->GetJoint("REP");

// left wrist

      this->pid_lwy.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_lwy.SetCmd(this->target_position_);
      this->lwy_joint_ = this->model_->GetJoint("LWY");

      this->pid_lwp.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_lwp.SetCmd(this->target_position_);
      this->lwp_joint_ = this->model_->GetJoint("LWP");
// right wrist

      this->pid_rwy.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_rwy.SetCmd(this->target_position_);
      this->rwy_joint_ = this->model_->GetJoint("RWY");

      this->pid_rwp.Init(50, 0, 0, 0, 0, 25, -25);
      this->pid_rwp.SetCmd(this->target_position_);
      this->rwp_joint_ = this->model_->GetJoint("RWP");

 

// hip joint

      this->pid_hpy.Init(250, 0, 0, 0, 0, 25, -25);
      this->pid_hpy.SetCmd(this->hip_target_position_);
      this->hpy_joint_ = this->model_->GetJoint("HPY");
// left leg joints

      this->pid_lhy.Init(250, 0, 0.15, 0, 0, 25, -25);
      this->pid_lhy.SetCmd(this->target_position_);
      this->lhy_joint_ = this->model_->GetJoint("LHY");

      this->pid_lhr.Init(575, 20, 0, 25, -25, 25, -25);
      this->pid_lhr.SetCmd(this->target_position_);
      this->lhr_joint_ = this->model_->GetJoint("LHR");

      this->pid_lhp.Init(500, 0, 0.15, 0, 0, 25, -25);
      this->pid_lhp.SetCmd(this->target_position_);
      this->lhp_joint_ = this->model_->GetJoint("LHP");

      this->pid_lkp.Init(200, 0, 0.15, 0, 0, 25, -25);
      this->pid_lkp.SetCmd(this->target_position_);
      this->lkp_joint_ = this->model_->GetJoint("LKP");

      this->pid_lap.Init(500, 0, 0.15, 0, 0, 25, -25);
      this->pid_lap.SetCmd(this->target_position_);
      this->lap_joint_ = this->model_->GetJoint("LAP");

      this->pid_lar.Init(390, 0, 0.1, 0, 0, 25, -25);
      this->pid_lar.SetCmd(this->target_position_);
      this->lar_joint_ = this->model_->GetJoint("LAR");

//right leg joints    
 
      this->pid_rhy.Init(250, 0, 0.1, 0, 0, 25, -25);
      this->pid_rhy.SetCmd(this->target_position_);
      this->rhy_joint_ = this->model_->GetJoint("RHY");
      this->pid_rhr.Init(575, 20, 0, 25, -25, 25, -25);
      this->pid_rhr.SetCmd(this->target_position_);
      this->rhr_joint_ = this->model_->GetJoint("RHR");
      this->pid_rhp.Init(500, 0, 0.15, 0, 0, 25, -25);
      this->pid_rhp.SetCmd(this->target_position_);
      this->rhp_joint_ = this->model_->GetJoint("RHP");
      this->pid_rkp.Init(200, 0, 0.15, 0, 0, 25, -25);
      this->pid_rkp.SetCmd(this->target_position_);
      this->rkp_joint_ = this->model_->GetJoint("RKP");
      this->pid_rap.Init(500, 0, 0.15, 0, 0, 25, -25);
      this->pid_rap.SetCmd(this->target_position_);
      this->rap_joint_ = this->model_->GetJoint("RAP");
      this->pid_rar.Init(390, 0, 0.1, 0, 0, 25, -25);
      this->pid_rar.SetCmd(this->target_position_);
      this->rar_joint_ = this->model_->GetJoint("RAR");
// neck
      this->pid_HNP.Init(390, 0, 0.1, 0, 0, 25, -25);
      this->pid_HNP.SetCmd(this->target_position_);
      this->HNP_joint_ = this->model_->GetJoint("HNP");

      this->last_update_time_ = this->model_->GetWorld()->GetSimTime();
      this->update_connection_ = event::Events::ConnectWorldUpdateStart(
        boost::bind(&PIDJoints::UpdatePID, this));
    }
    void UpdatePID()
    {
//     this->last_update_time_ = current_time;
//      gzdbg << "error [" << error
//            << "] cmd [" << this->pid.GetCmd() << "]\n";
      loop_No = loop_No + 1; 
      common::Time current_time = this->model_->GetWorld()->GetSimTime();
      double error = this->lhr_joint_->GetAngle(0).Radian()
                   - target_position_;
      double dt    = current_time.Double()
                   - this->last_update_time_.Double();

      this->pid_lhr.Update(error, dt);
      this->lhr_joint_->SetForce(0, this->pid_lhr.GetCmd());
//     gzdbg << "error [" << error
 //           << "] cmd [" << this->pid_lhr.GetCmd() << "]" << "[ loop No [" << loop_No << "]\n";

      error = this->lhp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lhp.Update(error, dt);
      this->lhp_joint_->SetForce(0, this->pid_lhp.GetCmd());

      error = this->lkp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lkp.Update(error, dt);
      this->lkp_joint_->SetForce(0, this->pid_lkp.GetCmd());
      error = this->lap_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lap.Update(error, dt);
      this->lap_joint_->SetForce(0, this->pid_lap.GetCmd());
      error = this->lar_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lar.Update(error, dt);
      this->lar_joint_->SetForce(0, this->pid_lar.GetCmd());
      error = this->lhy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lhy.Update(error, dt);
      this->lhy_joint_->SetForce(0, this->pid_lhy.GetCmd());
      
      error = this->rhr_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rhr.Update(error, dt);
      this->rhr_joint_->SetForce(0, this->pid_rhr.GetCmd());
      
      error = this->rhp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rhp.Update(error, dt);
      this->rhp_joint_->SetForce(0, this->pid_rhp.GetCmd());

      error = this->rkp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rkp.Update(error, dt);
      this->rkp_joint_->SetForce(0, this->pid_rkp.GetCmd());

      error = this->rap_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rap.Update(error, dt);
      this->rap_joint_->SetForce(0, this->pid_rap.GetCmd());

      error = this->rar_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rar.Update(error, dt);
      this->rar_joint_->SetForce(0, this->pid_rar.GetCmd());
      error = this->rhy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rhy.Update(error, dt);
      this->rhy_joint_->SetForce(0, this->pid_rhy.GetCmd());
//hyp
      error = this->hpy_joint_->GetAngle(0).Radian()
                   - hip_target_position_;
      this->pid_hpy.Update(error, dt);
      this->hpy_joint_->SetForce(0, this->pid_hpy.GetCmd());

// shoulder updates
      error = this->lsp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lsp.Update(error, dt);
      this->lsp_joint_->SetForce(0, this->pid_lsp.GetCmd());
      error = this->lsr_joint_->GetAngle(0).Radian()
                   - lshoulder_target_position_;
      this->pid_lsr.Update(error, dt);
      this->lsr_joint_->SetForce(0, this->pid_lsr.GetCmd());
      error = this->lsy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lsy.Update(error, dt);
      this->lsy_joint_->SetForce(0, this->pid_lsy.GetCmd());
      error = this->rsp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rsp.Update(error, dt);
      this->rsp_joint_->SetForce(0, this->pid_rsp.GetCmd());
      error = this->rsr_joint_->GetAngle(0).Radian()
                   - rshoulder_target_position_;
      this->pid_rsr.Update(error, dt);
      this->rsr_joint_->SetForce(0, this->pid_rsr.GetCmd());
      error = this->rsy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rsy.Update(error, dt);
      this->rsy_joint_->SetForce(0, this->pid_rsy.GetCmd());
// elbows
      error = this->lep_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lep.Update(error, dt);
      this->lep_joint_->SetForce(0, this->pid_lep.GetCmd());
      error = this->rep_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rep.Update(error, dt);
      this->rep_joint_->SetForce(0, this->pid_rep.GetCmd());
      
// wrist
      error = this->lwy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lwy.Update(error, dt);
      this->lwy_joint_->SetForce(0, this->pid_lwy.GetCmd());
      error = this->lwp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_lwp.Update(error, dt);
      this->lwp_joint_->SetForce(0, this->pid_lwp.GetCmd());
      error = this->rwy_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rwy.Update(error, dt);
      this->rwy_joint_->SetForce(0, this->pid_rwy.GetCmd());
      error = this->rwp_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_rwp.Update(error, dt);
      this->rwp_joint_->SetForce(0, this->pid_rwp.GetCmd());

// neck

      error = this->HNP_joint_->GetAngle(0).Radian()
                   - target_position_;
      this->pid_HNP.Update(error, dt);
      this->HNP_joint_->SetForce(0, this->pid_HNP.GetCmd());
//      gzdbg << "error [" << error
 //           << "] cmd [" << this->pid_HNP.GetCmd() << "]\n";
      this->last_update_time_ = current_time;

    }
//    common::PID pid;
// PID controllers for each joint :
// hip
    common::PID pid_hpy;
//shoulders
    common::PID pid_lsp;
    common::PID pid_lsr;
    common::PID pid_lsy;
    common::PID pid_rsp;
    common::PID pid_rsr;
    common::PID pid_rsy;

// elbow
    common::PID pid_lep;
    common::PID pid_rep;

// wrist
    common::PID pid_lwy;
    common::PID pid_lwp;
    common::PID pid_rwy;
    common::PID pid_rwp;

// leg joints
    common::PID pid_lhp;
    common::PID pid_lhr;
    common::PID pid_lhy;
    common::PID pid_lkp;
    common::PID pid_lap;
    common::PID pid_lar;
    common::PID pid_rhp;
    common::PID pid_rhr;
    common::PID pid_rhy;
    common::PID pid_rkp;
    common::PID pid_rap;
    common::PID pid_rar;

// neck

    common::PID pid_HNP;

// pid controllers for the leg joints

    double target_position_;
    double hip_target_position_;
    double lshoulder_target_position_;
    double rshoulder_target_position_;
    double lkp_;
    double lap_;
    double lhp_;
    double rkp_;
    double rap_;
    double rhp_;
    double loop_No;
    int dir;
// ** Pointers for each joints

// hip joint
    physics::JointPtr hpy_joint_;
// shoulder
    physics::JointPtr lsp_joint_;
    physics::JointPtr lsr_joint_;
    physics::JointPtr lsy_joint_;
    physics::JointPtr rsp_joint_;
    physics::JointPtr rsr_joint_;
    physics::JointPtr rsy_joint_;
//elbow
    physics::JointPtr lep_joint_;
    physics::JointPtr rep_joint_;
// wrist
    physics::JointPtr lwy_joint_;
    physics::JointPtr lwp_joint_;
    physics::JointPtr rwy_joint_;
    physics::JointPtr rwp_joint_;
// legs
    physics::JointPtr lkp_joint_;
    physics::JointPtr lap_joint_;
    physics::JointPtr lar_joint_;
    physics::JointPtr lhr_joint_;
    physics::JointPtr lhp_joint_;
    physics::JointPtr lhy_joint_;
    physics::JointPtr rhr_joint_;
    physics::JointPtr rhp_joint_;
    physics::JointPtr rkp_joint_;
    physics::JointPtr rap_joint_;
    physics::JointPtr rar_joint_;
    physics::JointPtr rhy_joint_;
// neck
    physics::JointPtr HNP_joint_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PIDJoints)
}
