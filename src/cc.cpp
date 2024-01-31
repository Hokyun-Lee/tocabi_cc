#include "cc.h"

using namespace TOCABI;



// CustomController::CustomController(DWBC::RobotData drd) : rdd_(drd) //, wbc_(dc.wbc_)
CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    haptic_pose_sub_ = nh_cc_.subscribe("/haptic/pose", 100, &CustomController::HapticPoseCallback, this);
    haptic_force_pub_ = nh_cc_.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
    // example_subsciber = nh_cc_.subscribe("/some/topic",10,&CustomController::myCallback,this);
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::PublishHapticData()
{
    geometry_msgs::Vector3 force;
    force.x = haptic_force_[0];
    force.y = haptic_force_[1];
    force.z = haptic_force_[2];

    haptic_force_pub_.publish(force);
}

void CustomController::computeSlow()
{
    //MODE 6,7,8 is reserved for cc
    queue_cc_.callAvailable(ros::WallDuration());

    //CC for Hokyun Lee(hkleetony@snu.ac.kr)
    if (rd_.tc_.mode == 8){
        static double time_start_mode2 = 0.0;
        double ang2rad = 0.0174533;
        
        //rd_.q_virtual_ : (40x1) 33dof+3(virtual pos)+4(virtual ori, quat)
        //rd_.q_dot_virtual_ : (39x1) 33dof+6dof(virtual)
        //rd_.q_ddot_virtual_ : (39x1) 33dof+6dof(virtual)
        drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
        drd_.control_time_ = rd_.control_time_;
        
        // int drd_lh_id = drd_.getLinkID("l_wrist2_link");
        // int drd_rh_id = drd_.getLinkID("r_wrist2_link");
        // int drd_ub_id = drd_.getLinkID("upperbody_link");
        // int drd_pl_id = drd_.getLinkID("pelvis_link");
        // int drd_com_id = drd_.getLinkID("COM");

        //TASK 1. Maintain COM POS
        int drd_com_id = drd_.getLinkID("COM");
        //TASK 2. Maintain Pelvis ORI
        int drd_pl_id = drd_.getLinkID("pelvis_link");
        //TASK 3. Left/Right hand POS
        int drd_lh_id = drd_.getLinkID("l_wrist2_link");
        int drd_rh_id = drd_.getLinkID("r_wrist2_link");

        static bool init_qp;
        if (rd_.tc_init)
        {
            if (rd_.tc_.customTaskGain)
            {
                rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            }

            init_qp = true;

            // if (rd_.tc_.solver == 0)
            // {
            //     std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: ORIGINAL " << std::endl;
            // }
            // else if (rd_.tc_.solver == 1)
            // {
            //     std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: REDUCED " << std::endl;
            // }
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
            drd_.ClearTaskSpace();
            // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
            drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());
            
            // if (rd_.tc_.maintain_lc)
            // {
            //     std::cout << "maintain lc" << std::endl;
            //     rd_.link_[COM_id].x_init = rd_.link_[Pelvis].x_desired;

            //     rd_.link_[Pelvis].rot_init = rd_.link_[Pelvis].rot_desired;

            //     rd_.link_[Upper_Body].rot_init = rd_.link_[Upper_Body].rot_desired;

            //     rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].x_desired;
            //     rd_.link_[Left_Hand].rot_init = drd_.ts_[3].task_link_[0].rot_traj;

            //     rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].x_desired;
            //     rd_.link_[Right_Hand].rot_init = drd_.ts_[3].task_link_[1].rot_traj;
            // }

            rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
            std::cout << "rd_.link_[Pelvis].x_desired" << std::endl;
            std::cout << rd_.link_[Pelvis].x_desired << std::endl;
            rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
            rd_.link_[Pelvis].rot_desired = DyrosMath::Euler2rot(0, rd_.tc_.pelv_pitch * ang2rad, rd_.link_[Pelvis].yaw_init);
            rd_.link_[Upper_Body].rot_desired = DyrosMath::Euler2rot(rd_.tc_.roll * ang2rad, rd_.tc_.pitch * ang2rad, rd_.tc_.yaw * ang2rad + rd_.link_[Pelvis].yaw_init);

            Vector3d com_diff = rd_.link_[Pelvis].x_desired - rd_.link_[COM_id].x_init;

            rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init + com_diff;
            rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init + com_diff;

            drd_.ts_[0].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
            drd_.ts_[0].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[COM_id].x_init, Vector3d::Zero(), rd_.link_[Pelvis].x_desired, Vector3d::Zero());
            // drd_.ts_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

            drd_.AddTaskSpace(1, DWBC::TASK_LINK_ROTATION, "pelvis_link", Vector3d::Zero());
            drd_.ts_[1].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
            drd_.ts_[1].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].rot_init, Vector3d::Zero(), rd_.link_[Pelvis].rot_desired, Vector3d::Zero());

            drd_.AddTaskSpace(2, DWBC::TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
            drd_.ts_[2].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
            drd_.ts_[2].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Upper_Body].rot_init, Vector3d::Zero(), rd_.link_[Upper_Body].rot_desired, Vector3d::Zero());

            drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "l_wrist2_link", Vector3d::Zero());
            drd_.ts_[3].task_link_[0].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
            drd_.ts_[3].task_link_[0].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].x_init, Vector3d::Zero(), rd_.link_[Left_Hand].x_desired, Vector3d::Zero());
            drd_.ts_[3].task_link_[0].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Left_Hand].rot_init, Vector3d::Zero(), rd_.link_[Left_Hand].rot_init, Vector3d::Zero());

            drd_.AddTaskSpace(3, DWBC::TASK_LINK_6D, "r_wrist2_link", Vector3d::Zero());
            drd_.ts_[3].task_link_[1].SetTaskGain(rd_.link_[0].pos_p_gain, rd_.link_[0].pos_d_gain, rd_.link_[0].pos_a_gain, rd_.link_[0].rot_p_gain, rd_.link_[0].rot_d_gain, rd_.link_[0].rot_a_gain);
            drd_.ts_[3].task_link_[1].SetTrajectoryQuintic(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].x_init, Vector3d::Zero(), rd_.link_[Right_Hand].x_desired, Vector3d::Zero());
            drd_.ts_[3].task_link_[1].SetTrajectoryRotation(rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Right_Hand].rot_init, Vector3d::Zero(), rd_.link_[Right_Hand].rot_init, Vector3d::Zero());
        }





    }
    else if (rd_.tc_.mode == 6)
    {   
        double ang2rad = 0.0174533;

        static bool init_qp;
        
        static VectorQd init_q_mode6;

        static Matrix3d rot_hand_init;
        static Matrix3d rot_haptic_init;

        static Vector3d pos_hand_init;
        static Vector3d pos_haptic_init;


        if (rd_.tc_init)
        {
            init_qp = true;

            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            init_q_mode6 = rd_.q_;

            rot_hand_init = rd_.link_[Right_Hand].rotm;
            rot_haptic_init = haptic_orientation_;

            pos_hand_init = rd_.link_[Right_Hand].xpos;
            pos_haptic_init = haptic_pos_;

        }

        WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
        if (rd_.tc_.customTaskGain)
        {
            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        }

        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
        rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(rd_.tc_.pelv_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.link_[Pelvis].yaw_init);

        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
        rd_.link_[Right_Hand].x_desired(0) += rd_.tc_.r_x;
        rd_.link_[Right_Hand].x_desired(1) += rd_.tc_.r_y;
        rd_.link_[Right_Hand].x_desired(2) += rd_.tc_.r_z;
        rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad) * DyrosMath::Euler2rot(0, 1.5708, -1.5708).transpose();
        // rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad);

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.yaw * ang2rad);

        rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
        rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        Vector3d hand_pos_desired = haptic_pos_ + pos_hand_init - pos_haptic_init;

        Matrix3d hand_rot_desired = rot_hand_init * rot_haptic_init.transpose() * haptic_orientation_;

        rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, hand_pos_desired);
        //rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);
        rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, hand_rot_desired, false);

        std::cout<<"pos"<<std::endl;
        std::cout<<haptic_pos_<<std::endl;
        std::cout<<"ori"<<std::endl;
        std::cout<<haptic_orientation_<< std::endl;

        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

        TaskSpace ts_(6);
        Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
        Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);

        ts_.Update(Jtask, fstar);
        WBC::CalcJKT(rd_, ts_);
        WBC::CalcTaskNull(rd_, ts_);
        static CQuadraticProgram task_qp_;
        WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

        VectorQd torque_Task2 = ts_.torque_h_ + rd_.torque_grav;

        TaskSpace ts1_(6);
        Eigen::MatrixXd Jtask1 = rd_.link_[Right_Hand].Jac();
        Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);

        ts1_.Update(Jtask1, fstar1);
        WBC::CalcJKT(rd_, ts1_);
        WBC::CalcTaskNull(rd_, ts1_);
        static CQuadraticProgram task_qp1_;
        WBC::TaskControlHQP(rd_, ts1_, task_qp1_, torque_Task2, ts_.Null_task, init_qp);

        torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + rd_.torque_grav;

        TaskSpace ts2_(3);
        Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
        Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
        ts2_.Update(Jtask2, fstar2);
        WBC::CalcJKT(rd_, ts2_);

        static CQuadraticProgram task_qp2_;
        WBC::TaskControlHQP(rd_, ts2_, task_qp2_, torque_Task2, ts_.Null_task * ts1_.Null_task, init_qp);

        torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + ts_.Null_task * ts1_.Null_task * ts2_.torque_h_ + rd_.torque_grav;

                    // rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (zero_m[i] - rd_.q_dot_[i]);
        VectorQd torque_pos_hold;

        for (int i=0;i<MODEL_DOF;i++)
        {
            torque_pos_hold[i] = rd_.pos_kp_v[i] * (init_q_mode6[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * ( - rd_.q_dot_[i]);
        }


        torque_pos_hold.segment(25,8).setZero();

        VectorQd torque_right_arm;
        
        torque_right_arm.setZero();

        torque_right_arm.segment(25,8) = WBC::ContactForceRedistributionTorque(rd_, torque_Task2).segment(25,8);

        rd_.torque_desired = torque_pos_hold + torque_right_arm;
        
        std::cout <<"torque" << rd_.RH_CF_FT<< std::endl;

        haptic_force_[0] = rd_.RH_CF_FT[0] * 0.1;
        haptic_force_[1] = rd_.RH_CF_FT[1]*0.1;
        haptic_force_[2] = rd_.RH_CF_FT[2]*0.1;

        PublishHapticData();

        init_qp = false;
    }
    else if (rd_.tc_.mode == 7)
    {
        // reserved
    }
}

void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
    }
    else if (rd_.tc_.mode == 7)
    {
    }
}

void CustomController::HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{

    float pos_x = CustomController::PositionMapping(msg -> position.x, 0);
    float pos_y = CustomController::PositionMapping(msg -> position.y, 1);
    float pos_z = CustomController::PositionMapping(msg -> position.z, 2);
    float ori_x = CustomController::PositionMapping(msg -> orientation.x, 3);
    float ori_y = CustomController::PositionMapping(msg -> orientation.y, 4);
    float ori_z = CustomController::PositionMapping(msg -> orientation.z, 5);
    float ori_w = CustomController::PositionMapping(msg -> orientation.w, 6);

    // double posx = static_cast<double>(pos_x);
    // double posy = static_cast<double>(pos_y); 
    // double posz = static_cast<double>(pos_z);
    double orix = static_cast<double>(ori_x);
    double oriy = static_cast<double>(ori_y); 
    double oriz = static_cast<double>(ori_z);
    double oriw = static_cast<double>(ori_w);

    haptic_pos_[0] = pos_x;
    haptic_pos_[1] = pos_y; 
    haptic_pos_[2] = pos_z;

    haptic_orientation_ = CustomController::Quat2rotmatrix(orix, oriy, oriz, oriw);

}

float CustomController::PositionMapping(float haptic_val, int i)
{
    if (i == 0){
        return -1 * (haptic_val + 0.051448) * 5.0 ;
    }

    else if(i == 1){
        return -1 * (haptic_val + 0.000152) * 5.0;
    }

    else if (i == 2){
        return (haptic_val - 0.007794) * 5.0;
    }
    else {
     return haptic_val;
    }
    
}

Eigen::Matrix3d CustomController::Quat2rotmatrix(double q0, double q1, double q2, double q3)
{
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1 ;
    double r01 = 2 * (q1 * q2 - q0 * q3) ;
    double r02 = 2 * (q1 * q3 + q0 * q2) ;

    double r10 = 2 * (q1 * q2 + q0 * q3) ;
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1 ;
    double r12 = 2 * (q2 * q3 - q0 * q1) ;

    double r20 = 2 * (q1 * q3 - q0 * q2) ;
    double r21 = 2 * (q2 * q3 + q0 * q1) ;
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1 ;

    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;
    return rot_matrix;
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}