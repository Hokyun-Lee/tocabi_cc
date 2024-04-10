#include "cc.h"

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    haptic_pose_sub_ = nh_cc_.subscribe("/haptic/pose", 100, &CustomController::HapticPoseCallback, this);
    haptic_force_pub_ = nh_cc_.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
    // example_subsciber = nh_cc_.subscribe("/some/topic",10,&CustomController::myCallback,this);
    ControlVal_.setZero();

    string urdf_path;

    ros::param::get("/tocabi_controller/urdf_path", urdf_path);

    drd_.LoadModelData(urdf_path, true, true);

    drd_.AddContactConstraint("l_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    drd_.AddContactConstraint("r_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    drd_.AddContactConstraint("l_wrist2_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    drd_.AddContactConstraint("r_wrist2_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
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
    //MODE 6,7,8,9 is reserved for cc
    queue_cc_.callAvailable(ros::WallDuration());

    
    if (rd_.tc_.mode == 8)
    {
        static double time_start_mode2 = 0.0;
        double ang2rad = 0.0174533;
        drd_.UpdateKinematics(rd_.q_virtual_, rd_.q_dot_virtual_, rd_.q_ddot_virtual_);
        drd_.control_time_ = rd_.control_time_;

        int drd_lh_id = drd_.getLinkID("l_wrist2_link");
        int drd_rh_id = drd_.getLinkID("r_wrist2_link");
        int drd_ub_id = drd_.getLinkID("upperbody_link");
        int drd_pl_id = drd_.getLinkID("pelvis_link");
        int drd_com_id = drd_.getLinkID("COM");

        static bool init_qp;
        if (rd_.tc_init)
        {

            if (rd_.tc_.customTaskGain)
            {
                rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
                rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            }

            init_qp = true;

            if (rd_.tc_.solver == 0)
            {
                std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: ORIGINAL " << std::endl;
            }
            else if (rd_.tc_.solver == 1)
            {
                std::cout << "TASK MODE 0 : 2LEVEL TASK EXPERIMENT :::: REDUCED " << std::endl;
            }
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
            drd_.ClearTaskSpace();
            // drd_.AddTaskSpace(DWBC::TASK_CUSTOM, 6);
            drd_.AddTaskSpace(0, DWBC::TASK_LINK_POSITION, "COM", Vector3d::Zero());

            if (rd_.tc_.maintain_lc)
            {
                std::cout << "maintain lc" << std::endl;
                rd_.link_[COM_id].x_init = rd_.link_[Pelvis].x_desired;

                rd_.link_[Pelvis].rot_init = rd_.link_[Pelvis].rot_desired;

                rd_.link_[Upper_Body].rot_init = rd_.link_[Upper_Body].rot_desired;

                rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].x_desired;
                rd_.link_[Left_Hand].rot_init = drd_.ts_[3].task_link_[0].rot_traj;

                rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].x_desired;
                rd_.link_[Right_Hand].rot_init = drd_.ts_[3].task_link_[1].rot_traj;
            }

            rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
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

        int d1, d2, d3, d4, d5;
        std::chrono::time_point<std::chrono::steady_clock> t0, t1, t2, t3, t4, t5;

        int ret1, ret2;

        if (rd_.tc_.solver == 0)
        {
            std::cout << "rd_.tc_.solver == 0" << std:: endl;


            t0 = std::chrono::steady_clock::now();

            drd_.SetContact(1, 1);
            drd_.CalcContactConstraint();
            drd_.CalcGravCompensation();

            t1 = std::chrono::steady_clock::now();

            drd_.CalcTaskSpace();

            t2 = std::chrono::steady_clock::now();

            ret1 = drd_.CalcTaskControlTorque(true, init_qp, false);

            t3 = std::chrono::steady_clock::now();

            ret2 = drd_.CalcContactRedistribute(true, init_qp);

            t4 = std::chrono::steady_clock::now();

            t5 = std::chrono::steady_clock::now();
        }
        else if (rd_.tc_.solver == 1)
        {
            std::cout << "rd_.tc_.solver == 1" << std:: endl;
            

            t0 = std::chrono::steady_clock::now();

            drd_.SetContact(1, 1);
            // drd_.CalcContactConstraint();
            drd_.ReducedDynamicsCalculate();
            t1 = std::chrono::steady_clock::now();

            drd_.ReducedCalcContactConstraint();
            drd_.ReducedCalcGravCompensation();

            t2 = std::chrono::steady_clock::now();

            drd_.ReducedCalcTaskSpace();

            t3 = std::chrono::steady_clock::now();

            ret1 = drd_.ReducedCalcTaskControlTorque(true, init_qp, false);

            t4 = std::chrono::steady_clock::now();

            ret2 = drd_.ReducedCalcContactRedistribute(true, init_qp);

            t5 = std::chrono::steady_clock::now();
        }

        d1 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        d2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        d3 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        d4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
        d5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

        rd_.torque_desired = drd_.torque_task_ + drd_.torque_grav_ + drd_.torque_contact_;

        // std::cout << "drd_.torque_task_ : \n" << drd_.torque_task_ << std:: endl;
        // std::cout << "drd_.torque_contact_ : \n" << drd_.torque_contact_ << std:: endl;
        // std::cout << "drd_.ts_[0].Lambda_task_ : \n" << drd_.ts_[0].Lambda_task_ << std:: endl;
        // std::cout << "drd_.ts_[0].J_kt_ : \n" << drd_.ts_[0].J_kt_ << std:: endl;
        // std::cout << "drd_.ts_[0].f_star_ : \n" << drd_.ts_[0].f_star_ << std:: endl;

        if (!ret1)
        {
            rd_.positionControlSwitch = true;
            std::cout << "task control error" << std::endl;
        }
        if (!ret2)
        {
            rd_.positionControlSwitch = true;
            std::cout << "contact control error" << std::endl;
        }

        init_qp = false;

        Vector3d plv_rpy = DyrosMath::rot2Euler(drd_.link_[drd_pl_id].rotm);
        Vector3d ub_rpy = DyrosMath::rot2Euler(drd_.link_[drd_ub_id].rotm);

        Vector3d plv_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[1].task_link_[0].rot_traj);
        Vector3d ub_rpy_traj = DyrosMath::rot2Euler(drd_.ts_[2].task_link_[0].rot_traj);

        // tf2::RotationMatrix plv_rot;

        for (int i = 0; i < 3; i++)
        {
            if (plv_rpy(i) > 0.5 * M_PI)
            {
                plv_rpy(i) -= M_PI;
            }
            else if (plv_rpy(i) < -0.5 * M_PI)
            {
                plv_rpy(i) += M_PI;
            }
            if (ub_rpy(i) > 0.5 * M_PI)
            {
                ub_rpy(i) -= M_PI;
            }
            else if (ub_rpy(i) < -0.5 * M_PI)
            {
                ub_rpy(i) += M_PI;
            }
            if (plv_rpy_traj(i) > 0.5 * M_PI)
            {
                plv_rpy_traj(i) -= M_PI;
            }
            else if (plv_rpy_traj(i) < -0.5 * M_PI)
            {
                plv_rpy_traj(i) += M_PI;
            }
            if (ub_rpy_traj(i) > 0.5 * M_PI)
            {
                ub_rpy_traj(i) -= M_PI;
            }
            else if (ub_rpy_traj(i) < -0.5 * M_PI)
            {
                ub_rpy_traj(i) += M_PI;
            }
        }
    }
    else if (rd_.tc_.mode == 9)
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