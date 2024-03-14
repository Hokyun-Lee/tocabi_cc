#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <random>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <vector>

#define M_PI    3.14159265358979323846
#define Rad2Deg 180/M_PI
#define Deg2Rad M_PI/180

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    //////////////////////////////////////////// Donghyeon RL /////////////////////////////////////////
    void loadNetwork();
    void processNoise();
    void processObservation();
    void feedforwardPolicy();
    void initVariable();
    Eigen::Vector3d mat2euler(Eigen::Matrix3d mat);

    static const int num_action = 21;
    static const int num_actuator_action = 20;
    static const int num_cur_state = 66;
    static const int num_cur_internal_state = 49;
    static const int num_state_skip = 2;
    static const int num_state_hist = 5;
    static const int num_state = num_cur_internal_state*num_state_hist+num_action*(num_state_hist-1);
    static const int num_hidden = 256;

    Eigen::MatrixXd policy_net_w0_;
    Eigen::MatrixXd policy_net_b0_;
    Eigen::MatrixXd policy_net_w2_;
    Eigen::MatrixXd policy_net_b2_;
    Eigen::MatrixXd action_net_w_;
    Eigen::MatrixXd action_net_b_;
    Eigen::MatrixXd hidden_layer1_;
    Eigen::MatrixXd hidden_layer2_;
    Eigen::MatrixXd rl_action_;

    Eigen::MatrixXd value_net_w0_;
    Eigen::MatrixXd value_net_b0_;
    Eigen::MatrixXd value_net_w2_;
    Eigen::MatrixXd value_net_b2_;
    Eigen::MatrixXd value_net_w_;
    Eigen::MatrixXd value_net_b_;
    Eigen::MatrixXd value_hidden_layer1_;
    Eigen::MatrixXd value_hidden_layer2_;
    double value_;

    bool stop_by_value_thres_ = false;
    Eigen::Matrix<double, MODEL_DOF, 1> q_stop_;
    float stop_start_time_;
    
    Eigen::MatrixXd state_;
    Eigen::MatrixXd state_cur_;
    Eigen::MatrixXd state_buffer_;
    Eigen::MatrixXd state_mean_;
    Eigen::MatrixXd state_var_;

    std::ofstream writeFile;

    float phase_ = 0.0;
    // float pi = 3.1415926535;

    bool is_on_robot_ = false;
    bool is_write_file_ = true;
    Eigen::Matrix<double, MODEL_DOF, 1> q_dot_lpf_;

    Eigen::Matrix<double, MODEL_DOF, 1> q_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_noise_pre_;
    Eigen::Matrix<double, MODEL_DOF, 1> q_vel_noise_;

    Eigen::Matrix<double, MODEL_DOF, 1> torque_init_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_spline_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_rl_;
    Eigen::Matrix<double, MODEL_DOF, 1> torque_bound_;

    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp_;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv_;

    float start_time_;
    float time_inference_pre_ = 0.0;
    float time_write_pre_ = 0.0;

    double time_cur_;
    double time_pre_;
    double action_dt_accumulate_ = 0.0;

    Eigen::Vector3d euler_angle_;

    // float ft_left_init_ = 500.0;
    // float ft_right_init_ = 500.0;

    // Joystick
    ros::NodeHandle nh_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub_;

    double target_vel_x_ = 0.0;
    double target_vel_y_ = 0.0;

    // HK
    Eigen::VectorQd ref_q_;
    int upper_init_tick_hk_ = 0;
    double hz_ = 2000.0;
    Eigen::VectorQd upper_init_q_;
    Eigen::VectorQd Initial_ref_q_;
    Eigen::VectorQd Initial_ref_upper_q_;
    Eigen::VectorQd Gravity_;

    std::vector<int> joint_pd_index = {12,
                                       15, 18, 20, 21, 22,
                                       23, 24,
                                       25, 28, 30, 31, 32};
    std::vector<int> actuator_action_index = {0, 1, 2, 3, 4,  5,
                                              6, 7, 8, 9, 10, 11,
                                              13, 14,
                                              16,17,19,
                                              26,27,29};

private:
    Eigen::VectorQd ControlVal_;
};