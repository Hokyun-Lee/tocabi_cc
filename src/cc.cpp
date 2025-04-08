#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();

    if (is_write_file_)
    {
        if (is_on_robot_)
        {
            writeFile.open("/home/dyros/catkin_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        else
        {
            writeFile.open("/home/dyros24/cadence/ros_ws/src/tocabi_cc/result/data.csv", std::ofstream::out | std::ofstream::app);
        }
        writeFile << std::fixed << std::setprecision(8);
    }
    initVariable();
    loadNetwork();

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &CustomController::joyCallback, this);
    // mujoco_ext_force_apply_pub = nh_.advertise<std_msgs::Float32MultiArray>("/tocabi_avatar/applied_ext_force", 10);
    mujoco_ext_force_apply_pub = nh_.advertise<mujoco_ros_msgs::applyforce>("/mujoco_ros_interface/applied_ext_force", 10);
    // mujoco_applied_ext_force_.data.resize(7);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::loadNetwork()
{
    state_.setZero();
    rl_action_.setZero();
    balance_rl_action_.setZero();


    string cur_path = "/home/dyros24/cadence/ros_ws/src/tocabi_cc/";

    if (is_on_robot_)
    {
        cur_path = "/home/dyros/catkin_ws/src/tocabi_cc/";
    }
    // std::ifstream file[15];
    // file[0].open(cur_path+"weight/a2c_network_actor_mlp_0_weight.txt", std::ios::in);
    // file[1].open(cur_path+"weight/a2c_network_actor_mlp_0_bias.txt", std::ios::in);
    // file[2].open(cur_path+"weight/a2c_network_actor_mlp_2_weight.txt", std::ios::in);
    // file[3].open(cur_path+"weight/a2c_network_actor_mlp_2_bias.txt", std::ios::in);
    // file[4].open(cur_path+"weight/a2c_network_mu_weight.txt", std::ios::in);
    // file[5].open(cur_path+"weight/a2c_network_mu_bias.txt", std::ios::in);
    // file[6].open(cur_path+"weight/obs_mean_fixed.txt", std::ios::in);
    // file[7].open(cur_path+"weight/obs_variance_fixed.txt", std::ios::in);
    // // file[6].open(cur_path+"weight/obs_mean_fixed_cadence.txt", std::ios::in);
    // // file[7].open(cur_path+"weight/obs_variance_fixed_cadence.txt", std::ios::in);
    // file[8].open(cur_path+"weight/a2c_network_critic_mlp_0_weight.txt", std::ios::in);
    // file[9].open(cur_path+"weight/a2c_network_critic_mlp_0_bias.txt", std::ios::in);
    // file[10].open(cur_path+"weight/a2c_network_critic_mlp_2_weight.txt", std::ios::in);
    // file[11].open(cur_path+"weight/a2c_network_critic_mlp_2_bias.txt", std::ios::in);
    // file[12].open(cur_path+"weight/a2c_network_value_weight.txt", std::ios::in);
    // file[13].open(cur_path+"weight/a2c_network_value_bias.txt", std::ios::in);
    // file[14].open(cur_path+"weight/commands.txt", std::ios::in);

    std::ifstream file[31];
    file[0].open(cur_path+"weight/loco_policy/a2c_network_actor_mlp_0_weight.txt", std::ios::in);
    file[1].open(cur_path+"weight/loco_policy/a2c_network_actor_mlp_0_bias.txt", std::ios::in);
    file[2].open(cur_path+"weight/loco_policy/a2c_network_actor_mlp_2_weight.txt", std::ios::in);
    file[3].open(cur_path+"weight/loco_policy/a2c_network_actor_mlp_2_bias.txt", std::ios::in);
    file[4].open(cur_path+"weight/loco_policy/a2c_network_mu_weight.txt", std::ios::in);
    file[5].open(cur_path+"weight/loco_policy/a2c_network_mu_bias.txt", std::ios::in);
    file[6].open(cur_path+"weight/loco_policy/obs_mean_fixed.txt", std::ios::in);
    file[7].open(cur_path+"weight/loco_policy/obs_variance_fixed.txt", std::ios::in);
    // file[6].open(cur_path+"weight/obs_mean_fixed_cadence.txt", std::ios::in);
    // file[7].open(cur_path+"weight/obs_variance_fixed_cadence.txt", std::ios::in);
    file[8].open(cur_path+"weight/loco_policy/a2c_network_critic_mlp_0_weight.txt", std::ios::in);
    file[9].open(cur_path+"weight/loco_policy/a2c_network_critic_mlp_0_bias.txt", std::ios::in);
    file[10].open(cur_path+"weight/loco_policy/a2c_network_critic_mlp_2_weight.txt", std::ios::in);
    file[11].open(cur_path+"weight/loco_policy/a2c_network_critic_mlp_2_bias.txt", std::ios::in);
    file[12].open(cur_path+"weight/loco_policy/a2c_network_value_weight.txt", std::ios::in);
    file[13].open(cur_path+"weight/loco_policy/a2c_network_value_bias.txt", std::ios::in);

    file[14].open(cur_path+"weight/commands.txt", std::ios::in);

    if (file[14].is_open()) {
        file[14] >> target_vel_x_cmd >> target_vel_y_cmd >> target_vel_yaw_cmd >> target_cadence_cmd;
        std::cout << "target_vel_x_cmd : " << target_vel_x_cmd << std::endl;
        std::cout << "target_vel_y_cmd : " << target_vel_y_cmd << std::endl;
        std::cout << "target_vel_yaw_cmd : " << target_vel_yaw_cmd << std::endl;
        std::cout << "target_cadence_cmd : " << target_cadence_cmd << std::endl;
    } else {
        std::cout << "failed to open commands.txt" << std::endl;
    }

    file[15].open(cur_path+"weight/balance_policy/a2c_network_actor_mlp_0_weight.txt", std::ios::in);
    file[16].open(cur_path+"weight/balance_policy/a2c_network_actor_mlp_0_bias.txt", std::ios::in);
    file[17].open(cur_path+"weight/balance_policy/a2c_network_actor_mlp_2_weight.txt", std::ios::in);
    file[18].open(cur_path+"weight/balance_policy/a2c_network_actor_mlp_2_bias.txt", std::ios::in);
    file[19].open(cur_path+"weight/balance_policy/a2c_network_mu_weight.txt", std::ios::in);
    file[20].open(cur_path+"weight/balance_policy/a2c_network_mu_bias.txt", std::ios::in);
    file[21].open(cur_path+"weight/balance_policy/obs_mean_fixed.txt", std::ios::in);
    file[22].open(cur_path+"weight/balance_policy/obs_variance_fixed.txt", std::ios::in);
    file[23].open(cur_path+"weight/balance_policy/a2c_network_critic_mlp_0_weight.txt", std::ios::in);
    file[24].open(cur_path+"weight/balance_policy/a2c_network_critic_mlp_0_bias.txt", std::ios::in);
    file[25].open(cur_path+"weight/balance_policy/a2c_network_critic_mlp_2_weight.txt", std::ios::in);
    file[26].open(cur_path+"weight/balance_policy/a2c_network_critic_mlp_2_bias.txt", std::ios::in);
    file[27].open(cur_path+"weight/balance_policy/a2c_network_value_weight.txt", std::ios::in);
    file[28].open(cur_path+"weight/balance_policy/a2c_network_value_bias.txt", std::ios::in);


    if(!file[0].is_open())
    {
        std::cout<<"Can not find the weight file"<<std::endl;
    }

    float temp;
    int row = 0;
    int col = 0;

    while(!file[0].eof() && row != policy_net_w0_.rows())
    {
        file[0] >> temp;
        if(temp != '\n')
        {
            policy_net_w0_(row, col) = temp;
            col ++;
            if (col == policy_net_w0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[1].eof() && row != policy_net_b0_.rows())
    {
        file[1] >> temp;
        if(temp != '\n')
        {
            policy_net_b0_(row, col) = temp;
            col ++;
            if (col == policy_net_b0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[2].eof() && row != policy_net_w2_.rows())
    {
        file[2] >> temp;
        if(temp != '\n')
        {
            policy_net_w2_(row, col) = temp;
            col ++;
            if (col == policy_net_w2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[3].eof() && row != policy_net_b2_.rows())
    {
        file[3] >> temp;
        if(temp != '\n')
        {
            policy_net_b2_(row, col) = temp;
            col ++;
            if (col == policy_net_b2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[4].eof() && row != action_net_w_.rows())
    {
        file[4] >> temp;
        if(temp != '\n')
        {
            action_net_w_(row, col) = temp;
            col ++;
            if (col == action_net_w_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[5].eof() && row != action_net_b_.rows())
    {
        file[5] >> temp;
        if(temp != '\n')
        {
            action_net_b_(row, col) = temp;
            col ++;
            if (col == action_net_b_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[6].eof() && row != state_mean_.rows())
    {
        file[6] >> temp;
        if(temp != '\n')
        {
            state_mean_(row, col) = temp;
            col ++;
            if (col == state_mean_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[7].eof() && row != state_var_.rows())
    {
        file[7] >> temp;
        if(temp != '\n')
        {
            state_var_(row, col) = temp;
            col ++;
            if (col == state_var_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[8].eof() && row != value_net_w0_.rows())
    {
        file[8] >> temp;
        if(temp != '\n')
        {
            value_net_w0_(row, col) = temp;
            col ++;
            if (col == value_net_w0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[9].eof() && row != value_net_b0_.rows())
    {
        file[9] >> temp;
        if(temp != '\n')
        {
            value_net_b0_(row, col) = temp;
            col ++;
            if (col == value_net_b0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[10].eof() && row != value_net_w2_.rows())
    {
        file[10] >> temp;
        if(temp != '\n')
        {
            value_net_w2_(row, col) = temp;
            col ++;
            if (col == value_net_w2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[11].eof() && row != value_net_b2_.rows())
    {
        file[11] >> temp;
        if(temp != '\n')
        {
            value_net_b2_(row, col) = temp;
            col ++;
            if (col == value_net_b2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[12].eof() && row != value_net_w_.rows())
    {
        file[12] >> temp;
        if(temp != '\n')
        {
            value_net_w_(row, col) = temp;
            col ++;
            if (col == value_net_w_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[13].eof() && row != value_net_b_.rows())
    {
        file[13] >> temp;
        if(temp != '\n')
        {
            value_net_b_(row, col) = temp;
            col ++;
            if (col == value_net_b_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }

    //////////////////////////////////balance policy
    /////////////////////////////////////////////////
    if(!file[15].is_open())
    {
        std::cout<<"Can not find the weight file"<<std::endl;
    }
    row = 0;
    col = 0;
    while(!file[15].eof() && row != balance_policy_net_w0_.rows())
    {
        file[15] >> temp;
        if(temp != '\n')
        {
            balance_policy_net_w0_(row, col) = temp;
            col ++;
            if (col == balance_policy_net_w0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[16].eof() && row != balance_policy_net_b0_.rows())
    {
        file[16] >> temp;
        if(temp != '\n')
        {
            balance_policy_net_b0_(row, col) = temp;
            col ++;
            if (col == balance_policy_net_b0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[17].eof() && row != balance_policy_net_w2_.rows())
    {
        file[17] >> temp;
        if(temp != '\n')
        {
            balance_policy_net_w2_(row, col) = temp;
            col ++;
            if (col == balance_policy_net_w2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[18].eof() && row != balance_policy_net_b2_.rows())
    {
        file[18] >> temp;
        if(temp != '\n')
        {
            balance_policy_net_b2_(row, col) = temp;
            col ++;
            if (col == balance_policy_net_b2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[19].eof() && row != balance_action_net_w_.rows())
    {
        file[19] >> temp;
        if(temp != '\n')
        {
            balance_action_net_w_(row, col) = temp;
            col ++;
            if (col == balance_action_net_w_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[20].eof() && row != balance_action_net_b_.rows())
    {
        file[20] >> temp;
        if(temp != '\n')
        {
            balance_action_net_b_(row, col) = temp;
            col ++;
            if (col == balance_action_net_b_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[21].eof() && row != balance_state_mean_.rows())
    {
        file[21] >> temp;
        if(temp != '\n')
        {
            balance_state_mean_(row, col) = temp;
            col ++;
            if (col == balance_state_mean_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[22].eof() && row != balance_state_var_.rows())
    {
        file[22] >> temp;
        if(temp != '\n')
        {
            balance_state_var_(row, col) = temp;
            col ++;
            if (col == balance_state_var_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[23].eof() && row != balance_value_net_w0_.rows())
    {
        file[23] >> temp;
        if(temp != '\n')
        {
            balance_value_net_w0_(row, col) = temp;
            col ++;
            if (col == balance_value_net_w0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[24].eof() && row != balance_value_net_b0_.rows())
    {
        file[24] >> temp;
        if(temp != '\n')
        {
            balance_value_net_b0_(row, col) = temp;
            col ++;
            if (col == balance_value_net_b0_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[25].eof() && row != balance_value_net_w2_.rows())
    {
        file[25] >> temp;
        if(temp != '\n')
        {
            balance_value_net_w2_(row, col) = temp;
            col ++;
            if (col == balance_value_net_w2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[26].eof() && row != balance_value_net_b2_.rows())
    {
        file[26] >> temp;
        if(temp != '\n')
        {
            balance_value_net_b2_(row, col) = temp;
            col ++;
            if (col == balance_value_net_b2_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[27].eof() && row != balance_value_net_w_.rows())
    {
        file[27] >> temp;
        if(temp != '\n')
        {
            balance_value_net_w_(row, col) = temp;
            col ++;
            if (col == balance_value_net_w_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
    row = 0;
    col = 0;
    while(!file[28].eof() && row != balance_value_net_b_.rows())
    {
        file[28] >> temp;
        if(temp != '\n')
        {
            balance_value_net_b_(row, col) = temp;
            col ++;
            if (col == balance_value_net_b_.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
}

void CustomController::initVariable()
{    
    policy_net_w0_.resize(num_hidden, num_state);
    policy_net_b0_.resize(num_hidden, 1);
    policy_net_w2_.resize(num_hidden, num_hidden);
    policy_net_b2_.resize(num_hidden, 1);
    action_net_w_.resize(num_action, num_hidden);
    action_net_b_.resize(num_action, 1);
    hidden_layer1_.resize(num_hidden, 1);
    hidden_layer2_.resize(num_hidden, 1);
    rl_action_.resize(num_action, 1);

    value_net_w0_.resize(num_hidden, num_state);
    value_net_b0_.resize(num_hidden, 1);
    value_net_w2_.resize(num_hidden, num_hidden);
    value_net_b2_.resize(num_hidden, 1);
    value_net_w_.resize(1, num_hidden);
    value_net_b_.resize(1, 1);
    value_hidden_layer1_.resize(num_hidden, 1);
    value_hidden_layer2_.resize(num_hidden, 1);

    balance_policy_net_w0_.resize(num_hidden, num_state);
    balance_policy_net_b0_.resize(num_hidden, 1);
    balance_policy_net_w2_.resize(num_hidden, num_hidden);
    balance_policy_net_b2_.resize(num_hidden, 1);
    balance_action_net_w_.resize(num_action, num_hidden);
    balance_action_net_b_.resize(num_action, 1);
    balance_hidden_layer1_.resize(num_hidden, 1);
    balance_hidden_layer2_.resize(num_hidden, 1);
    balance_rl_action_.resize(num_action, 1);

    balance_value_net_w0_.resize(num_hidden, num_state);
    balance_value_net_b0_.resize(num_hidden, 1);
    balance_value_net_w2_.resize(num_hidden, num_hidden);
    balance_value_net_b2_.resize(num_hidden, 1);
    balance_value_net_w_.resize(1, num_hidden);
    balance_value_net_b_.resize(1, 1);
    balance_value_hidden_layer1_.resize(num_hidden, 1);
    balance_value_hidden_layer2_.resize(num_hidden, 1);
    
    state_cur_.resize(num_cur_state, 1);
    state_.resize(num_state, 1);
    state_buffer_.resize(num_cur_state*num_state_skip*num_state_hist, 1);
    state_mean_.resize(num_cur_state, 1);
    state_var_.resize(num_cur_state, 1);

    q_dot_lpf_.setZero();

    torque_bound_ << 333, 232, 263, 289, 222, 166,
                    333, 232, 263, 289, 222, 166,
                    303, 303, 303, 
                    64, 64, 64, 64, 23, 23, 10, 10,
                    10, 10,
                    64, 64, 64, 64, 23, 23, 10, 10;  
                    
    q_init_ << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
                0.0, 0.0, 0.0,
                0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
                0.0, 0.0,
                -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0;

    kp_.setZero();
    kv_.setZero();
    kp_.diagonal() <<   2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kp_.diagonal() /= 9.0;
    kv_.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;
    kv_.diagonal() /= 3.0;
}

Eigen::Vector3d CustomController::mat2euler(Eigen::Matrix3d mat)
{
    Eigen::Vector3d euler;

    double cy = std::sqrt(mat(2, 2) * mat(2, 2) + mat(1, 2) * mat(1, 2));
    if (cy > std::numeric_limits<double>::epsilon())
    {
        euler(2) = -atan2(mat(0, 1), mat(0, 0));
        euler(1) =  -atan2(-mat(0, 2), cy);
        euler(0) = -atan2(mat(1, 2), mat(2, 2));
    }
    else
    {
        euler(2) = -atan2(-mat(1, 0), mat(1, 1));
        euler(1) =  -atan2(-mat(0, 2), cy);
        euler(0) = 0.0;
    }
    return euler;
}

void CustomController::processNoise()
{
    time_cur_ = rd_cc_.control_time_us_ / 1e6;
    if (is_on_robot_)
    {
        q_vel_noise_ = rd_cc_.q_dot_virtual_.segment(6,MODEL_DOF);
        q_noise_= rd_cc_.q_virtual_.segment(6,MODEL_DOF);
        if (time_cur_ - time_pre_ > 0.0)
        {
            q_dot_lpf_ = DyrosMath::lpf<MODEL_DOF>(q_vel_noise_, q_dot_lpf_, 1/(time_cur_ - time_pre_), 4.0);
        }
        else
        {
            q_dot_lpf_ = q_dot_lpf_;
        }
    }
    else
    {
        std::random_device rd;  
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.00001, 0.00001);
        for (int i = 0; i < MODEL_DOF; i++) {
            q_noise_(i) = rd_cc_.q_virtual_(6+i) + dis(gen);
        }
        if (time_cur_ - time_pre_ > 0.0)
        {
            q_vel_noise_ = (q_noise_ - q_noise_pre_) / (time_cur_ - time_pre_);
            q_dot_lpf_ = DyrosMath::lpf<MODEL_DOF>(q_vel_noise_, q_dot_lpf_, 1/(time_cur_ - time_pre_), 4.0);
        }
        else
        {
            q_vel_noise_ = q_vel_noise_;
            q_dot_lpf_ = q_dot_lpf_;
        }
        q_noise_pre_ = q_noise_;
    }
    time_pre_ = time_cur_;
}

void CustomController::processObservation()
{
    int data_idx = 0;

    Eigen::Quaterniond q;
    q.x() = rd_cc_.q_virtual_(3);
    q.y() = rd_cc_.q_virtual_(4);
    q.z() = rd_cc_.q_virtual_(5);
    q.w() = rd_cc_.q_virtual_(MODEL_DOF_QVIRTUAL-1);    

    euler_angle_ = DyrosMath::rot2Euler_tf(q.toRotationMatrix());

    state_cur_(data_idx) = euler_angle_(0);
    data_idx++;

    state_cur_(data_idx) = euler_angle_(1);
    data_idx++;

    state_cur_(data_idx) = euler_angle_(2);
    data_idx++;


    for (int i = 0; i < num_actuator_action; i++)
    {
        state_cur_(data_idx) = q_noise_(i);
        data_idx++;
    }

    for (int i = 0; i < num_actuator_action; i++)
    {
        if (is_on_robot_)
        {
            state_cur_(data_idx) = q_vel_noise_(i);
        }
        else
        {
            state_cur_(data_idx) = q_vel_noise_(i); //rd_cc_.q_dot_virtual_(i+6);
        }
        data_idx++;
    }

    float squat_duration = 1.7995;
    phase_ = std::fmod((rd_cc_.control_time_us_-start_time_)/1e6 + action_dt_accumulate_, squat_duration) / squat_duration;

    state_cur_(data_idx) = sin(2*M_PI*phase_);
    data_idx++;
    state_cur_(data_idx) = cos(2*M_PI*phase_);
    data_idx++;

    float init_vel = 0.0;
    float max_vel = 0.4;

    if(walking_tick_hk_ > 0 && walking_tick_hk_ < 20000)
    {
        target_vel_x_ = init_vel;
    }
    else if(walking_tick_hk_ >= 20000 && walking_tick_hk_ < 40000)
    {
        target_vel_x_ = max_vel * (walking_tick_hk_ - 20000) / 20000;
    }
    else if(walking_tick_hk_ >= 40000 && walking_tick_hk_ < 60000)
    {
        target_vel_x_ = max_vel;
    }
    else if(walking_tick_hk_ >= 60000 && walking_tick_hk_ < 80000)
    {
        target_vel_x_ = max_vel * (80000 - walking_tick_hk_) / 20000;
    }
    else
    {
        target_vel_x_ = init_vel;
    }
    
    // state_cur_(data_idx) = 0.0;//target_vel_x_;
    state_cur_(data_idx) = target_vel_x_;
    // state_cur_(data_idx) = target_vel_x_cmd;
    data_idx++;

    // state_cur_(data_idx) = 0.0;//target_vel_y_;
    state_cur_(data_idx) = target_vel_y_cmd;
    data_idx++;

    for (int i=0; i<6; i++)
    {
        state_cur_(data_idx) = rd_cc_.q_dot_virtual_(i);
        data_idx++;
    }


    // state_cur_(data_idx) = 60.0;
    // state_cur_(data_idx) = target_cadence_cmd;
    // state_cur_(data_idx) = 10.0;
    // data_idx++;
    // state_cur_(data_idx) = -rd_cc_.LF_FT(2);
    // data_idx++;

    // state_cur_(data_idx) = -rd_cc_.RF_FT(2);
    // data_idx++;

    // state_cur_(data_idx) = rd_cc_.LF_FT(3);
    // data_idx++;

    // state_cur_(data_idx) = rd_cc_.RF_FT(3);
    // data_idx++;

    // state_cur_(data_idx) = rd_cc_.LF_FT(4);
    // data_idx++;

    // state_cur_(data_idx) = rd_cc_.RF_FT(4);
    // data_idx++;

    for (int i = 0; i <num_actuator_action; i++) 
    {
        if(loco_policy_on){
            state_cur_(data_idx) = DyrosMath::minmax_cut(rl_action_(i), -1.0, 1.0);
        }
        else{
            state_cur_(data_idx) = DyrosMath::minmax_cut(balance_rl_action_(i), -1.0, 1.0);
        }
        data_idx++;
    }
    if(loco_policy_on){
        state_cur_(data_idx) = DyrosMath::minmax_cut(rl_action_(num_actuator_action), 0.0, 1.0);
    }
    else{
        state_cur_(data_idx) = DyrosMath::minmax_cut(balance_rl_action_(num_actuator_action), 0.0, 1.0);
    }
    data_idx++;
    
    state_buffer_.block(0, 0, num_cur_state*(num_state_skip*num_state_hist-1),1) = state_buffer_.block(num_cur_state, 0, num_cur_state*(num_state_skip*num_state_hist-1),1);
    state_buffer_.block(num_cur_state*(num_state_skip*num_state_hist-1), 0, num_cur_state,1) = (state_cur_ - state_mean_).array() / state_var_.cwiseSqrt().array();

    // Internal State First
    for (int i = 0; i < num_state_hist; i++)
    {
        state_.block(num_cur_internal_state*i, 0, num_cur_internal_state, 1) = state_buffer_.block(num_cur_state*(num_state_skip*(i+1)-1), 0, num_cur_internal_state, 1);
    }
    // Action History Second
    for (int i = 0; i < num_state_hist-1; i++)
    {
        state_.block(num_state_hist*num_cur_internal_state + num_action*i, 0, num_action, 1) = state_buffer_.block(num_cur_state*(num_state_skip*(i+1)) + num_cur_internal_state, 0, num_action, 1);
    }

}

void CustomController::feedforwardPolicy()
{
    hidden_layer1_ = policy_net_w0_ * state_ + policy_net_b0_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (hidden_layer1_(i) < 0)
            hidden_layer1_(i) = 0.0;
    }

    hidden_layer2_ = policy_net_w2_ * hidden_layer1_ + policy_net_b2_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (hidden_layer2_(i) < 0)
            hidden_layer2_(i) = 0.0;
    }

    rl_action_ = action_net_w_ * hidden_layer2_ + action_net_b_;

    value_hidden_layer1_ = value_net_w0_ * state_ + value_net_b0_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (value_hidden_layer1_(i) < 0)
            value_hidden_layer1_(i) = 0.0;
    }

    value_hidden_layer2_ = value_net_w2_ * value_hidden_layer1_ + value_net_b2_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (value_hidden_layer2_(i) < 0)
            value_hidden_layer2_(i) = 0.0;
    }

    value_ = (value_net_w_ * value_hidden_layer2_ + value_net_b_)(0);


    /// balance policy
    balance_hidden_layer1_ = balance_policy_net_w0_ * state_ + balance_policy_net_b0_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (balance_hidden_layer1_(i) < 0)
            balance_hidden_layer1_(i) = 0.0;
    }

    balance_hidden_layer2_ = balance_policy_net_w2_ * balance_hidden_layer1_ + balance_policy_net_b2_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (balance_hidden_layer2_(i) < 0)
            balance_hidden_layer2_(i) = 0.0;
    }

    balance_rl_action_ = balance_action_net_w_ * balance_hidden_layer2_ + balance_action_net_b_;

    balance_value_hidden_layer1_ = balance_value_net_w0_ * state_ + balance_value_net_b0_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (balance_value_hidden_layer1_(i) < 0)
            balance_value_hidden_layer1_(i) = 0.0;
    }

    balance_value_hidden_layer2_ = balance_value_net_w2_ * balance_value_hidden_layer1_ + balance_value_net_b2_;
    for (int i = 0; i < num_hidden; i++) 
    {
        if (balance_value_hidden_layer2_(i) < 0)
            balance_value_hidden_layer2_(i) = 0.0;
    }

    balance_value_ = (balance_value_net_w_ * balance_value_hidden_layer2_ + balance_value_net_b_)(0);
    
}

void CustomController::computeSlow()
{
    copyRobotData(rd_);
    if (rd_cc_.tc_.mode == 7)
    {
        if (rd_cc_.tc_init)
        {
            //Initialize settings for Task Control! 
            start_time_ = rd_cc_.control_time_us_;
            q_noise_pre_ = q_noise_ = q_init_ = rd_cc_.q_virtual_.segment(6,MODEL_DOF);
            time_cur_ = start_time_ / 1e6;
            time_pre_ = time_cur_ - 0.005;
            time_inference_pre_ = rd_cc_.control_time_us_ - (1/249.9)*1e6;

            rd_.tc_init = false;
            std::cout<<"cc mode 7"<<std::endl;
            torque_init_ = rd_cc_.torque_desired;

            walking_tick_hk_ = 0;
            ext_force_tick_ = 0;

            processNoise();
            processObservation();
            for (int i = 0; i < num_state_skip*num_state_hist; i++) 
            {
                // state_buffer_.block(num_cur_state*i, 0, num_cur_state, 1) = (state_cur_ - state_mean_).array() / state_var_.cwiseSqrt().array();
                state_buffer_.block(num_cur_state*i, 0, num_cur_state, 1).setZero();
            }
        }

        processNoise();

        // processObservation and feedforwardPolicy mean time: 15 us, max 53 us
        if ((rd_cc_.control_time_us_ - time_inference_pre_)/1.0e6 >= 1/250.0 - 1/10000.0)
        {
            processObservation();
            feedforwardPolicy();
            if(loco_policy_on){
                action_dt_accumulate_ += DyrosMath::minmax_cut(rl_action_(num_action-1)*5/250.0, 0.0, 5/250.0);
            }
            else{
                action_dt_accumulate_ += DyrosMath::minmax_cut(balance_rl_action_(num_action-1)*5/250.0, 0.0, 5/250.0);
            }

            if (value_ < 130.0){
                loco_policy_on = false;
            }
            else{
                loco_policy_on = true;
            }

            if (value_ < 25.0)
            {
                if (stop_by_value_thres_ == false)
                {
                    stop_by_value_thres_ = true;
                    stop_start_time_ = rd_cc_.control_time_us_;
                    q_stop_ = q_noise_;
                    std::cout << "Stop by Value Function" << std::endl;
                }
                // std::cout << "Stop by Value Function" << std::endl;
                std::cout << "Value :" << value_ << std::endl;
            }

            if (is_write_file_)
            {
                    writeFile << (rd_cc_.control_time_us_ - time_inference_pre_)/1e6 << "\t"; // 1
                    writeFile << phase_ << "\t"; // 2
                    writeFile << DyrosMath::minmax_cut(rl_action_(num_action-1)*1/100.0, 0.0, 1/100.0) << "\t"; //phase modulation //3

                    writeFile << rd_cc_.LF_FT.transpose() << "\t";    //4~9   
                    writeFile << rd_cc_.RF_FT.transpose() << "\t";    //10~15 
                    writeFile << rd_cc_.LF_CF_FT.transpose() << "\t"; //16~21
                    writeFile << rd_cc_.RF_CF_FT.transpose() << "\t"; //22~27 

                    writeFile << rd_cc_.torque_desired.transpose()  << "\t"; // 28~60
                    writeFile << q_noise_.transpose() << "\t"; // 61~93
                    writeFile << q_dot_lpf_.transpose() << "\t"; // 94~126
                    // writeFile << rd_cc_.q_dot_virtual_.transpose() << "\t"; //127~165 ?  //127: x_vel
                    writeFile << rd_cc_.q_dot_virtual_.transpose()(0) << "\t"; //127  x_vel
                    writeFile << rd_cc_.q_dot_virtual_.transpose()(1) << "\t"; //128  y_vel
                    writeFile << rd_cc_.q_dot_virtual_.transpose()(2) << "\t"; //129  z_vel
                    // writeFile << rd_cc_.q_virtual_.transpose() << "\t"; //173~205

                    // writeFile << value_ << "\t" << stop_by_value_thres_; //130~131
                    writeFile << value_ << "\t" << stop_by_value_thres_ << "\t" << balance_value_ << "\t"; //130~131
                    writeFile << (float)loco_policy_on;
                
                    writeFile << std::endl;

                    time_write_pre_ = rd_cc_.control_time_us_;
            }
            
            time_inference_pre_ = rd_cc_.control_time_us_;
        }

        for (int i = 0; i < num_actuator_action; i++)
        {
            if(loco_policy_on){
                torque_rl_(i) = DyrosMath::minmax_cut(rl_action_(i)*torque_bound_(i), -torque_bound_(i), torque_bound_(i));
            }
            else{
                torque_rl_(i) = DyrosMath::minmax_cut(balance_rl_action_(i)*torque_bound_(i), -torque_bound_(i), torque_bound_(i));
            }
        }
        for (int i = num_actuator_action; i < MODEL_DOF; i++)
        {
            torque_rl_(i) = kp_(i,i) * (q_init_(i) - q_noise_(i)) - kv_(i,i)*q_vel_noise_(i);
        }
        
        if (rd_cc_.control_time_us_ < start_time_ + 0.1e6)
        {
            for (int i = 0; i <MODEL_DOF; i++)
            {
                torque_spline_(i) = DyrosMath::cubic(rd_cc_.control_time_us_, start_time_, start_time_ + 0.1e6, torque_init_(i), torque_rl_(i), 0.0, 0.0);
            }
            rd_.torque_desired = torque_spline_;
        }
        else
        {
            rd_.torque_desired = torque_rl_;
            // rd_.torque_desired(0) = 0.0;
            // rd_.torque_desired(1) = 0.0;
            // rd_.torque_desired(2) = 0.0;
            // rd_.torque_desired(3) = 0.0;
            // rd_.torque_desired(4) = 0.0;
            // rd_.torque_desired(5) = 0.0;
            // for (int _joint : {3}) {
            //     rd_.torque_desired(_joint) = kp_(_joint,_joint) * (q_init_(_joint) - q_noise_(_joint)) - kv_(_joint,_joint)*q_vel_noise_(_joint);
            // }
            // for (int _joint : {3}) {
            //     rd_.torque_desired(_joint) = 10000 * (q_init_(_joint) - q_noise_(_joint)) - 50*q_vel_noise_(_joint);
            // }
        }

        if (stop_by_value_thres_)
        {
            rd_.torque_desired = kp_ * (q_stop_ - q_noise_) - kv_*q_vel_noise_;
        }

        //// Loco Policy
        // ext_force_apply_time_ = 1.0*hz_; //[s]
        // force_temp_ = 70.0;
        // ext_force_apply_time_ = 0.5*hz_; //[s]
        // force_temp_ = 140.0;
        // ext_force_apply_time_ = 0.3333*hz_; //[s]
        // force_temp_ = 210.0;
        // ext_force_apply_time_ = 0.2*hz_; //[s]
        // force_temp_ = 350.0;

        // Balance Policy
        ext_force_apply_time_ = 1.0*hz_; //[s]
        force_temp_ = 120.0;
        theta_temp_ = 270.0;

        if(walking_tick_hk_ == 50000){
            ext_force_flag = true;
            // ext_force_tick_ = 0;
        }

        if(ext_force_tick_ < ext_force_apply_time_ && ext_force_flag){
                mujoco_applied_ext_force_.wrench.force.x = force_temp_*sin(theta_temp_*DEG2RAD); //x-axis linear force
                mujoco_applied_ext_force_.wrench.force.y = -force_temp_*cos(theta_temp_*DEG2RAD); //y-axis linear force  
                mujoco_applied_ext_force_.wrench.force.z = 0.0; //z-axis linear force
                mujoco_applied_ext_force_.wrench.torque.x = 0.0; //x-axis angular moment
                mujoco_applied_ext_force_.wrench.torque.x = 0.0; //y-axis angular moment
                mujoco_applied_ext_force_.wrench.torque.x = 0.0; //z-axis angular moment

                mujoco_applied_ext_force_.link_idx = 1; //link idx; 1:pelvis

                mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);
                ext_force_tick_++;
        }
        else{
            ext_force_tick_ = 0;
            ext_force_flag = false;
            // ext_force_flag_X_ = false;
            // ext_force_flag_Y_ = false;
            // ext_force_flag_A_ = false;
            // ext_force_flag_B_ = false; 
            mujoco_applied_ext_force_.wrench.force.x = 0; //x-axis linear force
            mujoco_applied_ext_force_.wrench.force.y = 0; //y-axis linear force  
            mujoco_applied_ext_force_.wrench.force.z = 0.0; //z-axis linear force
            mujoco_applied_ext_force_.wrench.torque.x = 0.0; //x-axis angular moment
            mujoco_applied_ext_force_.wrench.torque.x = 0.0; //y-axis angular moment
            mujoco_applied_ext_force_.wrench.torque.x = 0.0; //z-axis angular moment

            mujoco_applied_ext_force_.link_idx = 1; //link idx; 1:pelvis

            mujoco_ext_force_apply_pub.publish(mujoco_applied_ext_force_);
        }

        if(walking_tick_hk_ % 500 == 0)
        {
            std::cout << "walking_tick_hk_ : " << walking_tick_hk_ << std::endl;
            std::cout << "phase : " << phase_ << std::endl;
            std::cout << "target_vel_x_ : " << target_vel_x_ << std::endl;
            std::cout << "target_vel_y_ : " << target_vel_y_ << std::endl;
            std::cout << "ext_force_flag :" << ext_force_flag << std::endl;
            std::cout << "ext_force_tick_ : " << ext_force_tick_ << std::endl;
            std::cout << "value_ : " << value_ << std::endl;
            std::cout << "balance_value_ : " << balance_value_ << std::endl;
        }

        walking_tick_hk_++;
    }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    target_vel_x_ = DyrosMath::minmax_cut(0.5*joy->axes[1], -0.2, 0.5);
    target_vel_y_ = DyrosMath::minmax_cut(0.5*joy->axes[0], -0.2, 0.2);
}