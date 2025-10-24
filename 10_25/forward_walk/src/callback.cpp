#include "callback.hpp"

bool flgflg = 0;
FILE *Trajectory_all;

Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr)
    : Node("callback_node_call"),  // Node 생성 시 노드 이름을 추가
      trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      pick_Ptr(pick_Ptr),
      SPIN_RATE(100)     
{
    // ROS 2에서의 Node 객체 생성
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("callback_node");
    
    // ROS 2에서 boost::thread 대신 std::thread 사용
    std::thread queue_thread(&Callback::callbackThread, this);
    queue_thread.detach();  // 비동기식 실행



    // set_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/SET", 10, std::bind(&Callback::SetMode, this, std::placeholders::_1));

    // ROS 2의 subscription 생성
    start_subscriber_= this->create_subscription<std_msgs::msg::Bool>("/START", 10, std::bind(&Callback::StartMode, this, std::placeholders::_1));

    balance_sub_ = this->create_subscription<robot_msgs::msg::BalanceOffsets>("/gyro/balance_offsets", 10, std::bind(&Callback::BalanceCallback, this, std::placeholders::_1));
    
    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
    omega_w = sqrt(g / z_c);

    pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 675);
    pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 675);
    pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 675);
    pick_Ptr->Ref_RL_th = MatrixXd::Zero(5, 675);
    pick_Ptr->Ref_LL_th = MatrixXd::Zero(5, 675);

    indext = 1;
    set_mode = Motion_Index::Start_pose;



    RCLCPP_INFO(this->get_logger(), "Callback activated");
}


void Callback::BalanceCallback(const robot_msgs::msg::BalanceOffsets::SharedPtr msg)
{
    m8_ = msg->m8;
    m9_ = msg->m9;
    m4_ = msg->m4;
    m5_ = msg->m5;
    m0_ = msg->m0;
    m1_ = msg->m1;
    m2_ = msg->m2;
    m3_ = msg->m3;

    RCLCPP_DEBUG(this->get_logger(),
        "Received BalanceOffsets: m8=%.2f, m9=%.2f, m4=%.2f, m5=%.2f, m0=%.2f, m1=%.2f, m2=%.2f, m3=%.2f",
        m8_, m9_, m4_, m5_, m0_, m1_, m2_, m3_);
}


void Callback::Set()
{
    All_Theta[0] = 0.0;
    All_Theta[1] = -0.050419;
    All_Theta[2] = -0.785155;
    All_Theta[3] = -0.327585;
    All_Theta[4] = 0.959987;
    All_Theta[5] = -0.032966;
    All_Theta[6] = 0.0;
    All_Theta[7] = 0.036848;
    All_Theta[8] = 0.785155;
    All_Theta[9] = 0.327585;
    All_Theta[10] = -0.907627;
    All_Theta[11] = -0.032966;

    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD
}




// ros2 topic pub /START std_msgs/msg/Bool "data: true" -1

void Callback::StartMode(const std_msgs::msg::Bool::SharedPtr start)
{
    RCLCPP_DEBUG(this->get_logger(), "StartMode called with data: %d", start->data);
    if (start->data)
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);


        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);


        re = 0;


        set_mode = Motion_Index::Forward_4step_L_turn;
        // set_mode = 11;

        RCLCPP_INFO(this->get_logger(), "StartMode activated with true data!");
    }
}

void Callback::callbackThread()
{
    // ROS 2의 spin 사용 대신 루프에서 메시지 처리
    rclcpp::Rate loop_rate(SPIN_RATE);
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

void Callback::TATA()
{
    double res_turn_angle = angle;

    if (res_turn_angle != 0)
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        // index_angle = 0;
    }
    // RCLCPP_WARN(this->get_logger(), "TURN_ANGLE : %.2f deg", res_turn_angle);
    // RCLCPP_INFO(this->get_logger(), "------------------------- TURN_ANGLE ----------------------------");
}

void Callback::SelectMotion()
{
    if(re == 0)
    {
        if(set_mode == Motion_Index::Start_pose)//Start_pose
        {
            startRL_st[0] = 0;
            startRL_st[1] = 0.001941;
            startRL_st[2] = 0.122416;
            startRL_st[3] = 0.196013;
            startRL_st[4] = -0.073595;//1.148133;
            startRL_st[5] = 0.001941;

            startLL_st[0] = 0;
            startLL_st[1] = 0.001941;
            startLL_st[2] = -0.122416;
            startLL_st[3] = -0.196013;
            startLL_st[4] = 0.073595;//-1.148133;
            startLL_st[5] = 0.001941;

            // startRL_st[6] = { 0.0, 0.001941, 0.122416, 0.196013, 1.148133, 0.001941 };
            // startLL_st[6] = { 0.0, 0.001941, -0.122416, -0.196013, -1.148133, 0.001941 };
        }

        else if(set_mode == Motion_Index::Forward_test)//Forward_test
        {
            // re = 1;
            // indext = 0;
            // angle = 8;
            // // turn_st = 1; //1 -> turn_left, 2 -> turn_right

            // trajectoryPtr->Change_Freq(2);
            // // mode = Motion_Index::Forward_20step;
            // IK_Ptr->Change_Com_Height(30);
            // trajectoryPtr->Go_Straight_start(0.05, 0.25, 0.05);
            // IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            // IK_Ptr->Set_Angle_Compensation(135);
            // trajectoryPtr->Stop_Trajectory_straightwalk(0.05);


            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 1.5, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }
        
        else if(set_mode == Motion_Index::Fast_6step)//Fast_6step
        {
            re = 1;
            indext = 0;
            angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.4, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(3, 3, -2, 6, 3, 2, 2, -6);   
            IK_Ptr->Set_Angle_Compensation(67);
        }

        else if(set_mode == Motion_Index::Step_in_place)//Step_in_place
        {
            re = 1;
            indext = 0;
            angle = 8;
            turn_st = 1; //1 -> turn_left, 2 -> turn_right


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Step_in_place(0.05, 0.25, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Forward_Halfstep)//Forward_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Forward_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight(0.01, 0.03, 0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Back_Halfstep)//Back_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.1, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Back_2step)//Back_2step
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Back_2step;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Back_Straight(-0.04, -0.22, 0.05);
            IK_Ptr->Change_Angle_Compensation(3, 4, 2, 0, 2, 2, 2, -4); 
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Set_Angle_Compensation(135);
        }
        else if(set_mode == Motion_Index::Huddle)//Huddle
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Huddle;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Huddle_Motion(0.22, 0.14, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(6.5, 3.5, 0, 3.5, 6.5, 3.5, 0, -3.5);
            IK_Ptr->Change_Angle_Compensation(6.5, 2.6, 0.0, 3.5, 6.5, 2.6, 0.0, -3.5);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Right_Halfstep)//Right_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Right_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Right1(0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 7, 4, -8, 2, 4, -4, 4);
            // IK_Ptr->Change_Angle_Compensation(2, 7, 1, 1, 2, 4, 1, -1);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Left_Halfstep)//Left_Halfstep
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Left_Halfstep;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Side_Left1(0.025);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            // IK_Ptr->Change_Angle_Compensation(2, 4, -4, 0, 2, 7, 4, -8);
            IK_Ptr->Change_Angle_Compensation(2, 4, -4, 0, 2, 7, 4, -8);
            IK_Ptr->Set_Angle_Compensation(135);
        }

        else if(set_mode == Motion_Index::Fast_4step)//Fast_4step
        {
            re = 1;
            indext = 0;
            angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);   
            IK_Ptr->Set_Angle_Compensation(67);
        }

        else if(set_mode == Motion_Index::Fast_1step)//Fast_1step
        {
            re = 1;
            indext = 0;
            angle = 1.5;


            trajectoryPtr->Change_Freq(2);
            // mode = Motion_Index::Step_in_place;
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 0.15, 0.05, 1);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2);   
            IK_Ptr->Set_Angle_Compensation(67);
        }


        else if (set_mode == Motion_Index::Picking_Ball)
        {
            re = 1;
            indext = 0;
            // mode = Motion_Index::Picking_Ball;
            
            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr -> Picking_Motion(300, 150, 0.165);

        }

        else if (set_mode == Motion_Index::Re_grapple)
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 400);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(-60,0,0,-30,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());
        }
            

        else if (set_mode == Motion_Index::Ready_to_throw)
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);

            pick_Ptr->WT_Trajectory(-10,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(-180,-22,50,-50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

        }

        else if (set_mode == Motion_Index::Shoot)
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(10);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);
        }

        else if (set_mode == Motion_Index::Grapple_FINISH)
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 400);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 400);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 400);

            pick_Ptr->WT_Trajectory(0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());  
            pick_Ptr->LA_Trajectory(60,0,0,30,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

            RCLCPP_INFO(this->get_logger(), "FINISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

        }
        
        else if (set_mode == Motion_Index::Shoot_FINISH)
        {
            re = 1;
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 500);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 500);
            // trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 500);
            // trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 500);
            trajectoryPtr->Ref_RL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);
            trajectoryPtr->Ref_LL_z = trajectoryPtr->zsimulation_standup_Shoot_FINISH(500, -0.02);

            pick_Ptr->WT_Trajectory(30,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->RA_Trajectory(0,0,0,0,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->LA_Trajectory(150,22,-110,50,trajectoryPtr->Ref_RL_x.cols());
            pick_Ptr->NC_Trajectory(0,0,trajectoryPtr->Ref_RL_x.cols());

        }
        
        else if (set_mode == Motion_Index::Stand_Up)
        {
            re = 1;            
            indext = 0;


            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 2800);
            trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 2800);
            trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 2800);
            trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 2800);
            trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 2800);
            trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 2800);
            // trajectoryPtr -> Stand_Up_Motion(300, 1000, 0.165);


        }

        else if(set_mode == Motion_Index::Forward_1step)//Forward_1step
        {
            re = 1;
            indext = 0;
            angle = 8;
            // turn_st = 1; //1 -> turn_left, 2 -> turn_right

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.15, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 2, 0, -2, 2, 2, 0, -2); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }


        else if(set_mode == Motion_Index::Forward_40step_speed_Up)//40step_speed_Up
        {
            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Freq_Change_Straight(0.05, 2.5, 0.05, 1.5);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3);
            IK_Ptr->Set_Angle_Compensation(101);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }
        
        else if(set_mode == Motion_Index::Forward_40step)//40step
        {

            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 2.5, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }
        
        else if(set_mode == Motion_Index::Forward_15step)//15step
        {

            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 1.5, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }        

        else if(set_mode == Motion_Index::Forward_2step)//2step
        {

            re = 1;
            indext = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(set_mode == Motion_Index::Forward_4step_L_turn)//Forward_4step_L_turn
        {

            re = 1;
            indext = 0;
            angle = 8;
            // index_angle = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

        else if(set_mode == Motion_Index::Forward_4step_R_turn)//Forward_4step_R_turn
        {

            re = 1;
            indext = 0;
            angle = 8;
            // index_angle = 0;

            trajectoryPtr->Change_Freq(2);
            IK_Ptr->Change_Com_Height(30);
            trajectoryPtr->Go_Straight_start(0.05, 0.2, 0.05);
            IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
            IK_Ptr->Change_Angle_Compensation(2, 3, 0, -1, 2, 1, 0, -3); 
            IK_Ptr->Set_Angle_Compensation(135);
            trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        }

    }

    
}

void Callback::Write_All_Theta()
{

    if(re == 1)
    {
        indext += 1;

        if (set_mode == Motion_Index::Forward_4step_L_turn)//Forward_4step_L_turn
        {
                
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

            // std::cout << "indext" << indext << std::endl;
            if(indext > 135 && indext <= 270)
            {

                IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                step = (IK_Ptr->LL_th[0])/2;
                index_angle = index_angle + 1;
                // std::cout << "index_angle" << index_angle << std::endl;
                if (index_angle > walktime_n - 1)
                {
                    index_angle = 0;
                }
            }

        }

        else if (set_mode == Motion_Index::Forward_4step_R_turn)//Forward_4step_R_turn
        {
                
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());

            // std::cout << "indext" << indext << std::endl;
            if(indext>=67 && indext <=337)
            {
                IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                step = (IK_Ptr->RL_th[0])/2;
                index_angle += 1;
                // std::cout << "index_angle" << index_angle << std::endl;
                if (index_angle > walktime_n - 1)
                {
                    index_angle = 0;
                }
            }
        }

        else if (set_mode == Motion_Index::Forward_test || set_mode == Motion_Index::Step_in_place || set_mode == Motion_Index::Forward_1step || set_mode == Motion_Index::Forward_40step || set_mode == Motion_Index::Forward_15step || set_mode == Motion_Index::Forward_2step)
        {                
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            // std::cout << "indext" << indext << std::endl;
            if(indext > 135 && indext <= 270 && turn_st == 1)
            {
                IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                step = (IK_Ptr->LL_th[0])/2;
                index_angle = index_angle + 1;
                std::cout << "index_angle" << index_angle << std::endl;
                if (index_angle > walktime_n - 1)
                {
                    index_angle = 0; 
                }
            }

            if(indext>=67 && indext <=337 && turn_st ==2)
            {
                IK_Ptr->RL_th[0] = -(trajectoryPtr->Turn_Trajectory(index_angle));
                step = (IK_Ptr->RL_th[0])/2;
                index_angle += 1;
                std::cout << "index_angle" << index_angle << std::endl;
                if (index_angle > walktime_n - 1)
                {
                    index_angle = 0;
                }
            }
        }

        else if (set_mode == Motion_Index::Fast_6step || set_mode == Motion_Index::Fast_4step || set_mode == Motion_Index::Fast_1step) 
        {
            // turnRL_st= 2 * DEG2RAD;
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
        }

        else if (set_mode == Motion_Index::Forward_40step_speed_Up) 
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Forward_40step_Angle_Compensation(indext);
        }
            
        else if (set_mode == Motion_Index::Forward_Halfstep)//Forward_Halfstep
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
        }

        else if (set_mode == Motion_Index::Back_Halfstep || set_mode == Motion_Index::Back_2step)//Back_Halfstep //Back_2step
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
        }

        else if(set_mode == Motion_Index::Huddle)//Huddle 1
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Huddle(indext);
        }

        else if (set_mode == Motion_Index::Left_Halfstep)//Left_Halfstep
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Leftwalk(indext);
        }

        else if (set_mode == Motion_Index::Right_Halfstep)//Right_Halfstep
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Rightwalk(indext);
        }

        else if (set_mode == Motion_Index::Picking_Ball)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            
            pick_Ptr->Picking(trajectoryPtr->Ref_RL_x, indext, RL_th2, LL_th2);
        }

        else if (set_mode == Motion_Index::Re_grapple || set_mode == Motion_Index::Ready_to_throw || set_mode == Motion_Index::Grapple_FINISH || set_mode == Motion_Index::Shoot_FINISH)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);

        }
        
        else if (set_mode == Motion_Index::Shoot)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->WT_th[0] =  -30 * DEG2RAD;
	        pick_Ptr->LA_th[0] = -150 * DEG2RAD;
            pick_Ptr->LA_th[1] = -22 * DEG2RAD;
            pick_Ptr->LA_th[2] = 110 * DEG2RAD;
            pick_Ptr->LA_th[3]  = -50 * DEG2RAD;
        }

        else if (set_mode == Motion_Index::Stand_Up)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            
            pick_Ptr->Stand_up(trajectoryPtr->Ref_RL_x, indext);
        }
        
    }


    if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
    {

        indext = 0;
        re = 2;
        turn_st = 0;

        turnRL_st=0;
        turnLL_st=0;
        
        m8_ = 0.0, m9_ = 0.0, m4_ = 0.0, m5_ = 0.0;
        m0_ = 0.0, m1_ = 0.0, m2_ = 0.0, m3_ = 0.0;

        for (int i = 0; i < 6; i++) 
        {
            pick_Ptr->RL_th_ALL[i] = 0.0;
            pick_Ptr->LL_th_ALL[i] = 0.0;


        }


        // set_mode = Motion_Index::Step_in_place;

        // std::this_thread::sleep_for(std::chrono::seconds(2));
        

        // if(set_mode == Motion_Index::Picking_Ball)
        // {
        //     set_mode = Motion_Index::Re_grapple;
        // }
        if(set_mode == Motion_Index::Re_grapple)
        {
            set_mode = Motion_Index::Grapple_FINISH;
        }
        else if(set_mode == Motion_Index::Grapple_FINISH)
        {
            set_mode = Motion_Index::Ready_to_throw;
        }
        else if(set_mode == Motion_Index::Ready_to_throw)
        {
            set_mode = Motion_Index::Shoot;
        }
        else if(set_mode == Motion_Index::Shoot)
        {
            set_mode = Motion_Index::Shoot_FINISH;
        }
        else if(set_mode == Motion_Index::Shoot_FINISH)
        {
            re = 2;
        }


    }

// {10, 8, 6, 4, 2, 0,              11, 9, 7, 5, 3, 1,             12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};

    // All_Theta 계산 및 저장
    All_Theta[0] = -IK_Ptr->RL_th[0] + pick_Ptr->RL_th_ALL[0] + startRL_st[0] + turnRL_st;
    All_Theta[1] = IK_Ptr->RL_th[1] + pick_Ptr->RL_th_ALL[1] +startRL_st[1] -RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] + pick_Ptr->RL_th_ALL[2] +startRL_st[2] -RL_th2 * DEG2RAD - 17 * DEG2RAD; //10.74 right
    All_Theta[3] = -IK_Ptr->RL_th[3] + pick_Ptr->RL_th_ALL[3] +startRL_st[3] + 40 * DEG2RAD; //38.34 * DEG2RAD;   
    All_Theta[4] = -IK_Ptr->RL_th[4] + pick_Ptr->RL_th_ALL[4] +startRL_st[4] + 24.22 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + pick_Ptr->RL_th_ALL[5] +startRL_st[5] - 2* DEG2RAD;

    All_Theta[6] = -IK_Ptr->LL_th[0] + pick_Ptr->LL_th_ALL[0] +startLL_st[0] + turnLL_st;
    All_Theta[7] = IK_Ptr->LL_th[1] + pick_Ptr->LL_th_ALL[1] +startLL_st[1] +LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + pick_Ptr->LL_th_ALL[2] +startLL_st[2] +LL_th2 * DEG2RAD + 17 * DEG2RAD; //left
    All_Theta[9] = IK_Ptr->LL_th[3] + pick_Ptr->LL_th_ALL[3] +startLL_st[3] - 40 * DEG2RAD; //40.34 * DEG2RAD;  
    All_Theta[10] = IK_Ptr->LL_th[4] + pick_Ptr->LL_th_ALL[4] +startLL_st[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + pick_Ptr->LL_th_ALL[5] +startLL_st[5] - 2 * DEG2RAD;


    // upper_body
    All_Theta[12] = pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD



    All_Theta[5] += m0_;
    All_Theta[11] += m1_;
    All_Theta[4] += m2_;
    All_Theta[10] += m3_;
    All_Theta[3] += m4_;
    All_Theta[9] += m5_;
    All_Theta[1] += m8_;
    All_Theta[7] += m9_;




    if(set_mode == Motion_Index::Start_pose)
    {
        for (int i = 0; i < 6; i++) 
        {
            startRL_st[i] = 0.0;
            startLL_st[i] = 0.0;



        }
    }

    // 디버깅 정보 출력
    // for (int i = 0; i < All_Theta.size(); ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]);
    // }
}