// #ifndef CALLBACK_H
// #define CALLBACK_H


// #include <eigen3/Eigen/Dense>
// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include "std_msgs/msg/bool.hpp"
// #include "robot_msgs/msg/balance_offsets.hpp"
// // #include <tf2/LinearMath/Quaternion.h>
// #include <boost/thread.hpp>


// #include "dynamixel.hpp"
// #include "BRP_Kinematics.hpp"
// #include "NewPattern2.hpp"


// using Eigen::VectorXd; 



// class Callback : public rclcpp::Node
// {
// private:
//     // double turn_angle = 0;     
//     // int arm_indext = 0;        
//     double z_c = 1.2 * 0.28224; 
//     double g = 9.81;           
//     double omega_w;            
//     double _dt = 0.01;         

//     Trajectory *trajectoryPtr;    // Trajectory 객체를 가리키는 포인터
//     IK_Function *IK_Ptr;          // IK_Function 객체를 가리키는 포인터
//     Dxl *dxlPtr;                  // Dxl 객체를 가리키는 포인터
//     Pick *pick_Ptr;               // Pick 객체를 가리키는 포인터
//     double Goal_joint_[NUMBER_OF_DYNAMIXELS];

// public:

//   enum Motion_Index
//   {
//     Start_pose = 0,
//     Forward_2step = 1,
//     Fast_6step = 2,
//     Step_in_place = 3,
//     Forward_Halfstep = 4,
//     Back_Halfstep = 5,
//     Back_2step = 6,
//     Huddle = 7,
//     Right_Halfstep = 8,
//     Left_Halfstep = 9,
//     Fast_4step = 10,
//     Fast_1step = 11,






//     Picking_Ball = 50,
//     Re_grapple = 51,
//     Ready_to_throw = 52,
//     Shoot = 53,
//     Grapple_FINISH = 54,
//     Shoot_FINISH = 55


//   };


//     Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr);
    
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscriber_;

//     virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);


//     double startRL_st[6] = { 0.0 };
//     double startLL_st[6] = { 0.0 };
//     double angle = 0;


//     virtual void SelectMotion();            
//     virtual void Write_All_Theta();           
//     void callbackThread();
//     void Set();
//     void TATA();

//     const int SPIN_RATE = 100;
    
//     int set_mode = 0;              
//     // int go = 0;
//     int re = 0;
//     // int emergency = 99;
//     int indext = 0;
//     // int mode = 0;
//     int index_angle = 0;
//     int turn_st = 0;                       

//     double step = 0;
//     double RL_th2 = 0, LL_th2 = 0;
//     double RL_th1 = 0, LL_th1 = 0;
//     double HS = 0;  
//     double SR = 0;
//     double turn_angle = 0;
//     double turnRL_st=0;
//     double turnLL_st=0;  

//     VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);





//     double walkfreq = 1.48114;
//     double walktime = 2 / walkfreq;
//     int freq = 100;
//     int walktime_n = walktime * freq;





















//     // double rl_neckangle = 0;                
//     // double ud_neckangle = 0;                
//     // double tmp_turn_angle = 0;              
//     // bool emergency_ = 1;                    
//     // double vel_x = 0;
//     // double vel_y = 0;
//     // double vel_z = 0;
//     // int error_counter = 0;
//     // bool error_printed = false;

//     // int8_t mode = 99;                       
//     // double walkfreq = 1.48114;             
//     // double walktime = 2 / walkfreq;        
//     // int freq = 100;                        
//     // int walktime_n = walktime * freq;      


//     // int upper_indext = 0;                 
//     // int check_indext = 0;                 
//     // int stop_indext = 0;  

//     // bool turn_left = false;
//     // bool turn_right = false;

//     // bool on_emergency = false;

//     // double angle = 0;
//     // int index_angle = 0;

//     // double Real_CP_Y = 0;
//     // double Real_CP_X = 0;
//     // double xZMP_from_CP = 0;
//     // double yZMP_from_CP = 0;
//     // double Real_zmp_y_accel = 0;

//     // MatrixXd RL_motion, LL_motion;
//     // MatrixXd RL_motion0, LL_motion0;
//     // MatrixXd RL_motion1, LL_motion1;
//     // MatrixXd RL_motion2, LL_motion2;
//     // MatrixXd RL_motion3, LL_motion3;
//     // MatrixXd RL_motion4, LL_motion4;
//     // MatrixXd RL_motion5, LL_motion5;
//     // MatrixXd RL_motion6, LL_motion6;
//     // MatrixXd RL_motion7, LL_motion7;

// };

// #endif 






#ifndef CALLBACK_H
#define CALLBACK_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "robot_msgs/msg/balance_offsets.hpp"
#include <boost/thread.hpp>

#include "dynamixel.hpp"
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"

using Eigen::VectorXd; 

class Callback : public rclcpp::Node
{
private:
    double z_c = 1.2 * 0.28224; 
    double g = 9.81;           
    double omega_w;            
    double _dt = 0.01;         

    Trajectory *trajectoryPtr;    // Trajectory 객체 포인터
    IK_Function *IK_Ptr;          // IK_Function 객체 포인터
    Dxl *dxlPtr;                  // Dxl 객체 포인터
    Pick *pick_Ptr;               // Pick 객체 포인터
    double Goal_joint_[NUMBER_OF_DYNAMIXELS];

    // Gyro 보정값 관련
    rclcpp::Subscription<robot_msgs::msg::BalanceOffsets>::SharedPtr balance_sub_;
    double m8_ = 0.0, m9_ = 0.0, m4_ = 0.0, m5_ = 0.0;
    double m0_ = 0.0, m1_ = 0.0, m2_ = 0.0, m3_ = 0.0;


public:

  enum Motion_Index
  {
    Start_pose = 0,
    Forward_test = 1,
    Fast_6step = 2,
    Step_in_place = 3,
    Forward_Halfstep = 4,
    Back_Halfstep = 5,
    Back_2step = 6,
    Huddle = 7,
    Right_Halfstep = 8,
    Left_Halfstep = 9,
    Fast_4step = 10,
    Fast_1step = 11,
    Forward_1step = 12,
    Forward_40step = 13,
    Forward_40step_speed_Up = 14,
    Forward_15step = 15,
    Forward_2step = 16,
    Forward_4step_L_turn =17,
    Forward_4step_R_turn = 18,

    Picking_Ball = 50,
    Re_grapple = 51,
    Ready_to_throw = 52,
    Shoot = 53,
    Grapple_FINISH = 54,
    Shoot_FINISH = 55,
    Stand_Up = 56
  };

    Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *pick_Ptr);
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_subscriber_;

    virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);

    // Gyro 보정 콜백
    void BalanceCallback(const robot_msgs::msg::BalanceOffsets::SharedPtr msg);

    double startRL_st[6] = { 0.0 };
    double startLL_st[6] = { 0.0 };
    double angle = 0;

    virtual void SelectMotion();            
    virtual void Write_All_Theta();           
    void callbackThread();
    void Set();
    void TATA();

    const int SPIN_RATE = 100;
    
    int set_mode = 0;              
    int re = 0;
    int indext = 0;
    int index_angle = 0;
    int turn_st = 0;                       

    double step = 0;
    double RL_th2 = 0, LL_th2 = 0;
    double RL_th1 = 0, LL_th1 = 0;
    double HS = 0;  
    double SR = 0;
    double turn_angle = 0;
    double turnRL_st=0;
    double turnLL_st=0;  

    VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

    double walkfreq = 1.48114;
    double walktime = 2 / walkfreq;
    int freq = 100;
    int walktime_n = walktime * freq;
};

#endif 
