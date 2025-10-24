#include "dynamixel.hpp"
#include <iostream>  

Dxl::Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
        std::cerr << "[Error] Failed to open the port!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to open the port!" << std::endl;

    if (!portHandler->setBaudRate(BAUDRATE))
        std::cerr << "[Error] Failed to set the baudrate!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to set the baudrate!" << std::endl;

    int16_t current_mode = SetPresentMode(Mode);

    if (current_mode == Current_Control_Mode)
    {
        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Current_Control_Mode, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                std::cerr << "[Error] Failed to set current control mode for ID: " << int(dxl_id[i]) << std::endl;
            else
                std::cout << "[Info] Set current control mode for ID: " << int(dxl_id[i]) << std::endl;
        }
    }
    else if (current_mode == Position_Control_Mode)
    {
        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Position_Control_Mode, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                std::cerr << "[Error] Failed to set position control mode for ID: " << int(dxl_id[i]) << std::endl;
            else
                std::cout << "[Info] Set position control mode for ID: " << int(dxl_id[i]) << std::endl;
        }
    }
    else
    {
        std::cerr << "[Error] Invalid mode set." << std::endl;
    }




    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable torque for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] Torque enabled for ID: " << int(dxl_id[i]) << std::endl;
    }


    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable LED for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] LED enabled for ID: " << int(dxl_id[i]) << std::endl;
    }




    // VectorXd theta_zero = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    // MoveToTargetSmoothCos(theta_zero, 150, 10);

    // VectorXd theta_goal = Eigen::Map<VectorXd>(Set_D, NUMBER_OF_DYNAMIXELS);
    // MoveToTargetSmoothCos(theta_goal, 150, 10);
    // std::cout << "[Info] Start is half!!!!!!" << std::endl;


    VectorXd PID_Gain(3);
    PID_Gain << 850, 0, 0;
    SetPIDGain(PID_Gain);
}

Dxl::~Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable torque for ID: " << int(i) << std::endl;
        else
            std::cout << "[Info] Torque disabled for ID: " << int(dxl_id[i]) << std::endl;
    }

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable LED for ID: " << int(i) << std::endl;
        else
            std::cout << "[Info] LED disabled for ID: " << int(dxl_id[i]) << std::endl;
    }

    portHandler->closePort();
}







// ************************************ GETTERS ***************************************** //

//Getter() : 각도 읽기(raw->rad)
void Dxl::syncReadTheta()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
}




// void Dxl::syncReadTheta()
// {
//     dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);

//     for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
//         groupSyncRead.addParam(dxl_id[i]);

//     int comm_result = groupSyncRead.txRxPacket();
//     if (comm_result != COMM_SUCCESS)
//     {
//         std::cerr << "[ERROR] txRxPacket failed: " << comm_result << std::endl;
//         return;
//     }

//     for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
//     {
//         if (!groupSyncRead.isAvailable(dxl_id[i], DxlReg_PresentPosition, 4))
//         {
//             std::cerr << "[WARN] ID " << static_cast<int>(dxl_id[i]) << " data not available!" << std::endl;
//             continue;
//         }

//         position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
//     }

//     groupSyncRead.clearParam();

//     for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++)
//     {
//         th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
//     }
// }










void Dxl::Printangle()
{
    // 현재 각도 갱신
    syncReadTheta();

    // 출력
    // std::cout << "[Dxl] 현재 각도 (rad):" << std::endl;
    // for (int i = 0; i < NUMBER_OF_DYNAMIXELS; ++i)
    // {
    //     std::cout << "Joint[" << i << "] : " << th_[i] << " rad" << std::endl;
    // }

    std::cout << "[Dxl] 현재 각도 (deg):" << std::endl;
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; ++i)
    {
        double deg = th_[i] * 180.0 / M_PI;
        std::cout << "Joint[" << i << "] : " << deg << " deg" << std::endl;
    }
}



//Getter() : 각도 getter() [rad]
VectorXd Dxl::GetThetaAct()
{
    syncReadTheta();
    return th_;
}

//Getter() : velocity 읽기 (raw data)
void Dxl::syncReadThetaDot()
{
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) groupSyncReadThDot.addParam(dxl_id[i]);
    groupSyncReadThDot.txRxPacket();
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    groupSyncReadThDot.clearParam();
}

//Getter() : 각속도 getter() [rad/s] 
//0.0239868240
VectorXd Dxl::GetThetaDot()
{
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; //4,294,967,295 = 0xFFFFFFFF   // 1 = 0.229rpm   // 1 = 0.003816667
        else vel_[i] = velocity[i] * 0.003816667;
    }
    return vel_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간) [rad/s]
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
    th_last_ = th_;
}

//Getter() : 각속도 추정계산 getter() [rad/s] 
VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}


//Getter() : PID gain getter()
// VectorXd Dxl:: GetPIDGain()
// {

// }

//Getter() : 전류값 [mA] 
void Dxl::SyncReadCurrent()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentCurrent, 2);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) current[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentCurrent, 2);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) cur_[i] = convertValue2Current(current[i]);
}

VectorXd Dxl::GetCurrent()
{
    SyncReadCurrent();
    return cur_;
}


//Getter() : 현재 모드 getter()
int16_t Dxl::GetPresentMode()
{
    return this->Mode;
}


// **************************** SETTERS ******************************** //

//setter() : 각도 setter() [rad]
void Dxl::syncWriteTheta()
{
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);

  uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0};

  for (uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++){
    ref_th_value_ = ref_th_ * RAD_TO_VALUE;
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dxl_id[i], (uint8_t *)&parameter);
  }
  gSyncWriteTh.txPacket();
  gSyncWriteTh.clearParam();
}




//Setter() : 목표 세타값 설정 [rad]
void Dxl::SetThetaRef(VectorXd theta)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++) 
    {
        ref_th_[i] = theta[i]+PI;
        // std::cout << ref_th_[i] << std::endl;
    }
}

//setter() : 토크 setter() [Nm]
void Dxl::syncWriteTorque()
{
    dynamixel::GroupSyncWrite groupSyncWriter(portHandler, packetHandler, DxlReg_GoalCurrent, 2);
    uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0};
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        ref_torque_value[i] = torqueToValue(ref_torque_[i], i);
        if(ref_torque_value[i] > 1000) ref_torque_value[i] = 1000; //상한값
        else if(ref_torque_value[i] < -1000) ref_torque_value[i] = -1000; //하한값
    }
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        getParam(ref_torque_value[i], parameter);
        groupSyncWriter.addParam(dxl_id[i], (uint8_t *)&parameter);
    }
    groupSyncWriter.txPacket();
    groupSyncWriter.clearParam();
}

//Setter() : 목표 토크 설정 [Nm]
void Dxl::SetTorqueRef(VectorXd a_torque)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) 
    {
        ref_torque_[i] = a_torque[i];
    }
}

// Setter() : PID gain setter()
void Dxl::SetPIDGain(VectorXd PID_Gain)
{    
    uint8_t dxl_error = 0;
    
    if (PID_Gain.size() != 3)
    {
        std::cerr << "PID_Gain should have exactly 3 elements: P, I, and D gains." << std::endl;
        return;
    }
    
    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));

    // P, I, D Gain을 각각의 레지스터에 설정
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        // P Gain 설정
        int result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionPGain, P_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set P gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // I Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionIGain, I_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set I gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }

        // D Gain 설정
        result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error);
        if (result != COMM_SUCCESS)
        {
            std::cerr << "Failed to set D gain for DXL ID: " << static_cast<int>(dxl_id[i]) << std::endl;
        }
    }
}

//Setter() : 현재 모드 설정
// Setter() : 현재 제어 모드 설정
int16_t Dxl::SetPresentMode(int16_t mode)
{
    switch (mode)
    {
        case 0:
            this->Mode = Current_Control_Mode;
            return Current_Control_Mode;

        case 1:
            this->Mode = Velocity_Control_Mode;
            return Velocity_Control_Mode;

        case 3:
            this->Mode = Position_Control_Mode;
            return Position_Control_Mode;

        case 5:
            this->Mode = Extended_Position_Control_Mode;
            return Extended_Position_Control_Mode;

        case 16:
            this->Mode = PWM_Control_Mode;
            return PWM_Control_Mode;

        default:
            std::cerr << "[Error] Invalid mode (" << mode << ") requested. Defaulting to Current Control Mode." << std::endl;
            this->Mode = Current_Control_Mode;
            return Current_Control_Mode;
    }
}


// **************************** Function ******************************** //

//Torque2Value : 토크 -> 로우 data
int32_t Dxl::torqueToValue(double torque, uint8_t index)
{
    int32_t value_ = int(torque * torque2value[index]); //MX-64
    return value_;
}

//Value2Radian (Raw data -> Radian)
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}

//Value2Curret (Raw data -> Current)
// 1raw  = 3.36[mA]
// Range = 0 ~ 1941 (raw)
float Dxl::convertValue2Current(int32_t value)
{
    float current_ = value *3.36;
    return current_;
}

//각도(rad), 각속도(rad/s) 읽고, torque(Nm->raw) 쓰기 
//제어 주파수(전류제어 : 300, 위치제어 : ?)
void Dxl::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
    // if(TxTorque) syncWriteTorque();
    
}

//dxl 초기 세팅
void Dxl::initActuatorValues()
{
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        torque2value[i] = TORQUE_TO_VALUE_MX_106;
    }


    
    for (int i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    zero_manual_offset[i] = 0;
}










// portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error


VectorXd Dxl::read_rad()
{
    // VectorXd rdl_(NUMBER_OF_DYNAMIXELS);
    // int32_t present_position = 0;
    // for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    // {
    //     packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition,(uint32_t*)&present_position);
    //     rdl_[i] = (present_position - 2048) * (2.0 * M_PI / 4096.0);
    // }



    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        uint32_t raw_position = 0;
        uint8_t dxl_error = 0;
        int result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition, &raw_position, &dxl_error);

        if (result == COMM_SUCCESS)
        {
            position[i] = raw_position;
            th_[i] = convertValue2Radian(position[i]) - M_PI - zero_manual_offset[i];
        }
        else
        {
            std::cerr << "[ERROR] ID " << static_cast<int>(dxl_id[i])
                      << " 개별 읽기 실패 (result: " << result << ")" << std::endl;
        }
    }

    return th_;
}

void Dxl::MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms)
{
    VectorXd theta_now = read_rad();

    for (int s = 1; s <= steps; ++s)
    {
        double rate = 0.5 * (1 - cos(M_PI * double(s) / steps));
        VectorXd theta_interp = theta_now + (theta_goal - theta_now) * rate;
        SetThetaRef(theta_interp);
        syncWriteTheta();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    SetThetaRef(theta_goal);
    syncWriteTheta();
    std::this_thread::sleep_for(std::chrono::seconds(3));
}





