#include "M32_PosCtrl.hpp"
#include "vector3.hpp"
#include "Sensors.hpp"
#include "MeasurementSystem.hpp"
#include "AC_Math.hpp"
#include "Receiver.hpp"
#include "Parameters.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "StorageSystem.hpp"
#include "Missions.hpp"
#include "NavCmdProcess.hpp"
#include "InFlightCmdProcess.hpp"
#include "drv_PWMOut.hpp"

extern uint16_t mav_mode;
extern uint16_t mav_main_mode;
extern uint16_t mav_sub_mode;

M32_PosCtrl::M32_PosCtrl():Mode_Base( "PosCtrl", 32 )
{
	
}

ModeResult M32_PosCtrl::main_func( void* param1, uint32_t param2 )
{
	mav_mode = MAV_MODE_STABILIZE_ARMED;
	mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
	mav_sub_mode = 0;
	
	double freq = 50;
	setLedMode(LEDMode_Flying1);
	Altitude_Control_Enable();
	Position_Control_Enable();
	uint16_t exit_mode_counter_rs = 0;
	uint16_t exit_mode_counter = 0;
	uint16_t exit_mode_Gcounter = 0;
	
	//任务模式
	bool MissionMode = false;
	bool mode_switched = false;
	uint16_t current_mission_ind;
	MissionInf current_mission_inf;
	double lastMissionButtonValue = -1;
	double lastRTLButtonValue = -1;
	bool last_SfRTL = false;
	//任务状态机
	NavCmdInf navInf;
	init_NavCmdInf(&navInf);
	
	//把航点设置为首个
	setCurrentMission(0);
	
	//是否处理inFlightCmd
	#define DealInFlightCmd 0
	//下一个任务递增量（中间的InFlightCmd个数）
	#define MissionInc 1
	//定距拍照当前距离倍数
	#define CamTriggDistMult 2
	
	//初始进入任务模式
	if( param2 & MAV_MODE_FLAG_AUTO_ENABLED )
		MissionMode = true;
	
	while(1)
	{
		os_delay(0.02);
		
		//获取接收机
		Receiver rc;
		getReceiver( &rc, 0, 0.02 );
		
		//接收机可用
		//进行任务、手动模式切换
		if( rc.available && get_Position_MSStatus()==MS_Ready )
		{
			//失控恢复后继续返航
			bool sticks_in_neutral = 
				in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
				in_symmetry_range_mid( rc.data[3] , 50 , 5 );
			if( !sticks_in_neutral )
				last_SfRTL = false;
			
			//判断返航
			if( rc.available_channels >=7 )
			{	
				double RTLButtonValue = rc.data[6];
				if( lastRTLButtonValue < 0 )
					lastRTLButtonValue = RTLButtonValue;
				if( fabs( RTLButtonValue - lastRTLButtonValue ) > 25 || last_SfRTL )
				{	//进入安全模式执行返航
					lastMissionButtonValue = -1;
					enter_MSafe(true);
					/*判断退出模式*/
						bool inFlight;
						get_is_inFlight(&inFlight);
						if( inFlight==false )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					/*判断退出模式*/
					continue;
				}
			}
			else
				lastRTLButtonValue = -1;
			
			//判断任务/手动模式
			double MissionButtonValue = rc.data[5];
			if( lastMissionButtonValue < 0 )
				lastMissionButtonValue = MissionButtonValue;
			else if( fabs( MissionButtonValue - lastMissionButtonValue ) > 25 )
			{
				MissionMode = !MissionMode;
				mode_switched = true;
				init_NavCmdInf(&navInf);
				lastMissionButtonValue = MissionButtonValue;
			}
		}
		else
			lastMissionButtonValue = -1;
		
		if( MissionMode )
		{	//任务模式
			mav_mode = MAV_MODE_AUTO_ARMED;
			Position_Control_Enable();
			bool pos_ena;
			is_Position_Control_Enabled(&pos_ena);
			if( pos_ena == false )
			{	//位置控制器无法打开返回手动模式
				MissionMode = false;
				goto Manual_Mode;
			}
			
			if( mode_switched )
			{	//刚切换到任务模式
				//首先刹车等待
				
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{	//摇杆不在中间返回手动模式
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				Position_Control_set_XYLock();
				Position_Control_set_ZLock();
				
				//等待刹车完成
				Position_ControlMode alt_mode, pos_mode;
				get_Altitude_ControlMode(&alt_mode);
				get_Position_ControlMode(&pos_mode);
				if( alt_mode==Position_ControlMode_Position && pos_mode==Position_ControlMode_Position )
				{	//刹车完成
					++navInf.counter2;
					//等待1秒再进入任务飞行
					if( navInf.counter2 >= 1*freq )
					{
						mode_switched = false;
						if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
						{	//载入下一航点成功
							//初始化任务信息
							init_NavCmdInf(&navInf);
						}
						else
						{	//获取不到航点信息
							//先试着把航点设置为首个
							setCurrentMission(0);
							if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
							{	//载入下一航点成功
								//初始化任务信息
								init_NavCmdInf(&navInf);
							}
							else
							{	//无航点信息返回手动模式
								MissionMode = false;
								goto Manual_Mode;
							}
						}
					}
				}
				else
					navInf.counter2 = 0;
			}
			else
			{	//任务飞行
				if( rc.available )
				{
					bool sticks_in_neutral = 
						in_symmetry_range_mid( rc.data[0] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[1] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[2] , 50 , 5 ) &&
						in_symmetry_range_mid( rc.data[3] , 50 , 5 );
					if( !sticks_in_neutral )
					{
						init_NavCmdInf(&navInf);
						MissionMode = false;
						goto Manual_Mode;
					}
				}
				
				//设定mavlink模式
				mav_mode = MAV_MODE_AUTO_ARMED;
				mav_main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
				mav_sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
				
				int16_t res = -100;
				if( Process_InflightCmd( current_mission_inf.cmd, current_mission_inf.params ) == false )
				{
					res = Process_NavCmd(
						current_mission_inf.cmd,
						freq, 
						current_mission_inf.frame,
						current_mission_inf.params,
						&navInf
					);
				}
				
				if( NavCmdRs_SuccessOrFault(res) )
				{	//错误或执行完成
					
					//不自动执行返回手动模式
					if( current_mission_inf.autocontinue == 0 )
						MissionMode = false;
					
					if( res < 0 )
					{	//切换到下一模式
						MissionInf chk_inf;
						uint16_t chk_ind;
						if( ReadCurrentMission(&chk_inf, &chk_ind) )
						{	//读取当前任务信息比较						
							if( chk_ind==current_mission_ind && memcmp( &chk_inf, &current_mission_inf, sizeof(MissionInf) ) == 0 )
							{	//如果相同才切换下一个任务
								if( setCurrentMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1 ) == false )
								{	//无航点信息返回手动模式
									setCurrentMission( 0 );
									MissionMode = false;
								}
								else
								{
									if( ReadCurrentMission(&current_mission_inf, &current_mission_ind) )
									{	//载入下一航点成功
										//初始化任务信息
										init_NavCmdInf(&navInf);
									}
									else
									{	//无航点信息返回手动模式
										setCurrentMission( 0 );
										MissionMode = false;
									}
								}
							}
							else
							{	//航点信息不相同不切换下一任务
								//使用新获取的任务信息
								current_mission_inf = chk_inf;
								//初始化任务信息
								init_NavCmdInf(&navInf);
							}
						}
						else
						{	//无航点信息返回手动模式
							setCurrentMission( 0 );
							MissionMode = false;
						}
					}
					else
					{	//切换到指定模式
						if( setCurrentMission( res ) == false )
						{	//切换失败返回手动模式
							setCurrentMission( 0 );
							MissionMode = false;			
						}
					}
				}
				else
				{	//任务执行中
					
					//定距拍照
					double camTriggDist = CamTriggDist;
					if( camTriggDist > 0 )
					{
						double flightDistance = 0;
						Position_Control_get_LineFlightDistance(&flightDistance);
						int mult = (int)(flightDistance / camTriggDist);
						if( mult > navInf.usr_temp[CamTriggDistMult] )
						{
							InflightCmd_CamTakePhoto();
							navInf.usr_temp[CamTriggDistMult] = mult;
						}
					}
					
					if( NavCmdRs_InProgress_CanExInFlightCmd(res) )
					{	//可执行InFlightCmd
						if( navInf.usr_temp[DealInFlightCmd] == 0 )
						{	//还未执行inFlightCmd
							//执行所有inFlightCmd
							MissionInf inFlightMs_inf;
							while(1)
							{
								if( ReadMission( getCurrentMissionInd() + navInf.usr_temp[MissionInc] + 1, &inFlightMs_inf ) )
								{
									if( Process_InflightCmd( inFlightMs_inf.cmd, inFlightMs_inf.params ) )
										navInf.usr_temp[MissionInc] += 1;
									else
										break;
								}
								else
									break;
							}
						}
						navInf.usr_temp[DealInFlightCmd] = 1;
					}
				}
			}
		}
		else
		{	//手动飞行模式（包含定高定点控制）
			Manual_Mode:
			mav_mode = MAV_MODE_STABILIZE_ARMED;
			if( rc.available )
			{				
				if(rc.available_channels >= 7 )
					Aux_PWM_Out(rc.data[6],0);
				if(rc.available_channels >= 8 )
					Aux_PWM_Out(rc.data[7],1);
				
				/*判断退出模式*/
					//获取飞行状态
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( rc.data[0] > 30 )
					{
						exit_mode_counter_rs = 400;
						if( exit_mode_counter < exit_mode_counter_rs )
							exit_mode_counter = exit_mode_counter_rs;
					}
					//降落后自动加锁
					if( inFlight==false && rc.data[0]<30 )
					{
						if( ++exit_mode_counter >= 500 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_counter = exit_mode_counter_rs;
					//手势加锁
					if( inFlight==false && (rc.data[0] < 10 && rc.data[1] < 10 && rc.data[2] < 10 && rc.data[3] > 90) )
					{
						if( ++exit_mode_Gcounter >= 50 )
						{
							Attitude_Control_Disable();
							return MR_OK;
						}
					}
					else
						exit_mode_Gcounter = 0;
				/*判断退出模式*/
				
				if( rc.data[4] > 60 )
				{
					Position_Control_Enable();
				}
				else if( rc.data[4] < 40 )
				{
					Position_Control_Disable();
				}
					
				//油门杆控制垂直速度
				if( in_symmetry_range_mid( rc.data[0] , 50 , 5 ) )
					Position_Control_set_ZLock();
				else
				{
					double thr_stick = remove_deadband( rc.data[0] - 50.0 , 5.0 );
					if( thr_stick > 0 )
						thr_stick *= get_maxVelUp() / 50;
					else
						thr_stick *= get_maxVelDown() / 50;
					Position_Control_set_TargetVelocityZ(thr_stick);
				}
				
				bool pos_ena;
				is_Position_Control_Enabled(&pos_ena);
				if( pos_ena )
				{
					//设定mavlink模式
					mav_mode = MAV_MODE_STABILIZE_ARMED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
					mav_sub_mode = 0;
					
					//俯仰横滚杆控水平速度
					if( in_symmetry_range_mid( rc.data[3] , 50 , 5 ) && in_symmetry_range_mid( rc.data[2] , 50 , 5 ) )
						Position_Control_set_XYLock();
					else
					{
						double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
						double XYCtrlScale = get_maxVelXY() / 50.0;
						double roll_sitck_d = remove_deadband( rc.data[3] - 50.0, 5.0 );
						double pitch_sitck_d = remove_deadband( rc.data[2] - 50.0, 5.0 );
						Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
							pitch_sitck_d * XYCtrlScale ,\
							-roll_sitck_d * XYCtrlScale , \
							( roll_sitck_d == 0 ) ? 0.3 : fabs( roll_sitck_d  )*RPCtrlScale, \
							( pitch_sitck_d == 0 ) ? 0.3 : fabs( pitch_sitck_d )*RPCtrlScale \
						);
					}
				}
				else
				{
					//设定mavlink模式
					mav_mode = MAV_MODE_STABILIZE_ARMED;
					mav_main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
					mav_sub_mode = 0;
					
					//补偿风力扰动
					vector3<double> WindDisturbance;
					get_WindDisturbance( &WindDisturbance );
					Quaternion attitude;
					get_Attitude_quat(&attitude);
					double yaw = attitude.getYaw();		
					double sin_Yaw, cos_Yaw;
					fast_sin_cos( yaw, &sin_Yaw, &cos_Yaw );
					double WindDisturbance_Bodyheading_x = ENU2BodyHeading_x( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					double WindDisturbance_Bodyheading_y = ENU2BodyHeading_y( WindDisturbance.x , WindDisturbance.y , sin_Yaw , cos_Yaw );
					
					//俯仰横滚杆控俯仰横滚
					double RPCtrlScale = degree2rad( get_maxLean() / 50.0 );
	//				Attitude_Control_set_Target_RollPitch( 
	//					( rc.data[3] - 50 )*RPCtrlScale - atan2(-WindDisturbance_Bodyheading_y , GravityAcc ),
	//					( rc.data[2] - 50 )*RPCtrlScale - atan2( WindDisturbance_Bodyheading_x , GravityAcc ) 
	//				);				
					Attitude_Control_set_Target_RollPitch( 
						( rc.data[3] - 50 )*RPCtrlScale,
						( rc.data[2] - 50 )*RPCtrlScale
					);
				}
				
				//偏航杆在中间锁偏航
				//不在中间控制偏航速度
				double YCtrlScale = degree2rad( get_maxYawSpeed() / 50.0 );
				if( in_symmetry_range_mid( rc.data[1] , 50 , 5 ) )
					Attitude_Control_set_YawLock();
				else
					Attitude_Control_set_Target_YawRate( ( 50 - rc.data[1] )*YCtrlScale );
			}
			else
			{
				last_SfRTL = true;
				//无遥控信号进入安全模式
				enter_MSafe();
				/*判断退出模式*/
					bool inFlight;
					get_is_inFlight(&inFlight);
					if( inFlight==false )
					{
						Attitude_Control_Disable();
						return MR_OK;
					}
				/*判断退出模式*/
				
			}
		}
	}
	return MR_OK;
}




