// -*- C++ -*-
/*!
 * @file  EV3_Leg_Controller.h
 * @brief EV3 Leg Controller
 * @date  $Date$
 *
 * @author 宮本　信彦　n-miyamoto@aist.go.jp
 * 産業技術総合研究所　ロボットイノベーション研究センター
 * ロボットソフトウエアプラットフォーム研究チーム
 *
 * LGPL
 *
 * $Id$
 */

#ifndef EV3_LEG_CONTROLLER_H
#define EV3_LEG_CONTROLLER_H

//#define MOTOR0_ADDRESS ev3dev::OUTPUT_A
#define MOTOR1_ADDRESS ev3dev::OUTPUT_B
#define MOTOR2_ADDRESS ev3dev::OUTPUT_C

#include <ev3dev.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

/*!
 * @class EV3_Leg_Controller
 * @brief EV3 Leg Controller
 *
 * EV3で脚関節のモーターを制御するコンポーネント
 *
 */
class EV3_Leg_Controller
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  EV3_Leg_Controller(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~EV3_Leg_Controller();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * モーター0(Mモーター)の速度
   * - Name: motor0_speed motor0_speed
   * - DefaultValue: 10.0
   * - Unit: rad/s
   */
  double m_motor0_speed;
  /*!
   * モーター1(Lモーター)の速度
   * - Name: motor1_speed motor1_speed
   * - DefaultValue: 8.0
   * - Unit: rad/s
   */
  double m_motor1_speed;
  /*!
   * モーター2(Lモーター)の速度
   * - Name: motor2_speed motor2_speed
   * - DefaultValue: 8.0
   * - Unit: rad/s
   */
  double m_motor2_speed;
  /*!
   * 関節0のギア比
   * - Name: motor0_gear_ratio motor0_gear_ratio
   * - DefaultValue: 2.333
   */
  double m_motor0_gear_ratio;
  /*!
   * 関節1のギア比
   * - Name: motor1_gear_ratio motor1_gear_ratio
   * - DefaultValue: 4.0
   */
  double m_motor1_gear_ratio;
  /*!
   * 関節2のギア比
   * - Name: motor2_gear_ratio motor2_gear_ratio
   * - DefaultValue: 4.0
   */
  double m_motor2_gear_ratio;
  /*!
   * モーター0のオフセット
   * - Name: motor0_offset motor0_offset
   * - DefaultValue: 0
   * - Unit: rad
   */
  double m_motor0_offset;
  /*!
   * モーター1のオフセット
   * - Name: motor1_offset motor1_offset
   * - DefaultValue: 0
   * - Unit: rad
   */
  double m_motor1_offset;
  /*!
   * モーター2のオフセット
   * - Name: motor2_offset motor2_offset
   * - DefaultValue: 0
   * - Unit: rad
   */
  double m_motor2_offset;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedDoubleSeq m_in;
  /*!
   * モーターの目標角度
   * - Type: RTC::TimedDoubleSeq
   * - Number: 3
   * - Unit: rad
   */
  InPort<RTC::TimedDoubleSeq> m_inIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_out;
  /*!
   * モーターの現在の角度
   * - Type: RTC::TimedDoubleSeq
   * - Number: 3
   * - Unit: rad
   */
  OutPort<RTC::TimedDoubleSeq> m_outOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
	/**
	* @brief モーターの位置設定
	* @param m モーター
	* @param p 位置
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_position_motor(ev3dev::motor *m, float p, float v, bool &ret);
	/**
	* @brief モーターの位置取得
	* @param m モーター
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 位置
	*/
	float get_position_motor(ev3dev::motor *m, bool &ret);
	/**
	* @brief 角度をモーターのカウントに変換
	* @param rot 角度
	* @param m モーター
	* @param ret 変換できた場合はtrue、失敗した場合はfalse
	* @return カウント
	*/
	int rot_to_count(float rot, ev3dev::motor *m, bool &ret);
	/**
	* @brief モーターのカウントを角度に変換
	* @param count カウント
	* @param m モーター
	* @param ret 変換できた場合はtrue、失敗した場合はfalse
	* @return 角度
	*/
	float count_to_rot(int count, ev3dev::motor *m, bool &ret);

	ev3dev::medium_motor *motor0;
	ev3dev::large_motor *motor1;
	ev3dev::large_motor *motor2;
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void EV3_Leg_ControllerInit(RTC::Manager* manager);
};

#endif // EV3_LEG_CONTROLLER_H
