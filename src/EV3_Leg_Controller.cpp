// -*- C++ -*-
/*!
 * @file  EV3_Leg_Controller.cpp
 * @brief EV3 Leg Controller
 * @date $Date$
 *
 * @author 宮本　信彦　n-miyamoto@aist.go.jp
 * 産業技術総合研究所　ロボットイノベーション研究センター
 * ロボットソフトウエアプラットフォーム研究チーム
 *
 * LGPL
 *
 * $Id$
 */

#include "EV3_Leg_Controller.h"

// Module specification
// <rtc-template block="module_spec">
static const char* ev3_leg_controller_spec[] =
  {
    "implementation_id", "EV3_Leg_Controller",
    "type_name",         "EV3_Leg_Controller",
    "description",       "EV3 Leg Controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.motor0_speed", "10.0",
    "conf.default.motor1_speed", "8.0",
    "conf.default.motor2_speed", "8.0",
    "conf.default.motor0_gear_ratio", "2.333",
    "conf.default.motor1_gear_ratio", "4.0",
    "conf.default.motor2_gear_ratio", "4.0",
    "conf.default.motor0_offset", "0",
    "conf.default.motor1_offset", "0",
    "conf.default.motor2_offset", "0",

    // Widget
    "conf.__widget__.motor0_speed", "text",
    "conf.__widget__.motor1_speed", "text",
    "conf.__widget__.motor2_speed", "text",
    "conf.__widget__.motor0_gear_ratio", "text",
    "conf.__widget__.motor1_gear_ratio", "text",
    "conf.__widget__.motor2_gear_ratio", "text",
    "conf.__widget__.motor0_offset", "text",
    "conf.__widget__.motor1_offset", "text",
    "conf.__widget__.motor2_offset", "text",
    // Constraints

    "conf.__type__.motor0_speed", "double",
    "conf.__type__.motor1_speed", "double",
    "conf.__type__.motor2_speed", "double",
    "conf.__type__.motor0_gear_ratio", "double",
    "conf.__type__.motor1_gear_ratio", "double",
    "conf.__type__.motor2_gear_ratio", "double",
    "conf.__type__.motor0_offset", "double",
    "conf.__type__.motor1_offset", "double",
    "conf.__type__.motor2_offset", "double",


    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
EV3_Leg_Controller::EV3_Leg_Controller(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inIn("in", m_in),
    m_outOut("out", m_out)

    // </rtc-template>
{
	motor0 = NULL;
	motor1 = NULL;
	motor2 = NULL;
}

/*!
 * @brief destructor
 */
EV3_Leg_Controller::~EV3_Leg_Controller()
{
}



RTC::ReturnCode_t EV3_Leg_Controller::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("in", m_inIn);
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("motor0_speed", m_motor0_speed, "10.0");
  bindParameter("motor1_speed", m_motor1_speed, "8.0");
  bindParameter("motor2_speed", m_motor2_speed, "8.0");
  bindParameter("motor0_gear_ratio", m_motor0_gear_ratio, "2.333");
  bindParameter("motor1_gear_ratio", m_motor1_gear_ratio, "4.0");
  bindParameter("motor2_gear_ratio", m_motor2_gear_ratio, "4.0");
  bindParameter("motor0_offset", m_motor0_offset, "0");
  bindParameter("motor1_offset", m_motor1_offset, "0");
  bindParameter("motor2_offset", m_motor2_offset, "0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onFinalize()
{
	if(motor0)
	{
		if(motor0->connected())
		{
			motor0->reset();
		}
		delete motor0;
	}
	if(motor1)
	{
		if(motor1->connected())
		{
			motor1->reset();
		}
		delete motor1;
	}
	if(motor2)
	{
		if(motor2->connected())
		{
			motor2->reset();
		}
		delete motor2;
	}
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onActivated(RTC::UniqueId ec_id)
{
	if(motor0 == NULL)
	{
		motor0 = new ev3dev::medium_motor();
		if(motor0->connected())
		{
			motor0->reset();
			motor0->set_stop_command(ev3dev::motor::stop_command_hold);
		}
		else
		{
			delete motor0;
			motor0 = NULL;
			return RTC::RTC_ERROR;
		}

	}
	if(motor1 == NULL)
	{
		motor1 = new ev3dev::large_motor(MOTOR1_ADDRESS);
		if(motor1->connected())
		{
			motor1->reset();
			motor1->set_stop_command(ev3dev::motor::stop_command_hold);
		}
		else
		{
			delete motor1;
			motor1 = NULL;
			return RTC::RTC_ERROR;
		}
	}
	if(motor2 == NULL)
	{
		motor2 = new ev3dev::large_motor(MOTOR2_ADDRESS);
		if(motor2->connected())
		{
			motor2->reset();
			motor2->set_stop_command(ev3dev::motor::stop_command_hold);
		}
		else
		{
			delete motor2;
			motor2 = NULL;
			return RTC::RTC_ERROR;
		}
	}
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onDeactivated(RTC::UniqueId ec_id)
{
	if(motor0->connected())
	{
		motor0->stop();
	}
	if(motor1->connected())
	{
		motor1->stop();
	}
	if(motor2->connected())
	{
		motor2->stop();
	}
  return RTC::RTC_OK;
}

/**
* @brief モーターの位置設定
* @param m モーター
* @param p 位置
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void EV3_Leg_Controller::set_position_motor(ev3dev::motor *m, float p, float v, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		m->set_speed_regulation_enabled("on");
		m->set_speed_sp(rot_to_count(v, m, ret));
		m->set_position_sp(p*180/M_PI);
		m->run_to_abs_pos();
		return;
	}
	ret = false;
}

/**
* @brief モーターの位置取得
* @param m モーター
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float EV3_Leg_Controller::get_position_motor(ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)m->position()*M_PI/180.0;
	}
	ret = false;
	return 0;
}

/**
* @brief 角度をモーターのカウントに変換
* @param rot 角度
* @param m モーター
* @param ret 変換できた場合はtrue、失敗した場合はfalse
* @return カウント
*/
int EV3_Leg_Controller::rot_to_count(float rot, ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return rot*(180/M_PI)*(m->count_per_rot()/(float)360.0);
	}
	ret = false;
	return 0;
}

/**
* @brief モーターのカウントを角度に変換
* @param count カウント
* @param m モーター
* @param ret 変換できた場合はtrue、失敗した場合はfalse
* @return 角度
*/
float EV3_Leg_Controller::count_to_rot(int count, ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)count/(180/M_PI)/((float)m->count_per_rot()/360.0);
	}
	ret = false;
	return 0;
}


RTC::ReturnCode_t EV3_Leg_Controller::onExecute(RTC::UniqueId ec_id)
{
	if(m_inIn.isNew())
	{
		m_inIn.read();
		for(int i=0;i < m_in.data.length();i++)
		{
			bool ret = true;
			if(i == 0)
			{
				set_position_motor(motor0, m_in.data[0]*m_motor0_gear_ratio+m_motor0_offset, m_motor0_speed, ret);
			}
			else if(i == 1)
			{
				set_position_motor(motor1, m_in.data[1]*m_motor1_gear_ratio+m_motor1_offset, m_motor1_speed, ret);
			}
			else if(i == 2)
			{
				set_position_motor(motor2, m_in.data[2]*m_motor2_gear_ratio+m_motor2_offset, m_motor2_speed, ret);
			}
			if(ret == false)
			{
				return RTC::RTC_ERROR;
			}
		}
	}
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EV3_Leg_Controller::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}



extern "C"
{
 
  void EV3_Leg_ControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(ev3_leg_controller_spec);
    manager->registerFactory(profile,
                             RTC::Create<EV3_Leg_Controller>,
                             RTC::Delete<EV3_Leg_Controller>);
  }
  
};


