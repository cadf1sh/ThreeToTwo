/**
  ******************************************************************************
  * ?: 
  *     : ?
  *     : V1.0
  * §Õ: 
  *     : 
  ******************************************************************************
  */
/* ?? ----------------------------------------------------------------*/

#include "motor_publicdata.h"

MOTORCONTROL_STRUCT MC;                           //???

/**
  * :?? 
  * :
  * ?:
  * ?    : 
  */
void Motor_Struct_Init()
{	
	/*?*/
	MC.Motor.RunState = ADC_CALIB;      			 	  	 //???
#if MOTOR_TWO_PHASE_STEPPER
	MC.Motor.RunMode = CURRENT_CLOSE_LOOP;           // Stepper: default current loop
#else
	MC.Motor.RunMode = ENCODER_CALIB;                //§Ü??
#endif
		
	MC.Sample.CurrentDir = 1;       					  	   //??(?)
	MC.Sample.CurrentFactor = PHASE_CURRENT_FACTOR;  //?(?????ADC??¨®)
	MC.Sample.BusFactor = VBUS_FACTOR;               //??????¨®

	MC.Encoder.Dir = CCW;              							 //???? ??0360?
	MC.Encoder.PolePairs = POLEPAIRS;								 //??2
	MC.Encoder.EncoderValMax = PUL_MAX;  					   //???
	
	MC.Foc.IdLPFFactor = 0.1f;	                     //d???
	MC.Foc.IqLPFFactor = 0.1f;	                     //q???
	MC.Foc.PwmCycle = PWM_CYCLE;									   //PWM
	MC.Foc.PwmLimit = PWM_LIMLT;                           // PWM limit (Stepper: Ualpha/Ubeta limit)
	
	MC.Position.ElectricalValMax = PUL_MAX; 			   //???
	
	MC.TAccDec.AccSpeed = ACCELERATION;              //??????	
	
	MC.Speed.ElectricalValMax = PUL_MAX; 					   //???	
	MC.Speed.ElectricalSpeedLPFFactor = 0.05f;       //?????
	MC.Speed.ElectricalSpeedFactor = 146.5f;         //???

	MC.Identify.CurMax = 0.6f;                       //?????¦Ë
	
	MC.SMO.Gain = 14.0f;                             //???
	MC.SMO.Ts = TS;                                  //????
  MC.SMO.EabForeLPFFactor = 0.1f;                  //?¾T????
	
	MC.SPLL.Ts = TS;                                 //??
	MC.SPLL.Kp = 80.0f;                              //??
	MC.SPLL.Ki = 0.5f;                               //??
	MC.SPLL.WeForeLPFFactor = 0.01f;	               //¨´??????
	
	MC.HFI.Uin = 1.4f;	                             //??????
	MC.HPLL.Dir = 1;                                 //??
	MC.HPLL.Kp = 900.0f;                             //??
	MC.HPLL.Ki = 20.0f;                              //??
	MC.HPLL.Ts = TS;                                 //??
	MC.HPLL.WeForeLPFFactor = 0.01f;                 //¨´??????
	
	MC.IqPid.Kp = 0.2f;                              //qPID?
	MC.IqPid.Ki = 0.002f;                            //qPID?
	MC.IqPid.OutMax = 6;                             //qPID
	MC.IqPid.OutMin = -6;                            //qPID

	MC.IdPid.Kp = 0.2f;                              //dPID?
	MC.IdPid.Ki = 0.002f;                            //dPID?
	MC.IdPid.OutMax = 6;                             //dPID
	MC.IdPid.OutMin = -6;                            //dPID

#if MOTOR_TWO_PHASE_STEPPER
	MC.IdPid.Ref = 0.0f;                             // Stepper demo: Id = 0
	MC.IqPid.Ref = 0.6f;                             // Stepper demo: Iq constant (adjust as needed)
	MC.Encoder.ElectricalSpdSet = 200.0f;           // Stepper demo: electrical speed command
#endif

	MC.SpdPid.Kp = 0.001f;                           //?PID?
	MC.SpdPid.KpMax = 0.005f;                        //?PID????¦Ë?PID
	MC.SpdPid.KpMin = 0.001f;	                       //?PID?§³???¦Ë?PID
	MC.SpdPid.Ki = 0.000002f;                        //?PID?
	MC.SpdPid.OutMax = 8;                            //?PID  
	MC.SpdPid.OutMin = -8;	                         //?PID

  MC.PosPid.Kp = 0.5f;                             //¦ËPID?
	MC.PosPid.Ki = 0;                                //¦ËPID?
	MC.PosPid.Kd = 0;                                //¦ËPID??
	MC.PosPid.OutMax = 14000;                        //¦ËPID
	MC.PosPid.OutMin = -14000;                       //¦ËPID
}  