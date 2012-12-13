#include "LightSensor.h"
#include "GyroSensor.h"
#include "SonarSensor.h"
#include "TouchSensor.h"
#include "BlueTooth.h"
#include "Motor.h"
#include "Tail.h"
#include "Input.h"
#include "UI.h"
#include "PID.h"
#include "balanceControl.h"
#include "Control.h"
#include "SlopeDetection.h"
#include "ColorDetection.h"
#include "ObstacleDetection.h"
#include "Detection.h"
#include "Section.h"

extern "C"{
	#include "kernel.h"
	#include "kernel_id.h"
	#include "ecrobot_interface.h"
};

void ecrobot_device_initialize(){
	// センサ、モータなどの各デバイスの初期化関数を
	// ここで実装することができます
	// ⇒　光センサ赤色LEDをONにする
	ecrobot_set_light_sensor_active(NXT_PORT_S1);
	// ⇒　超音波センサ(I2C通信)を初期化
	ecrobot_init_sonar_sensor(NXT_PORT_S3);
	
	if(ecrobot_get_bt_status() == BT_NO_INIT){
		ecrobot_set_bt_device_name("ET337");
	}
	
	ecrobot_init_bt_slave("unagipai");
}
	
void ecrobot_device_terminate(){
	// センサ、モータなどの各デバイスの終了関数を
	// ここで実装することができます。
	// ⇒　光センサ赤色LEDをOFFにする
	ecrobot_set_light_sensor_inactive(NXT_PORT_S1);
	// ⇒　超音波センサ(I2C通信)を終了
	ecrobot_term_sonar_sensor(NXT_PORT_S3 );
	
	ecrobot_term_bt_connection();
}
	
	// 1msec周期割り込み(ISRカテゴリ2)から起動されるユーザー用フック関数
void user_1ms_isr_type2(void){
	// 今回は何も行わない
}

extern "C" TASK(TaskMain){
	LightSensor *lightSensor = new LightSensor(NXT_PORT_S1);
	GyroSensor *gyroSensor = new GyroSensor(NXT_PORT_S2);
	SonarSensor *sonarSensor = new SonarSensor(NXT_PORT_S3);
	TouchSensor *touchSensor = new TouchSensor(NXT_PORT_S4);
	Motor *rightMotor = new Motor(NXT_PORT_A);
	Tail *tail = new Tail(NXT_PORT_B);
	Motor *leftMotor = new Motor(NXT_PORT_C);
	BlueTooth *blueTooth = new BlueTooth();
	UI *ui = new UI(*touchSensor, *blueTooth, *lightSensor, *tail);
	Input *input = new Input(*gyroSensor, *lightSensor, *sonarSensor);
	PID *pid = new PID();
	BalanceControl *balanceControl = new BalanceControl();
	Control *control = new Control(*pid, *balanceControl, *leftMotor, *rightMotor, *tail);
	SlopeDetection *slopeDetection = new SlopeDetection();
	ColorDetection *colorDetection = new ColorDetection();
	ObstacleDetection *obstacleDetection = new ObstacleDetection();
	
	int course = ui->courseSelect();
		
	if(course == 1){
		//U16 THRESHOLD1 = ui.calibrate(100);	// PID走行閾値
		//U16 THRESHOLD2 = ui.calibrate(100);	// 灰色マーカー閾値
		//U16 THRESHOLD3 = ui.calibrate(100);	// ルックアップゲート検知用閾値
		//U16 THRESHOLD4 = ui.calibrate(60);	// ルックアップゲート通過時閾値
		
		// 区間遷移条件の設定
		//                                   gyro_threshold,    g, light_threshold,    l,    s, count
		Detection *detection = new Detection(             0,   -1,               0,   -1,   -1,   500, *slopeDetection, *colorDetection, *obstacleDetection);
		
		ui->waitStart(100);
		
		float forward = 50;
		float forward_var = 0;
		float turn = 0;
		float tail = 90;
		float tail_var = 0;
		int gyro_offset = 610;
		BOOL pid_flag = TRUE;
		U16 pid_target = 600;
		F32 KP = 0.7;
		BOOL balance_flag = FALSE;
		BOOL balance_init = FALSE;
		
		if(balance_init){
			control->resetBalance();
			balance_init = FALSE;
		}
		
		while(1){
			// 各センサから入力値を得る
			input->getInput();
			
			// forwardを計算する
			forward += forward_var;
			if(forward >= 100){
				forward = 100;
			}
			else if(forward <= -100){
				forward = -100;
			}
			
			// tailを計算する
			tail += tail_var;
			
			// Controlに各制御と出力を依頼する
			control->basicRun(
				forward,
				turn,
				tail,
				gyro_offset,
				pid_flag,
				pid_target,
				KP,
				balance_flag,
				input->getGyroSensor(),
				input->getBrightness()
				);
			
			// 状態遷移条件を満たしている場合、この区間の走行を終了する
			if(detection->detect(
				input->getGyroSensor(), 
				input->getBrightness(),
				input->getSonarSensor()
				)){
				break;
			}
		
			// 4msecウェイトする
			systick_wait_ms(4);
		}
		
	}
	else{
		
		ui->waitStart(100);
		while(1){
			
		}
	}
}
