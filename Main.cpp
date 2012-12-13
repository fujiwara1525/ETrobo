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
	// �Z���T�A���[�^�Ȃǂ̊e�f�o�C�X�̏������֐���
	// �����Ŏ������邱�Ƃ��ł��܂�
	// �ˁ@���Z���T�ԐFLED��ON�ɂ���
	ecrobot_set_light_sensor_active(NXT_PORT_S1);
	// �ˁ@�����g�Z���T(I2C�ʐM)��������
	ecrobot_init_sonar_sensor(NXT_PORT_S3);
	
	if(ecrobot_get_bt_status() == BT_NO_INIT){
		ecrobot_set_bt_device_name("ET337");
	}
	
	ecrobot_init_bt_slave("unagipai");
}
	
void ecrobot_device_terminate(){
	// �Z���T�A���[�^�Ȃǂ̊e�f�o�C�X�̏I���֐���
	// �����Ŏ������邱�Ƃ��ł��܂��B
	// �ˁ@���Z���T�ԐFLED��OFF�ɂ���
	ecrobot_set_light_sensor_inactive(NXT_PORT_S1);
	// �ˁ@�����g�Z���T(I2C�ʐM)���I��
	ecrobot_term_sonar_sensor(NXT_PORT_S3 );
	
	ecrobot_term_bt_connection();
}
	
	// 1msec�������荞��(ISR�J�e�S��2)����N������郆�[�U�[�p�t�b�N�֐�
void user_1ms_isr_type2(void){
	// ����͉����s��Ȃ�
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
		//U16 THRESHOLD1 = ui.calibrate(100);	// PID���s臒l
		//U16 THRESHOLD2 = ui.calibrate(100);	// �D�F�}�[�J�[臒l
		//U16 THRESHOLD3 = ui.calibrate(100);	// ���b�N�A�b�v�Q�[�g���m�p臒l
		//U16 THRESHOLD4 = ui.calibrate(60);	// ���b�N�A�b�v�Q�[�g�ʉߎ�臒l
		
		// ��ԑJ�ڏ����̐ݒ�
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
			// �e�Z���T������͒l�𓾂�
			input->getInput();
			
			// forward���v�Z����
			forward += forward_var;
			if(forward >= 100){
				forward = 100;
			}
			else if(forward <= -100){
				forward = -100;
			}
			
			// tail���v�Z����
			tail += tail_var;
			
			// Control�Ɋe����Əo�͂��˗�����
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
			
			// ��ԑJ�ڏ����𖞂����Ă���ꍇ�A���̋�Ԃ̑��s���I������
			if(detection->detect(
				input->getGyroSensor(), 
				input->getBrightness(),
				input->getSonarSensor()
				)){
				break;
			}
		
			// 4msec�E�F�C�g����
			systick_wait_ms(4);
		}
		
	}
	else{
		
		ui->waitStart(100);
		while(1){
			
		}
	}
}
