# nxtOSEK���[�g�f�B���N�g��
NXTOSEK_ROOT = ../../../../nxtOSEK

# �^�[�Q�b�g���s�`���t�@�C����
#TARGET = Main
TARGET = oka++

# �C���N���[�h�p�X
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer

# ���C�u����
USER_LIB = nxtway_gs_balancer

# C�\�[�X�t�@�C��
TARGET_SOURCES = balancer_param.c

# C++�\�[�X�t�@�C��
TARGET_CPP_SOURCES = UI.cpp TouchSensor.cpp BlueTooth.cpp GyroSensor.cpp LightSensor.cpp Control.cpp PID.cpp BalanceControl.cpp Motor.cpp Tail.cpp SonarSensor.cpp Input.cpp SlopeDetection.cpp ColorDetection.cpp ObstacleDetection.cpp Detection.cpp Section.cpp Main.cpp

# TOPPERS/ATK1(OSEK)�ݒ�t�@�C��
TOPPERS_OSEK_OIL_SOURCE = ./model_impl.oil

# ���L�̃}�N���͕ύX���Ȃ��ł�������
O_PATH ?= build

# makefile for C++(.cpp) build
include $(NXTOSEK_ROOT)/ecrobot/ecrobot++.mak
