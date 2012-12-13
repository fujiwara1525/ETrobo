# nxtOSEKルートディレクトリ
NXTOSEK_ROOT = ../../../../nxtOSEK

# ターゲット実行形式ファイル名
#TARGET = Main
TARGET = oka++

# インクルードパス
USER_INC_PATH= $(NXTOSEK_ROOT)/ecrobot/nxtway_gs_balancer

# ライブラリ
USER_LIB = nxtway_gs_balancer

# Cソースファイル
TARGET_SOURCES = balancer_param.c

# C++ソースファイル
TARGET_CPP_SOURCES = UI.cpp TouchSensor.cpp BlueTooth.cpp GyroSensor.cpp LightSensor.cpp Control.cpp PID.cpp BalanceControl.cpp Motor.cpp Tail.cpp SonarSensor.cpp Input.cpp SlopeDetection.cpp ColorDetection.cpp ObstacleDetection.cpp Detection.cpp Section.cpp Main.cpp

# TOPPERS/ATK1(OSEK)設定ファイル
TOPPERS_OSEK_OIL_SOURCE = ./model_impl.oil

# 下記のマクロは変更しないでください
O_PATH ?= build

# makefile for C++(.cpp) build
include $(NXTOSEK_ROOT)/ecrobot/ecrobot++.mak
