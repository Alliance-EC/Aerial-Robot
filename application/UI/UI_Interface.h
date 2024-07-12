#ifndef UI_INTERFACE_H
#define UI_INTERFACE_H

#include "UI.h"
#include "referee_UI.h"
#include "string.h"
#include "crc_ref.h"
#include "stdio.h"
#include "rm_referee.h"
#include "message_center.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "robot_def.h"

void Is_change(Graph_Data_t *graph_now, Graph_Data_t *graph_last);
void MY_High_Refresh();

#endif