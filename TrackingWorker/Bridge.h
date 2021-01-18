#pragma once

#include <Kinect.h>
#include "BodyTracking.h"

#define KINECT_NAMED_PIPE L"\\\\.\\pipe\\kinect_bridge_pipe"

typedef struct {
	float X;
	float Y;
	float Z;
} Point3D_t;

typedef struct {
	Point3D_t WaistPos;
	Point3D_t FootLeftPos;
	Point3D_t FootRightPos;
	Point3D_t HandLeftPos;
	Point3D_t HandRightPos;
	HandState HandLeftState;
	HandState HandRightState;
} BodyEventMsg_t;