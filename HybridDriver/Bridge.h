#pragma once

#include <Kinect.h>
#include "BodyTracking.h"

typedef struct {
	float X;
	float Y;
	float Z;
} Point3D_t;

struct Quaternion_t
{
	double W, X, Y, Z;
};

typedef struct {
	Point3D_t WaistPos;
	Quaternion_t WaistRot;
	Point3D_t FootLeftPos;
	Quaternion_t FootLeftRot;
	Point3D_t FootRightPos;
	Quaternion_t FootRightRot;
	Point3D_t HandLeftPos;
	Quaternion_t HandLeftRot;
	Point3D_t HandRightPos;
	Quaternion_t HandRightRot;

	Point3D_t HeadPos;
	Quaternion_t HeadRot;

	HandState HandLeftState;
	HandState HandRightState;
} BodyEventMsg_t;
