#pragma once

#include <Kinect.h>
#include <openvr.h>

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

typedef struct {
	vr::VRBoneTransform_t bonesLeftHand[31];
	Point3D_t LeftHandPos;
	vr::VRBoneTransform_t bonesRightHand[31];
	Point3D_t RightHandPos;
	uint32_t state;
} HandEventMsg_t;

typedef struct {
	HandEventMsg_t handMsg;
	BodyEventMsg_t bodyMsg;
} GlobalEventMsg_t;
