#include <thread>

#include "driverlog.h"
#include "interface_gesture.hpp"

#include "HandTracking.h"

inline float dotProduct(vr::HmdVector3_t v[], vr::HmdVector3_t u[])
{
	int product = 0.0f;
	// Loop for calculate cot product 
	for (int i = 0; i < 3; i++)
		product += v->v[i] * u->v[i];

	return product;
}

// Function to find 
// cross product of two vector array. 
inline vr::HmdVector3_t crossProduct(vr::HmdVector3_t * v, vr::HmdVector3_t * u)
{
	vr::HmdVector3_t out;
	out.v[0] = v->v[1] * u->v[2] - v->v[2] * u->v[1];
	out.v[1] = v->v[2] * u->v[0] - v->v[0] * u->v[2];
	out.v[2] = v->v[0] * u->v[1] - v->v[1] * u->v[0];
	return out;
}

inline float vectorLength(vr::HmdVector3_t* v) {
	return sqrt(v->v[0] * v->v[0] + v->v[1] * v->v[1] + v->v[2] * v->v[2]);
}

inline vr::HmdQuaternionf_t quaternionFromTwoVectors(vr::HmdVector3_t v[], vr::HmdVector3_t u[]) {
	vr::HmdQuaternionf_t q;
	vr::HmdVector3_t a = crossProduct(v, u);
	q.x = a.v[0];
	q.y = a.v[1];
	q.z = a.v[2];
	float dotP = dotProduct(u, v);
	if (dotP > 0.999999) { // Handling parallel vectors
		q.x = 0.0f;
		q.y = 0.0f;
		q.z = 0.0f;
		q.w = 1.0f;
	}
	float v_length = vectorLength(v);
	float u_length = vectorLength(u);
	//Todo: Handle opposite vectors
	q.w = sqrt(v_length * v_length * u_length * u_length) + dotP;
	return q;
}

CHandTracking::CHandTracking() :
m_isDataAvailable(false),
m_handCount(0),
m_stop(false),
m_state(HandTrackingState::Unitialized)
{

}

CHandTracking::~CHandTracking() {
	m_stop = true;
	m_gestureRecognition->join();
	StopGestureDetection();
}

void CHandTracking::InitializeDefaultSensor() {
	m_state = HandTrackingState::Initializing;
	std::thread* init_thread = new std::thread(&CHandTracking::initialize, this);
}

void CHandTracking::initialize() {
	GestureOption option;
	option.maxFPS = 90;
	//option.mode = GestureMode3DPoint;
	UseExternalTransform(true);
	GestureFailure result = StartGestureDetection(&option);
	if (result != GestureFailureNone) {
		DriverLog("Initilization of HandTracking failed");
		m_state = HandTrackingState::Error;
		return;
	}

	m_gestureRecognition = new std::thread(&CHandTracking::updateHandTracking, this);

	DriverLog("Initilization of HandTracking successful");
	m_state = HandTrackingState::Initialized;
	return;
}

bool CHandTracking::isDataAvailable() {
	return m_isDataAvailable;
}

GestureResult* CHandTracking::getHandTrackingData(int* handCount) {
	m_isDataAvailable = false;
	*handCount = m_handCount;
	return m_handtrackingPoints;
}

void CHandTracking::updateHandTracking() {
	int lastFrameIndex = -1;
	while (!m_stop) {
		int frameIndex = -1;
		m_handCount = GetGestureResult((const GestureResult**)&m_handtrackingPoints, &frameIndex);
		m_handtrackingPoints->isLeft;
		m_handtrackingPoints->points;
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(500ns);
		if (frameIndex < 0) {
			break;
		}
		else if (frameIndex == lastFrameIndex)
			continue;

		if (m_handCount > 0) m_isDataAvailable = true;
		lastFrameIndex = frameIndex;
		


	}
}

vr::HmdVector4_t vector3to4(vr::HmdVector3_t v) {
	vr::HmdVector4_t out;
	out.v[0] = v.v[0];
	out.v[1] = v.v[1];
	out.v[2] = v.v[2];
	out.v[3] = 1.0f;
	return out;
}

void assignBone(vr::VRBoneTransform_t* bone, int point_one, int point_two, vr::HmdVector3_t* vectors) {
	bone->position = vector3to4(vectors[point_one]);
	bone->orientation = quaternionFromTwoVectors(&vectors[point_one], &vectors[point_two]);
}

void assignBone(vr::VRBoneTransform_t* bone, int point_one, int point_two, int fake_orientation_point, vr::HmdVector3_t* vectors) {
	bone->position = vector3to4(vectors[point_one]);
	bone->orientation = quaternionFromTwoVectors(&vectors[fake_orientation_point], &vectors[point_two]);
}


void CHandTracking::getRightHandBones(vr::VRBoneTransform_t* bones, float points[]) {
	vr::HmdVector3_t vectors[21];
	for (int i = 0; i < 21; i++) {
		vectors[i].v[0] = points[i * 3];
		vectors[i].v[1] = points[i * 3 + 1];
		vectors[i].v[2] = -points[i * 3 + 2];
	}
	for (int i = 0; i < 31; i++) {
		switch (i) {
		case HandSkeletonBone::eBone_Root:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Middle0, vectors); break;
		case HandSkeletonBone::eBone_Wrist:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Middle0, vectors); break;
		case HandSkeletonBone::eBone_Thumb0:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Thumb0, vectors); break;
		case HandSkeletonBone::eBone_Thumb1:
			assignBone(&bones[i], PointNaming::Thumb0, PointNaming::Thumb1, vectors); break;
		case HandSkeletonBone::eBone_Thumb2:
			assignBone(&bones[i], PointNaming::Thumb1, PointNaming::Thumb2, vectors); break;
		case HandSkeletonBone::eBone_Thumb3:
			assignBone(&bones[i], PointNaming::Thumb2, PointNaming::Thumb3, vectors); break;
		case HandSkeletonBone::eBone_IndexFinger0:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Index0, vectors); break;
		case HandSkeletonBone::eBone_IndexFinger1:
			assignBone(&bones[i], PointNaming::Index0, PointNaming::Index1, vectors); break;
		case HandSkeletonBone::eBone_IndexFinger2:
			assignBone(&bones[i], PointNaming::Index1, PointNaming::Index2, vectors); break;
		case HandSkeletonBone::eBone_IndexFinger3:
			assignBone(&bones[i], PointNaming::Index2, PointNaming::Index3, vectors); break;
		case HandSkeletonBone::eBone_IndexFinger4:
			assignBone(&bones[i], PointNaming::Index3, PointNaming::Index3, PointNaming::Index2, vectors); break;
		case HandSkeletonBone::eBone_MiddleFinger0:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Middle0, vectors); break;
		case HandSkeletonBone::eBone_MiddleFinger1:
			assignBone(&bones[i], PointNaming::Middle0, PointNaming::Middle1, vectors); break;
		case HandSkeletonBone::eBone_MiddleFinger2:
			assignBone(&bones[i], PointNaming::Middle1, PointNaming::Middle2, vectors); break;
		case HandSkeletonBone::eBone_MiddleFinger3:
			assignBone(&bones[i], PointNaming::Middle2, PointNaming::Middle3, vectors); break;
		case HandSkeletonBone::eBone_MiddleFinger4:
			assignBone(&bones[i], PointNaming::Middle3, PointNaming::Middle3, PointNaming::Middle2, vectors); break;
		case HandSkeletonBone::eBone_RingFinger0:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Ring0, vectors); break;
		case HandSkeletonBone::eBone_RingFinger1:
			assignBone(&bones[i], PointNaming::Ring0, PointNaming::Ring1, vectors); break;
		case HandSkeletonBone::eBone_RingFinger2:
			assignBone(&bones[i], PointNaming::Ring1, PointNaming::Ring2, vectors); break;
		case HandSkeletonBone::eBone_RingFinger3:
			assignBone(&bones[i], PointNaming::Ring2, PointNaming::Ring3, vectors); break;
		case HandSkeletonBone::eBone_RingFinger4:
			assignBone(&bones[i], PointNaming::Ring3, PointNaming::Ring3, PointNaming::Ring2, vectors); break;
		case HandSkeletonBone::eBone_PinkyFinger0:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Pinky0, vectors); break;
		case HandSkeletonBone::eBone_PinkyFinger1:
			assignBone(&bones[i], PointNaming::Pinky0, PointNaming::Pinky1, vectors); break;
		case HandSkeletonBone::eBone_PinkyFinger2:
			assignBone(&bones[i], PointNaming::Pinky1, PointNaming::Pinky2, vectors); break;
		case HandSkeletonBone::eBone_PinkyFinger3:
			assignBone(&bones[i], PointNaming::Pinky2, PointNaming::Pinky3, vectors); break;
		case HandSkeletonBone::eBone_PinkyFinger4:
			assignBone(&bones[i], PointNaming::Pinky3, PointNaming::Pinky3, PointNaming::Pinky2, vectors); break;
		case HandSkeletonBone::eBone_Aux_Thumb:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Thumb3, vectors); break;
		case HandSkeletonBone::eBone_Aux_IndexFinger:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Index3, vectors); break;
		case HandSkeletonBone::eBone_Aux_MiddleFinger:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Middle3, vectors); break;
		case HandSkeletonBone::eBone_Aux_RingFinger:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Ring3, vectors); break;
		case HandSkeletonBone::eBone_Aux_PinkyFinger:
			assignBone(&bones[i], PointNaming::Base, PointNaming::Pinky3, vectors); break;
		}
		
	}
}