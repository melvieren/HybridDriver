#include <thread>
#include <iostream>

#include "interface_gesture.hpp"

#include "HandTracking.h"
#include "Bridge.h"

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
inline void crossProduct(vr::HmdVector3_t* out, vr::HmdVector3_t* v, vr::HmdVector3_t* u)
{
	out->v[0] = v->v[1] * u->v[2] - v->v[2] * u->v[1];
	out->v[1] = v->v[2] * u->v[0] - v->v[0] * u->v[2];
	out->v[2] = v->v[0] * u->v[1] - v->v[1] * u->v[0];
}

inline float vectorLength(vr::HmdVector3_t* v) {
	return sqrt(v->v[0] * v->v[0] + v->v[1] * v->v[1] + v->v[2] * v->v[2]);
}

inline void quaternionFromTwoVectors(vr::HmdQuaternionf_t* out, vr::HmdVector3_t v[], vr::HmdVector3_t u[]) {
	vr::HmdQuaternionf_t q;
	vr::HmdVector3_t a;
	crossProduct(&a, v, u);
	out->x = a.v[0];
	out->y = a.v[1];
	out->z = a.v[2];
	float dotP = dotProduct(u, v);
	if (dotP > 0.999999) { // Handling parallel vectors
		out->x = 0.0f;
		out->y = 0.0f;
		out->z = 0.0f;
		out->w = 1.0f;
	}
	float v_length = vectorLength(v);
	float u_length = vectorLength(u);
	//Todo: Handle opposite vectors
	out->w = sqrt(v_length * v_length * u_length * u_length) + dotP;
}

CHandTracking::CHandTracking() :
	m_isDataAvailable(false),
	m_handCount(0),
	m_startTryCount(0),
	m_stop(false),
	m_gestureRecognition(NULL),
	m_initThread(NULL),
	m_state(HandTrackingState::Unitialized)
{

}

CHandTracking::~CHandTracking() {
	std::cout << " HandTracking | Destructing HandTracking structure" << std::endl;
	m_stop = true;
	if (m_gestureRecognition != NULL) m_gestureRecognition->join();
	StopGestureDetection();
	std::cout << " HandTracking | Stopped gesture detection" << std::endl;
}

void CHandTracking::InitializeDefaultSensor() {
	m_state = HandTrackingState::Initializing;
	std::thread * init = new std::thread(&CHandTracking::initialize, this);
}

void CHandTracking::initialize() {
	while (m_startTryCount < 5) {
		GestureOption option;
		// option.maxFPS = 60;
		// option.mode = GestureMode3DPoint;
		UseExternalTransform(true);
		GestureFailure result = StartGestureDetection(&option);
		if (result != GestureFailureNone) {
			std::cout << " HandTracking | Initilization of HandTracking failed, error code " << result << std::endl;
			m_state = HandTrackingState::Error;
			m_startTryCount++;
			Sleep(5000);
		}
		else {
			m_gestureRecognition = new std::thread(&CHandTracking::updateHandTracking, this);
			std::cout << " HandTracking | Initilization of HandTracking successful" << std::endl;
			m_state = HandTrackingState::Initialized;
			return;
		}
	}
	std::cout << "HandTracking failed to start 5 times, stopped trying to start it, please check your HMD camera access" << std::endl;
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
		using namespace std::chrono_literals;
		std::this_thread::sleep_for(500ns);
		if (frameIndex < 0) {
			break;
		}
		else if (frameIndex == lastFrameIndex)
			continue;

		if (m_handCount > 0) {
			std::cout << "Found " << m_handCount << " hand(s) " << std::endl;
			m_isDataAvailable = true;
		}
		lastFrameIndex = frameIndex;
	}
}

void vector3to4(vr::HmdVector4_t* out, vr::HmdVector3_t v) {
	out->v[0] = v.v[0];
	out->v[1] = v.v[1];
	out->v[2] = v.v[2];
	out->v[3] = 1.0f;
}

void assignBone(vr::VRBoneTransform_t* bone, int point_one, int point_two, vr::HmdVector3_t* vectors) {
	vector3to4(&bone->position, vectors[point_one]);
	quaternionFromTwoVectors(&bone->orientation, &vectors[point_one], &vectors[point_two]);
}

void assignBone(vr::VRBoneTransform_t* bone, int point_one, int point_two, int fake_orientation_point, vr::HmdVector3_t* vectors) {
	vector3to4(&bone->position, vectors[point_one]);
	quaternionFromTwoVectors(&bone->orientation, &vectors[fake_orientation_point], &vectors[point_two]);
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