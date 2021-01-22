#include <thread>
#include <iostream>

#include "interface_gesture.hpp"

#include "HandTracking.h"
#include "Bridge.h"

inline float vectorLength(vr::HmdVector3_t* v) {
	return sqrt(v->v[0] * v->v[0] + v->v[1] * v->v[1] + v->v[2] * v->v[2]);
}

inline float dotProduct(vr::HmdVector3_t * in_1, vr::HmdVector3_t * in_2)
{
	vr::HmdVector3_t v, u;
	float v_mag = vectorLength(in_1);
	float u_mag = vectorLength(in_2);
	for (int i = 0; i < 3; i++) {
		v.v[i] = in_1->v[i] / v_mag;
		u.v[i] = in_2->v[i] / u_mag;
	}
	int product = 0.0f;
	// Loop for calculate cot product 
	for (int i = 0; i < 3; i++)
		product += v.v[i] * u.v[i];

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

inline void quaternionFromTwoVectors(vr::HmdQuaternionf_t* out, vr::HmdVector3_t v[], vr::HmdVector3_t u[]) {
	vr::HmdQuaternionf_t q;
	vr::HmdVector3_t a;
	crossProduct(&a, v, u);
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
	float q_length = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	out->x = q.x / q_length;
	out->y = q.y / q_length;
	out->z = q.z / q_length;
	out->w = q.w / q_length;
}

inline void normalizeVector(vr::HmdVector3_t* out, vr::HmdVector3_t* v) {
	float v_length = vectorLength(v);
	for (int i = 0; i < 3; i++) out->v[i] = out->v[i] / v_length;

}

CHandTracking::CHandTracking() :
	m_isDataAvailable(false),
	m_handCount(0),
	m_startTryCount(0),
	m_stop(false),
	m_gestureRecognition(NULL),
	m_initThread(NULL),
	m_state(HandTrackingState::Unitialized),
	m_leftHandBonesNeedUpdate(false),
	m_rightHandBonesNeedUpdate(false)
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
	m_pMsg->state = (int)m_state;
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
			m_pMsg->state = (int)m_state;
			m_startTryCount++;
			Sleep(5000);
		}
		else {
			m_gestureRecognition = new std::thread(&CHandTracking::updateHandTracking, this);
			m_leftHandBoneUpdater = new std::thread(&CHandTracking::updateLeftHandBones, this);
			m_rightHandBoneUpdater = new std::thread(&CHandTracking::updateRightHandBones, this);
			std::cout << " HandTracking | Initilization of HandTracking successful" << std::endl;
			m_state = HandTrackingState::Initialized;
			m_pMsg->state = (int)m_state;
			return;
		}
	}
	std::cout << "HandTracking failed to start 5 times, stopped trying to start it, please check your HMD camera access" << std::endl;
}

/// <summary>
/// Set pointer to shm which will be used to update hands informations
/// <param name="pMsg">Pointer to SHM</param>
/// </summary>
void CHandTracking::SetEventMsgBuffer(HandEventMsg_t* pMsg)
{
	m_pMsg = pMsg;
	m_pMsg->leftHandGesture = GestureTypeUnknown;
	m_pMsg->rightHandGesture = GestureTypeUnknown;
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
			//std::cout << "Found " << m_handCount << " hand(s) " << std::endl;
			m_isDataAvailable = true;
		}
		bool leftHandDetected = false;
		bool rightHandDetected = false;
		for (int i = 0; i < m_handCount; i++) {
			if (m_handtrackingPoints[i].isLeft) {
				m_pMsg->leftHandGesture = m_handtrackingPoints[i].gesture;
				m_pMsg->LeftHandPos.X = m_handtrackingPoints[i].points[0];
				m_pMsg->LeftHandPos.Y = m_handtrackingPoints[i].points[1];
				m_pMsg->LeftHandPos.Z = m_handtrackingPoints[i].points[2];
				m_leftHandBonesNeedUpdate = true;
				leftHandDetected = true;
			}
			else {
				m_pMsg->rightHandGesture = m_handtrackingPoints[i].gesture;
				m_pMsg->RightHandPos.X = m_handtrackingPoints[i].points[0];
				m_pMsg->RightHandPos.Y = m_handtrackingPoints[i].points[1];
				m_pMsg->RightHandPos.Z = m_handtrackingPoints[i].points[2];
				m_rightHandBonesNeedUpdate = true;
				rightHandDetected = true;
			}
		}
		m_pMsg->leftHandDetected = leftHandDetected;
		m_pMsg->rightHandDetected = rightHandDetected;
		lastFrameIndex = frameIndex;
	}
}

void vector3to4(vr::HmdVector4_t* out, vr::HmdVector3_t v) {
	out->v[0] = v.v[0];
	out->v[1] = v.v[1];
	out->v[2] = v.v[2];
	out->v[3] = 1.0f;
}

void assignBone(vr::VRBoneTransform_t* bone, int previous_point, int current_point, int next_point, vr::HmdVector3_t* vectors) {
	vr::HmdVector3_t new_point;
	for (int i = 0; i < 3; i++) new_point.v[i] = vectors[current_point].v[i] - vectors[previous_point].v[i];
	vector3to4(&bone->position, new_point);
	quaternionFromTwoVectors(&bone->orientation, &vectors[next_point], &vectors[current_point]);
}

void assignBone(vr::VRBoneTransform_t* bone, int previous_point, int current_point, bool tip, vr::HmdVector3_t* vectors) {
	vr::HmdVector3_t new_point;
	for (int i = 0; i < 3; i++) new_point.v[i] = vectors[current_point].v[i] - vectors[previous_point].v[i];
	vector3to4(&bone->position, new_point);
	bone->orientation.w = 1.0f;
	bone->orientation.x = 0.0f;
	bone->orientation.y = 0.0f;
	bone->orientation.z = 0.0f;
}

void CHandTracking::updateRightHandBones() {
	while (true) {
		while (!m_rightHandBonesNeedUpdate) Sleep(1);
		vr::VRBoneTransform_t* bones = m_pMsg->bonesRightHand;
		float points[63];
		if (!m_handtrackingPoints[0].isLeft) memcpy(points, m_handtrackingPoints[0].points, sizeof(float) * 63);
		else memcpy(points, m_handtrackingPoints[1].points, sizeof(float) * 63);
		vr::HmdVector3_t vectors[21];
		for (int i = 0; i < 21; i++) {
			vectors[i].v[0] = points[i * 3] - points[0];
			vectors[i].v[1] = points[i * 3 + 1] - points[1];
			vectors[i].v[2] = -points[i * 3 + 2] - points[2];
		}
		for (int i = 0; i < 31; i++) {
			switch (i) {
			case HandSkeletonBone::eBone_Root:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle2, vectors); break;
			case HandSkeletonBone::eBone_Wrist:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle2, vectors); break;
			case HandSkeletonBone::eBone_Thumb0:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Thumb0, vectors); break;
			case HandSkeletonBone::eBone_Thumb1:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Thumb0, PointNaming::Thumb1, vectors); break;
			case HandSkeletonBone::eBone_Thumb2:
				assignBone(&bones[i], PointNaming::Thumb0, PointNaming::Thumb1, PointNaming::Thumb2, vectors); break;
			case HandSkeletonBone::eBone_Thumb3:
				assignBone(&bones[i], PointNaming::Thumb1, PointNaming::Thumb2, PointNaming::Thumb3, vectors); break;
			case HandSkeletonBone::eBone_IndexFinger0:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Index0, vectors); break;
			case HandSkeletonBone::eBone_IndexFinger1:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Index0, PointNaming::Index1, vectors); break;
			case HandSkeletonBone::eBone_IndexFinger2:
				assignBone(&bones[i], PointNaming::Index0, PointNaming::Index1, PointNaming::Index2, vectors); break;
			case HandSkeletonBone::eBone_IndexFinger3:
				assignBone(&bones[i], PointNaming::Index1, PointNaming::Index2, PointNaming::Index3, vectors); break;
			case HandSkeletonBone::eBone_IndexFinger4:
				assignBone(&bones[i], PointNaming::Index2, PointNaming::Index3, true, vectors); break;
			case HandSkeletonBone::eBone_MiddleFinger0:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle0, vectors); break;
			case HandSkeletonBone::eBone_MiddleFinger1:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Middle0, PointNaming::Middle1, vectors); break;
			case HandSkeletonBone::eBone_MiddleFinger2:
				assignBone(&bones[i], PointNaming::Middle0, PointNaming::Middle1, PointNaming::Middle2, vectors); break;
			case HandSkeletonBone::eBone_MiddleFinger3:
				assignBone(&bones[i], PointNaming::Middle1, PointNaming::Middle2, PointNaming::Middle3, vectors); break;
			case HandSkeletonBone::eBone_MiddleFinger4:
				assignBone(&bones[i], PointNaming::Middle2, PointNaming::Middle3, true, vectors); break;
			case HandSkeletonBone::eBone_RingFinger0:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Ring0, vectors); break;
			case HandSkeletonBone::eBone_RingFinger1:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Ring0, PointNaming::Ring1, vectors); break;
			case HandSkeletonBone::eBone_RingFinger2:
				assignBone(&bones[i], PointNaming::Ring0, PointNaming::Ring1, PointNaming::Ring2, vectors); break;
			case HandSkeletonBone::eBone_RingFinger3:
				assignBone(&bones[i], PointNaming::Ring1, PointNaming::Ring2, PointNaming::Ring3, vectors); break;
			case HandSkeletonBone::eBone_RingFinger4:
				assignBone(&bones[i], PointNaming::Ring2, PointNaming::Ring3, true, vectors); break;
			case HandSkeletonBone::eBone_PinkyFinger0:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Pinky0, vectors); break;
			case HandSkeletonBone::eBone_PinkyFinger1:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Pinky0, PointNaming::Pinky1, vectors); break;
			case HandSkeletonBone::eBone_PinkyFinger2:
				assignBone(&bones[i], PointNaming::Pinky0, PointNaming::Pinky1, PointNaming::Pinky2, vectors); break;
			case HandSkeletonBone::eBone_PinkyFinger3:
				assignBone(&bones[i], PointNaming::Pinky1, PointNaming::Pinky2, PointNaming::Pinky3, vectors); break;
			case HandSkeletonBone::eBone_PinkyFinger4:
				assignBone(&bones[i], PointNaming::Pinky2, PointNaming::Pinky3, true, vectors); break;
			case HandSkeletonBone::eBone_Aux_Thumb:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Thumb3, vectors); break;
			case HandSkeletonBone::eBone_Aux_IndexFinger:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Index3, vectors); break;
			case HandSkeletonBone::eBone_Aux_MiddleFinger:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle3, vectors); break;
			case HandSkeletonBone::eBone_Aux_RingFinger:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Ring3, vectors); break;
			case HandSkeletonBone::eBone_Aux_PinkyFinger:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Pinky3, vectors); break;
			}

		}

		m_rightHandBonesNeedUpdate = false;
	}
}

void CHandTracking::updateLeftHandBones() {
	while (true) {
		while (!m_leftHandBonesNeedUpdate) Sleep(1);
		vr::VRBoneTransform_t* bones = m_pMsg->bonesLeftHand;
		float points[63];
		if (m_handtrackingPoints[0].isLeft) memcpy(points, m_handtrackingPoints[0].points, sizeof(float) * 63);
		else memcpy(points, m_handtrackingPoints[1].points, sizeof(float) * 63);
		vr::HmdVector3_t vectors[21];
		for (int i = 0; i < 21; i++) {
			vectors[i].v[0] = -points[i * 3] - points[0];
			vectors[i].v[1] = points[i * 3 + 1] - points[1];
			vectors[i].v[2] = -points[i * 3 + 2] - points[2];
		}
		/*for (int i = 0; i < 31; i++) {
			switch (i) {
			case HandSkeletonBone::eBone_Root:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle0, vectors); break;
			case HandSkeletonBone::eBone_Wrist:
				assignBone(&bones[i], PointNaming::Base, PointNaming::Base, PointNaming::Middle0, vectors); break;
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
		}*/

		m_leftHandBonesNeedUpdate = false;
	}
	
}