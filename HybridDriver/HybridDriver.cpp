//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>

//#include <windows.h>
#include <sstream>
#if defined( _WINDOWS )
#include <windows.h>
#endif

#include "Bridge.h"

#include "HandTracking.h"

#include <interface_gesture.hpp>

#include <stdio.h>
#include <conio.h>
#include <tchar.h>

#define BUF_SIZE 256
TCHAR szName[] = TEXT("Global\HybridDriverSHM");

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

HANDLE g_hSharedMem = NULL;
GlobalEventMsg_t* g_SharedBuf = NULL;

int initSharedMemory(void)
{
	//DriverLog("Initializing SHM");

	g_hSharedMem = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, szName);
	if (g_hSharedMem == NULL)
	{
		return 0;
	}

	g_SharedBuf = (GlobalEventMsg_t *)MapViewOfFile(g_hSharedMem, FILE_MAP_ALL_ACCESS, 0, 0, BUF_SIZE);
	if (g_SharedBuf == NULL)
	{
		CloseHandle(g_hSharedMem);
		return 0;
	}

	DriverLog("Successfully initialized SHM ! g_SharedBuf at %p", g_SharedBuf);
	return 1;
}

HmdVector3d_t MultiplyByQuaternion(HmdQuaternion_t quat, HmdVector3d_t vec) {
	float num = quat.x * 2.0f;
	float num2 = quat.y * 2.0f;
	float num3 = quat.z * 2.0f;
	float num4 = quat.x * num;
	float num5 = quat.y * num2;
	float num6 = quat.z * num3;
	float num7 = quat.x * num2;
	float num8 = quat.x * num3;
	float num9 = quat.y * num3;
	float num10 = quat.w * num;
	float num11 = quat.w * num2;
	float num12 = quat.w * num3;
	HmdVector3d_t result;
	result.v[0] = (1.0f - (num5 + num6)) * vec.v[0] + (num7 - num12) * vec.v[1] + (num8 + num11) * vec.v[2];
	result.v[1] = (num7 + num12) * vec.v[0] + (1.0f - (num4 + num6)) * vec.v[1] + (num9 - num10) * vec.v[2];
	result.v[2] = (num8 - num11) * vec.v[0] + (num9 + num10) * vec.v[1] + (1.0f - (num4 + num5)) * vec.v[2];
	return result;
}

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

vr::HmdQuaternion_t GetHMDRotation(vr::HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

inline void HmdMatrix_SetIdentity( HmdMatrix34_t *pMatrix )
{
	pMatrix->m[0][0] = 1.f;
	pMatrix->m[0][1] = 0.f;
	pMatrix->m[0][2] = 0.f;
	pMatrix->m[0][3] = 0.f;
	pMatrix->m[1][0] = 0.f;
	pMatrix->m[1][1] = 1.f;
	pMatrix->m[1][2] = 0.f;
	pMatrix->m[1][3] = 0.f;
	pMatrix->m[2][0] = 0.f;
	pMatrix->m[2][1] = 0.f;
	pMatrix->m[2][2] = 1.f;
	pMatrix->m[2][3] = 0.f;
}
// keys for use with the settings API
static const char * const k_pch_Sample_Section = "driver_sample";
static const char * const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char * const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char * const k_pch_Sample_WindowX_Int32 = "windowX";
static const char * const k_pch_Sample_WindowY_Int32 = "windowY";
static const char * const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char * const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char * const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char * const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char * const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char * const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

enum class TrackerType { Undefined = -1, LeftHand = 0, RightHand, LeftFoot, RightFoot, Waist, TrackersCount };

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CSampleControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
	CSampleControllerDriver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		m_sSerialNumber = "CTRL_1234";
		m_sModelNumber = "HybridController";
		m_type = TrackerType::Undefined;
		m_handtrackingStarted = false;
		m_calibrationDone = false;
	}

	virtual ~CSampleControllerDriver()
	{
	}

	void SetControllerType(TrackerType type)
	{
		m_type = type;
		m_sSerialNumber = "HybridController_" + std::to_string(static_cast<int>(type));
	}

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());

		if (m_type == TrackerType::LeftHand || m_type == TrackerType::RightHand) {
			uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
			vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);

			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_controller");

			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");

			//vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{HybridDriver}/rendermodels/right_hand_test");

			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_Controller);

			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{HybridDriver}/input/mycontroller_profile.json");

			// create all the input components
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &m_compA);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &m_compB);
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/c/click", &m_compC);

			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "VR Controller");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{HybridDriver}/rendermodels/right_hand_test");
			if(m_type == TrackerType::LeftHand)
				vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/left", "/skeleton/hand/left", "/pose/raw", EVRSkeletalTrackingLevel::VRSkeletalTracking_Full, NULL, 31, &m_leftHandHandler);
			else 
				vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(m_ulPropertyContainer, "/input/skeleton/right", "/skeleton/hand/right", "/pose/raw", EVRSkeletalTrackingLevel::VRSkeletalTracking_Full, NULL, 31, &m_rightHandHandler);
		}
		else {
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "arrow");
		}
		
		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);


		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	bool CalibrateJointPositions()
	{
		float kinectHeadPos[3] = { 0 };
		bool isDataAvailable;

		kinectHeadPos[0] = g_SharedBuf->bodyMsg.HeadPos.X;
		kinectHeadPos[1] = g_SharedBuf->bodyMsg.HeadPos.Y;
		kinectHeadPos[2] = g_SharedBuf->bodyMsg.HeadPos.Z;

		TrackedDevicePose_t hmd_tracker;
		VRServerDriverHost()->GetRawTrackedDevicePoses(0, &hmd_tracker, 1);

		// Get vec3 from matrix34
		vr::HmdVector3_t hmdPos;
		hmdPos.v[0] = hmd_tracker.mDeviceToAbsoluteTracking.m[0][3];
		hmdPos.v[1] = hmd_tracker.mDeviceToAbsoluteTracking.m[1][3];
		hmdPos.v[2] = hmd_tracker.mDeviceToAbsoluteTracking.m[2][3];

		// if either of kinect/lighthouse is not synced, just bail out
		if ((hmdPos.v[0] == 0.0 && hmdPos.v[1] == 0.0 && hmdPos.v[2] == 0.0) || (kinectHeadPos[0] == 0.0 && kinectHeadPos[1] == 0.0 && kinectHeadPos[2] == 0.0))
			return false;

	/*	DriverLog("Calibration data:\n"\
			"\tLighthouse position: (%f, %f, %f)\n"\
			"\tKinect position: (%f, %f, %f)",
			hmdPos.v[0], hmdPos.v[1], hmdPos.v[2],
			kinectHeadPos[0], kinectHeadPos[1], kinectHeadPos[2]);*/

		for (int i = 0; i < 3; i++)
			m_calibrationPos[i] = hmdPos.v[i] - kinectHeadPos[i];

		//DriverLog("Computed calibration vector: (%f, %f, %f)", m_calibrationPos[0], m_calibrationPos[1], m_calibrationPos[2]);

		m_calibrationDone = true;
		return m_calibrationDone;
	}

	DriverPose_t GetHandPose() {
		DriverPose_t pose = { 0 };
		HandEventMsg_t* msg = &g_SharedBuf->handMsg;

		if (msg->state != (int)HandTrackingState::Initialized) //TODO: Also check if we can see the hand we are looking for
		{
			pose.poseIsValid = false;
			pose.result = TrackingResult_Calibrating_InProgress;
			return pose;
		}

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		TrackedDevicePose_t hmd_tracker;
		VRServerDriverHost()->GetRawTrackedDevicePoses(0, &hmd_tracker, 1);

		vr::HmdVector3d_t handPos;
		handPos.v[0] = m_type == TrackerType::LeftHand ? msg->LeftHandPos.X : msg->RightHandPos.X;
		handPos.v[1] = m_type == TrackerType::LeftHand ? msg->LeftHandPos.Y : msg->RightHandPos.Y;
		handPos.v[2] = -(m_type == TrackerType::LeftHand ? msg->LeftHandPos.Z : msg->RightHandPos.Z);
		HmdQuaternion_t hmdQuaternion = GetHMDRotation(hmd_tracker.mDeviceToAbsoluteTracking);
		handPos = MultiplyByQuaternion(hmdQuaternion, handPos);

		// Get vec3 from matrix34
		vr::HmdVector3_t hmdPos;
		hmdPos.v[0] = hmd_tracker.mDeviceToAbsoluteTracking.m[0][3];
		hmdPos.v[1] = hmd_tracker.mDeviceToAbsoluteTracking.m[1][3];
		hmdPos.v[2] = hmd_tracker.mDeviceToAbsoluteTracking.m[2][3];

		pose.vecPosition[0] = hmdPos.v[0] + static_cast<double>(handPos.v[0]);
		pose.vecPosition[1] = hmdPos.v[1] + static_cast<double>(handPos.v[1]);
		pose.vecPosition[2] = hmdPos.v[2] + static_cast<double>(handPos.v[2]);

		pose.qRotation = hmdQuaternion;
		return pose;

	}

	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };

		if (g_SharedBuf == NULL && !initSharedMemory()) {
			pose.poseIsValid = false;
			pose.result = TrackingResult_Calibrating_InProgress;
			return pose;
		}
		HandEventMsg_t* handMsg = &g_SharedBuf->handMsg;
		if (handMsg->state == (int)HandTrackingState::Initialized && (m_type == TrackerType::LeftHand || m_type == TrackerType::RightHand)) {
			if ((m_type == TrackerType::LeftHand && handMsg->leftHandDetected) || (m_type == TrackerType::RightHand && handMsg->rightHandDetected)) {
				DriverPose_t handPose = GetHandPose();
				if (handPose.poseIsValid)
						return handPose;
			}
			
		}

		//if (!m_calibrationDone && !CalibrateJointPositions())
		if (!CalibrateJointPositions())
		{
			pose.poseIsValid = false;
			pose.result = TrackingResult_Calibrating_InProgress;
			return pose;
		}

		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		// Retrieve actual joint positions from Kinect
		BodyEventMsg_t* msg = &g_SharedBuf->bodyMsg;

		// Estimate joint positions in lighthouse coordinates
		switch (m_type) {
		case TrackerType::LeftFoot:
			pose.vecPosition[0] = static_cast<double>(msg->FootLeftPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg->FootLeftPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg->FootLeftPos.Z) + m_calibrationPos[2];
			//memcpy(&pose.qRotation, &msg->FootLeftRot, sizeof(HmdQuaternion_t));
			break;
		case TrackerType::RightFoot:
			pose.vecPosition[0] = static_cast<double>(msg->FootRightPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg->FootRightPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg->FootRightPos.Z) + m_calibrationPos[2];
			//memcpy(&pose.qRotation, &msg->FootRightRot, sizeof(HmdQuaternion_t));
			break;
		case TrackerType::LeftHand:
			pose.vecPosition[0] = static_cast<double>(msg->HandLeftPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg->HandLeftPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg->HandLeftPos.Z) + m_calibrationPos[2];
			//memcpy(&pose.qRotation, &msg->HandLeftRot, sizeof(HmdQuaternion_t));
			break;
		case TrackerType::RightHand:
			pose.vecPosition[0] = static_cast<double>(msg->HandRightPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg->HandRightPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg->HandRightPos.Z) + m_calibrationPos[2];
			//memcpy(&pose.qRotation, &msg->HandRightRot, sizeof(HmdQuaternion_t));
			break;
		case TrackerType::Waist:
			pose.vecPosition[0] = static_cast<double>(msg->WaistPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg->WaistPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg->WaistPos.Z) + m_calibrationPos[2];
			//memcpy(&pose.qRotation, &msg->WaistRot, sizeof(HmdQuaternion_t));
			break;
		default:
			pose.poseIsValid = false;
			return pose;
		}

		TrackedDevicePose_t hmd_tracker;
		VRServerDriverHost()->GetRawTrackedDevicePoses(0, &hmd_tracker, 1);

		HmdQuaternion_t hmdQuaternion = GetHMDRotation(hmd_tracker.mDeviceToAbsoluteTracking);
		pose.qRotation = hmdQuaternion;

		/*
		double cyaw = 0, cpitch = 0, croll = 0; // TODO: Get rotation values

		//Convert yaw, pitch, roll to quaternion
		double ct0, ct1, ct2, ct3, ct4, ct5;
		ct0 = cos(cyaw * 0.5);
		ct1 = sin(cyaw * 0.5);
		ct2 = cos(croll * 0.5);
		ct3 = sin(croll * 0.5);
		ct4 = cos(cpitch * 0.5);
		ct5 = sin(cpitch * 0.5);

		//Set controller rotation
		pose.qRotation.w = ct0 * ct2 * ct4 + ct1 * ct3 * ct5;
		pose.qRotation.x = ct0 * ct3 * ct4 - ct1 * ct2 * ct5;
		pose.qRotation.y = ct0 * ct2 * ct5 + ct1 * ct3 * ct4;
		pose.qRotation.z = ct1 * ct2 * ct4 - ct0 * ct3 * ct5;
		*/

		return pose;
	}

	void RunFrame()
	{
#if defined( _WINDOWS )
		// Your driver would read whatever hardware state is associated with its input components and pass that
		// in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
		// state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.

		/*vr::VRDriverInput()->UpdateBooleanComponent(m_compA, (0x8000 & GetAsyncKeyState('A')) != 0, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(m_compB, (0x8000 & GetAsyncKeyState('B')) != 0, 0);
		vr::VRDriverInput()->UpdateBooleanComponent(m_compC, (0x8000 & GetAsyncKeyState('C')) != 0, 0);*/

		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
		}

		if (g_SharedBuf != NULL && (m_type == TrackerType::LeftHand || m_type == TrackerType::RightHand)) {
			HandEventMsg_t* msg = &g_SharedBuf->handMsg;
			if (msg->state == (int)HandTrackingState::Initialized)
			{
				if (m_type == TrackerType::LeftHand) {
					vr::VRDriverInput()->UpdateBooleanComponent(m_compA, msg->leftHandGesture == GestureTypeOK, 0);
					vr::VRDriverInput()->UpdateBooleanComponent(m_compB, msg->leftHandGesture == GestureTypeVictory, 0);
					vr::VRDriverInput()->UpdateBooleanComponent(m_compC, msg->leftHandGesture == GestureTypeFist, 0);
					vr::EVRInputError err = vr::VRDriverInput()->UpdateSkeletonComponent(m_leftHandHandler, vr::VRSkeletalMotionRange_WithController, msg->bonesLeftHand, 31);
					if (err != vr::VRInputError_None)
					{
						// Handle failure case
						DriverLog("UpdateSkeletonComponent failed.  Error: %i\n", err);
					}
					err = vr::VRDriverInput()->UpdateSkeletonComponent(m_leftHandHandler, vr::VRSkeletalMotionRange_WithoutController, msg->bonesLeftHand, 31);
					if (err != vr::VRInputError_None)
					{
						// Handle failure case
						DriverLog("UpdateSkeletonComponent failed.  Error: %i\n", err);
					}
				}
				else if (m_type == TrackerType::RightHand) {
					vr::VRDriverInput()->UpdateBooleanComponent(m_compA, msg->rightHandGesture == GestureTypeOK, 0);
					vr::VRDriverInput()->UpdateBooleanComponent(m_compB, msg->rightHandGesture == GestureTypeVictory, 0);
					vr::VRDriverInput()->UpdateBooleanComponent(m_compC, msg->rightHandGesture == GestureTypeFist, 0);
					vr::EVRInputError err = vr::VRDriverInput()->UpdateSkeletonComponent(m_rightHandHandler, vr::VRSkeletalMotionRange_WithController, msg->bonesRightHand, 31);
					if (err != vr::VRInputError_None)
					{
						// Handle failure case
						DriverLog("UpdateSkeletonComponent failed.  Error: %i\n", err);
					}
					err = vr::VRDriverInput()->UpdateSkeletonComponent(m_rightHandHandler, vr::VRSkeletalMotionRange_WithoutController, msg->bonesRightHand, 31);
					if (err != vr::VRInputError_None)
					{
						// Handle failure case
						DriverLog("UpdateSkeletonComponent failed.  Error: %i\n", err);
					}
				}

			}
		}

		/* TODO: Update skeleton components */
#endif
	}

	void ProcessEvent( const vr::VREvent_t & vrEvent )
	{
		switch ( vrEvent.eventType )
		{
		case vr::VREvent_Input_HapticVibration:
		{
			if ( vrEvent.data.hapticVibration.componentHandle == m_compHaptic )
			{
				// This is where you would send a signal to your hardware to trigger actual haptic feedback
				DriverLog( "BUZZ!\n" );
			}
		}
		break;
		}
	}

	std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	vr::VRInputComponentHandle_t m_compA;
	vr::VRInputComponentHandle_t m_compB;
	vr::VRInputComponentHandle_t m_compC;
	vr::VRInputComponentHandle_t m_compHaptic;

	vr::VRInputComponentHandle_t m_leftHandHandler;
	vr::VRInputComponentHandle_t m_rightHandHandler;

	std::string m_sSerialNumber;
	std::string m_sModelNumber;
	
	TrackerType m_type;

	bool m_calibrationDone;
	float m_calibrationPos[3];

	bool m_handtrackingStarted;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_Sample: public IServerTrackedDeviceProvider
{
public:
	~CServerDriver_Sample();
	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() ;
	virtual bool ShouldBlockStandbyMode()  { return false; }
	virtual void EnterStandby()  {}
	virtual void LeaveStandby()  {}

private:
	std::vector<CSampleControllerDriver*> m_pTrackers;
};

CServerDriver_Sample::~CServerDriver_Sample() {
	DriverLog("Entered Driver Destructor");
}

CServerDriver_Sample g_serverDriverNull;

EVRInitError CServerDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	DriverLog("Adding %d virtual trackers", TrackerType::TrackersCount);

	for (int i = 0; i < static_cast<int>(TrackerType::TrackersCount); i++) {
		/*if (static_cast<TrackerType>(i) == TrackerType::RightFoot || static_cast<TrackerType>(i) == TrackerType::LeftFoot)
			continue;*/
		CSampleControllerDriver* tracker = new CSampleControllerDriver();
		tracker->SetControllerType(static_cast<TrackerType>(i));

		vr::VRServerDriverHost()->TrackedDeviceAdded(tracker->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, tracker);
		m_pTrackers.push_back(tracker);
	}

	return VRInitError_None;
}

void CServerDriver_Sample::Cleanup()
{
	DriverLog("Entered Driver cleanup");

	for (auto it = std::begin(m_pTrackers); it != std::end(m_pTrackers); ++it) {
		(*it)->~CSampleControllerDriver();
	}
	m_pTrackers.clear();
	//CleanupDriverLog();
}


void CServerDriver_Sample::RunFrame()
{
	for (auto it = std::begin(m_pTrackers); it != std::end(m_pTrackers); ++it) {
		(*it)->RunFrame();
	}

	vr::VREvent_t vrEvent;
	while ( vr::VRServerDriverHost()->PollNextEvent( &vrEvent, sizeof( vrEvent ) ) )
	{
		for (auto it = std::begin(m_pTrackers); it != std::end(m_pTrackers); ++it) {
				(*it)->ProcessEvent(vrEvent);
			
		}
	}
}


//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory( const char *pInterfaceName, int *pReturnCode )
{
	if( 0 == strcmp( IServerTrackedDeviceProvider_Version, pInterfaceName ) )
	{
		return &g_serverDriverNull;
	}

	if( pReturnCode )
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}
