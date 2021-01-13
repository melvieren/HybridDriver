//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>

#if defined( _WINDOWS )
#include <windows.h>
#endif

#include "BodyTracking.h"
#include "Bridge.h"

#include "HandTracking.h"

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

inline HmdQuaternion_t HmdQuaternion_Init( double w, double x, double y, double z )
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
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

CBodyTracking g_bodyTracking;

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

enum class TrackerType { Undefined = 0, LeftHand, RightHand, LeftFoot, RightFoot, Waist };

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
	}

	virtual ~CSampleControllerDriver()
	{
	}

	void SetControllerType(TrackerType type)
	{
		m_type = type;
		m_sSerialNumber = "HybridController_" + std::to_string(static_cast<int>(type));
	}

	virtual EVRInitError Activate( vr::TrackedDeviceIndex_t unObjectId )
	{
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "locator_one_sided");
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);

		vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );

		uint64_t supportedButtons = 0xFFFFFFFFFFFFFFFFULL;
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_SupportedButtons_Uint64, supportedButtons);

		if (m_type == TrackerType::RightHand)
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);
		else if (m_type == TrackerType::LeftHand)
			vr::VRProperties()->SetInt32Property( m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_LeftHand );
		else
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);

		// this file tells the UI what to show the user for binding this controller as well as what default bindings should
		// be for legacy or other apps
		vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_InputProfilePath_String, "{sample}/input/mycontroller_profile.json" );

		// create all the input components
		vr::VRDriverInput()->CreateBooleanComponent( m_ulPropertyContainer, "/input/a/click", &m_compA );
		vr::VRDriverInput()->CreateBooleanComponent( m_ulPropertyContainer, "/input/b/click", &m_compB );
		vr::VRDriverInput()->CreateBooleanComponent( m_ulPropertyContainer, "/input/c/click", &m_compC );

		// create our haptic component
		vr::VRDriverInput()->CreateHapticComponent( m_ulPropertyContainer, "/output/haptic", &m_compHaptic );

		/* TODO: Create skeleton components */


		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void *GetComponent( const char *pchComponentNameAndVersion )
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest( const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize )
	{
		if ( unResponseBufferSize >= 1 )
			pchResponseBuffer[0] = 0;
	}

	bool CalibrateJointPositions()
	{
		float kinectHeadPos[3] = { 0 };
		bool isDataAvailable;

		g_bodyTracking.GetHeadPosition(kinectHeadPos, &isDataAvailable);
		if (!isDataAvailable) {
			return false;
		}

		TrackedDevicePose_t hmd_tracker;
		VRServerDriverHost()->GetRawTrackedDevicePoses(0, &hmd_tracker, 1);

		// Get vec3 from matrix34
		vr::HmdVector3_t hmdPos;
		hmdPos.v[0] = hmd_tracker.mDeviceToAbsoluteTracking.m[0][3];
		hmdPos.v[1] = hmd_tracker.mDeviceToAbsoluteTracking.m[1][3];
		hmdPos.v[2] = hmd_tracker.mDeviceToAbsoluteTracking.m[2][3];
		
		DriverLog("Calibration data:\n"\
			"\tLighthouse position: (%f, %f, %f)\n"\
			"\tKinect position: (%f, %f, %f)",
			hmdPos.v[0], hmdPos.v[1], hmdPos.v[2],
			kinectHeadPos[0], kinectHeadPos[1], kinectHeadPos[2]);

		for (int i = 0; i < 3; i++)
			m_calibrationPos[i] = hmdPos.v[i] - kinectHeadPos[i];

		DriverLog("Computed calibration vector: (%f, %f, %f)", m_calibrationPos[0], m_calibrationPos[1], m_calibrationPos[2]);

		m_calibrationDone = true;
		return m_calibrationDone;
	}

	virtual DriverPose_t GetPose()
	{
		DriverPose_t pose = { 0 };

		if (!m_calibrationDone && !CalibrateJointPositions())
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
		BodyEventMsg_t msg;
		bool isDataAvailable;

		g_bodyTracking.Update(&msg, &isDataAvailable);
		if (!isDataAvailable) {
			pose.poseIsValid = false;
			return pose;
		}

		// Estimate joint positions in lighthouse coordinates
		double x, y, z;
		switch (m_type) {
		case TrackerType::LeftFoot:
			pose.vecPosition[0] = static_cast<double>(msg.FootLeftPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg.FootLeftPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg.FootLeftPos.Z) + m_calibrationPos[2];
			break;
		case TrackerType::RightFoot:
			pose.vecPosition[0] = static_cast<double>(msg.FootRightPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg.FootRightPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg.FootRightPos.Z) + m_calibrationPos[2];
			break;
		case TrackerType::LeftHand:
			pose.vecPosition[0] = static_cast<double>(msg.HandLeftPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg.HandLeftPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg.HandLeftPos.Z) + m_calibrationPos[2];
			break;
		case TrackerType::RightHand:
			pose.vecPosition[0] = static_cast<double>(msg.HandRightPos.X) + m_calibrationPos[0];
			pose.vecPosition[1] = static_cast<double>(msg.HandRightPos.Y) + m_calibrationPos[1];
			pose.vecPosition[2] = static_cast<double>(msg.HandRightPos.Z) + m_calibrationPos[2];
			break;
		default:
			pose.poseIsValid = false;
			return pose;
		}

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

		return pose;
	}

	void RunFrame()
	{
#if defined( _WINDOWS )
		// Your driver would read whatever hardware state is associated with its input components and pass that
		// in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
		// state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.

		vr::VRDriverInput()->UpdateBooleanComponent( m_compA, (0x8000 & GetAsyncKeyState( 'A' )) != 0, 0 );
		vr::VRDriverInput()->UpdateBooleanComponent( m_compB, (0x8000 & GetAsyncKeyState( 'B' )) != 0, 0 );
		vr::VRDriverInput()->UpdateBooleanComponent( m_compC, (0x8000 & GetAsyncKeyState( 'C' )) != 0, 0 );

		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
		}

		if (m_type == TrackerType::LeftHand || m_type == TrackerType::RightHand) {
			int handNumber = 0;
			GestureResult* hands = getHandTrackingData(&handNumber);
			for (int i = 0; i < handNumber; i++) {
				if (hands[i].isLeft && m_type != TrackerType::LeftHand) continue;
				if (!hands[i].isLeft && m_type != TrackerType::RightHand) continue;
				// Do something
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

	std::string m_sSerialNumber;
	std::string m_sModelNumber;
	
	TrackerType m_type;

	bool m_calibrationDone = false;
	float m_calibrationPos[3];
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_Sample: public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() ;
	virtual bool ShouldBlockStandbyMode()  { return false; }
	virtual void EnterStandby()  {}
	virtual void LeaveStandby()  {}

private:
	std::vector<CSampleControllerDriver*> m_pTrackers;
	bool m_handtrackingStarted = false;
};

CServerDriver_Sample g_serverDriverNull;
#define N_TRACKERS 1

EVRInitError CServerDriver_Sample::Init( vr::IVRDriverContext *pDriverContext )
{
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	DriverLog("Initializing Kinect");
	g_bodyTracking.InitializeDefaultSensor();

	DriverLog("Adding %d virtual trackers", N_TRACKERS);

	for (int i = 0; i < N_TRACKERS; i++) {
		CSampleControllerDriver* tracker = new CSampleControllerDriver();
		tracker->SetControllerType(TrackerType::RightHand); // TODO: Add 1 of each type

		vr::VRServerDriverHost()->TrackedDeviceAdded(tracker->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, tracker);
		m_pTrackers.push_back(tracker);
	}

	return VRInitError_None;
}

void CServerDriver_Sample::Cleanup()
{
	CleanupDriverLog();

	for (auto it = std::begin(m_pTrackers); it != std::end(m_pTrackers); ++it) {
		delete *it;
	}

	m_pTrackers.clear();
}


void CServerDriver_Sample::RunFrame()
{
	if (!m_handtrackingStarted) {
		DriverLog("Initializing Camera Module of HandTracking");

		initHandTracking();

		m_handtrackingStarted = true;
	}

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
