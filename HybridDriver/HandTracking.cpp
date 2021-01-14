#include <thread>

#include "driverlog.h"
#include "interface_gesture.hpp"

#include "HandTracking.h"


double clockToMilliseconds(clock_t ticks) {
	// units/(units/time) => time (seconds) * 1000 = milliseconds
	return (ticks / (double)CLOCKS_PER_SEC) * 1000.0;
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
	option.mode = GestureMode3DPoint;
	UseExternalTransform(true);
	GestureFailure result = StartGestureDetection(&option);
	if (result != GestureFailureNone) {
		DriverLog("Initilization of HandTracking failed");
		m_state = HandTrackingState::Error; // Todo: manage errors
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
		if (frameIndex < 0) {
			break;
		}
		else if (frameIndex == lastFrameIndex)
			continue;

		if (m_handCount > 0) m_isDataAvailable = true;
		lastFrameIndex = frameIndex;
		


	}
}
