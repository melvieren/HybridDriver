#include <thread>

#include "driverlog.h"
#include "interface_gesture.hpp"

GestureResult* handtrackingPoints = NULL;
static int g_handCount = 0;
std::thread *gestureRecognition;
static bool isDataAvailable = false;


void updateHandTracking() {
	
	int lastFrameIndex = -1;
	while(true) {
		int frameIndex = -1;
		g_handCount = GetGestureResult((const GestureResult**)&handtrackingPoints, &frameIndex);
		handtrackingPoints->isLeft;
		handtrackingPoints->points;
		if (frameIndex < 0) {
			break;
		}
		else if (frameIndex == lastFrameIndex)
			continue;

		if (g_handCount > 0) isDataAvailable = true;
		lastFrameIndex = frameIndex;

	}
}

GestureResult* getHandTrackingData(int * handCount) {
	isDataAvailable = false;
	*handCount = g_handCount;
	return handtrackingPoints;
}

int initHandTracking() {
	GestureOption option;
	GestureFailure result = StartGestureDetection(&option);
	if (result != GestureFailureNone) {
		DriverLog("Initilization of HandTracking failed");
		return result;
	}

	gestureRecognition = new std::thread(updateHandTracking);

	DriverLog("Initilization of HandTracking successful");

}
