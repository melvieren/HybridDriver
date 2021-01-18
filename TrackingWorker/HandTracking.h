#pragma once

#include <thread>

#include <interface_gesture.hpp>
#include <openvr.h>

enum class HandTrackingState {
    Unitialized,
    Initializing,
    Initialized,
    Error
};


class CHandTracking
{

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CHandTracking();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CHandTracking();

    /// <summary>
    /// Initializes Vive Hand Tracker
    /// </summary>
    /// <returns>True on success, otherwise false</returns>
    void                 InitializeDefaultSensor();


    /// <summary>
    /// Retrieve Hand Recognition Data
    /// <param name="handCount">(out) set to the number of hand detected</param>
    /// </summary>
    /// <returns>True on success, otherwise false</returns>
    GestureResult* getHandTrackingData(int* handCount);

    /// <summary>
    /// Check if data is available
    /// </summary>
    /// <returns>True if new data, else false</returns>
    bool isDataAvailable();

    /// <summary>
    /// Return current state of the handtracking
    /// </summary>
    /// <returns>The current state of the handtracking library</returns>
    HandTrackingState getState() { return m_state; };

    /// <summary>
    /// Create and get bones from the points of a hand
    /// <param name="bones">(out) array of VRBoneTransform_t, the bones of the hand</param>
    /// <param name="handCount">(in) array of 63 coordinates that compose 21 points</param>
    /// </summary>
    /// <returns>True if new data, else false</returns>
    void getRightHandBones(vr::VRBoneTransform_t* bones, float points[]);

    /*vr::VRInputComponentHandle_t leftHandComponentHandler;
    vr::VRInputComponentHandle_t rightHandComponentHandler;*/
    std::thread* m_initThread;

private:
    void updateHandTracking();

    void initialize();

    int m_handCount;
    std::thread* m_gestureRecognition;

    bool m_isDataAvailable;
    GestureResult* m_handtrackingPoints;

    bool m_stop;

    HandTrackingState m_state;

};

enum PointNaming
{
    Base = 0,
    Thumb0,
    Thumb1,
    Thumb2,
    Thumb3,
    Index0,
    Index1,
    Index2,
    Index3,
    Middle0,
    Middle1,
    Middle2,
    Middle3,
    Ring0,
    Ring1,
    Ring2,
    Ring3,
    Pinky0,
    Pinky1,
    Pinky2,
    Pinky3
};

enum HandSkeletonBone
{
    eBone_Root = 0,
    eBone_Wrist,
    eBone_Thumb0,
    eBone_Thumb1,
    eBone_Thumb2,
    eBone_Thumb3,
    eBone_IndexFinger0,
    eBone_IndexFinger1,
    eBone_IndexFinger2,
    eBone_IndexFinger3,
    eBone_IndexFinger4,
    eBone_MiddleFinger0,
    eBone_MiddleFinger1,
    eBone_MiddleFinger2,
    eBone_MiddleFinger3,
    eBone_MiddleFinger4,
    eBone_RingFinger0,
    eBone_RingFinger1,
    eBone_RingFinger2,
    eBone_RingFinger3,
    eBone_RingFinger4,
    eBone_PinkyFinger0,
    eBone_PinkyFinger1,
    eBone_PinkyFinger2,
    eBone_PinkyFinger3,
    eBone_PinkyFinger4,
    eBone_Aux_Thumb,
    eBone_Aux_IndexFinger,
    eBone_Aux_MiddleFinger,
    eBone_Aux_RingFinger,
    eBone_Aux_PinkyFinger,
    eBone_Count
};