#pragma once
#include <interface_gesture.hpp>

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

    HandTrackingState getState() { return m_state; };

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