//------------------------------------------------------------------------------
// <copyright file="BodyBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <thread>

#include "Bridge.h"
#include "stdafx.h"

class CBodyTracking
{
    static const int        cDepthWidth = 512;
    static const int        cDepthHeight = 424;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    CBodyTracking();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CBodyTracking();

    /// <summary>
    /// Set pointer to shm which will be used to update body information
    /// <param name="pMsg">Pointer to SHM</param>
    /// </summary>
    void                    SetEventMsgBuffer(BodyEventMsg_t* pMsg);

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();

    /// <summary>
    /// Handle new body data
    /// <param name="nTime">timestamp of frame</param>
    /// <param name="nBodyCount">body data count</param>
    /// <param name="ppBodies">body data in frame</param>
    /// <param name="msg">Out message to be sent over named pipe to the main component</param>
    /// <returns>TRUE if body is found, FALSE otherwise</returns>
    /// </summary>
    bool                    ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies, BodyEventMsg_t* pMsg);

private:

    void Update_Internal();

    HWND                    m_hWnd;
    INT64                   m_nStartTime;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

    // Current Kinect
    IKinectSensor* m_pKinectSensor;
    ICoordinateMapper* m_pCoordinateMapper;

    // Body reader
    IBodyFrameReader* m_pBodyFrameReader;
    BodyEventMsg_t *m_pMsg;

    std::thread* m_thread;
    bool m_stop;
};

