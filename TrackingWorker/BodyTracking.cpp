//------------------------------------------------------------------------------
// <copyright file="BodyBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include <iostream>
#include <thread>

#include "stdafx.h"
#include <strsafe.h>

#include "BodyTracking.h"

static const float c_JointThickness = 3.0f;
static const float c_TrackedBoneThickness = 6.0f;
static const float c_InferredBoneThickness = 1.0f;
static const float c_HandSize = 30.0f;

/// <summary>
/// Constructor
/// </summary>
CBodyTracking::CBodyTracking() :
    m_hWnd(NULL),
    m_nStartTime(0),
    m_nLastCounter(0),
    m_nFramesSinceUpdate(0),
    m_fFreq(0),
    m_nNextStatusTime(0LL),
    m_pKinectSensor(NULL),
    m_pCoordinateMapper(NULL),
    m_pBodyFrameReader(NULL),
    m_Msg({ 0 }),
    m_thread(NULL),
    m_stop(false)
{
    LARGE_INTEGER qpf = { 0 };
    if (QueryPerformanceFrequency(&qpf))
    {
        m_fFreq = double(qpf.QuadPart);
    }
}


/// <summary>
/// Destructor
/// </summary>
CBodyTracking::~CBodyTracking()
{
    m_stop = true;
    m_thread->join();

    // done with body frame reader
    SafeRelease(m_pBodyFrameReader);

    // done with coordinate mapper
    SafeRelease(m_pCoordinateMapper);

    // close the Kinect Sensor
    if (m_pKinectSensor)
    {
        m_pKinectSensor->Close();
    }

    SafeRelease(m_pKinectSensor);
}

void CBodyTracking::GetHeadPosition(float headPosition[3], bool* pIsDataAvailable)
{
    *pIsDataAvailable = false;

    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;

    HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

    if (SUCCEEDED(hr))
    {
        INT64 nTime = 0;

        hr = pBodyFrame->get_RelativeTime(&nTime);

        IBody* ppBodies[BODY_COUNT] = { 0 };

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
        }

        if (SUCCEEDED(hr))
        {
            if (m_pCoordinateMapper)
            {
                for (int i = 0; i < BODY_COUNT; ++i)
                {
                    IBody* pBody = ppBodies[i];
                    if (pBody)
                    {
                        BOOLEAN bTracked = false;
                        HRESULT hr = pBody->get_IsTracked(&bTracked);

                        if (SUCCEEDED(hr) && bTracked)
                        {
                            Joint joints[JointType_Count];
                            hr = pBody->GetJoints(_countof(joints), joints);
                            if (SUCCEEDED(hr))
                            {
                                headPosition[0] = joints[JointType_Head].Position.X; // static_cast<double>(joints[JointType_Head].Position.X);
                                headPosition[1] = joints[JointType_Head].Position.Y; // static_cast<double>(joints[JointType_Head].Position.Y);
                                headPosition[2] = joints[JointType_Head].Position.Z; // static_cast<double>(joints[JointType_Head].Position.Z);

                                *pIsDataAvailable = true;
                                break;
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < _countof(ppBodies); ++i)
        {
            SafeRelease(ppBodies[i]);
        }
    }

    SafeRelease(pBodyFrame);
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyTracking::Update(BodyEventMsg_t** ppMsg)
{
    *ppMsg = &m_Msg;
}

/// <summary>
/// Main processing function
/// </summary>
void CBodyTracking::Update_Internal()
{
    if (!m_pBodyFrameReader)
    {
        return;
    }

    IBodyFrame* pBodyFrame = NULL;
    while (!m_stop) {
        HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

        if (SUCCEEDED(hr))
        {
            INT64 nTime = 0;

            hr = pBodyFrame->get_RelativeTime(&nTime);

            IBody* ppBodies[BODY_COUNT] = { 0 };

            if (SUCCEEDED(hr))
            {
                hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
            }

            if (SUCCEEDED(hr))
            {
                ProcessBody(nTime, BODY_COUNT, ppBodies, &m_Msg);
            }

            for (int i = 0; i < _countof(ppBodies); ++i)
            {
                SafeRelease(ppBodies[i]);
            }
        }

        SafeRelease(pBodyFrame);
    }
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CBodyTracking::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get coordinate mapper and the body reader
        IBodyFrameSource* pBodyFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
        }

        SafeRelease(pBodyFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        return E_FAIL;
    }

    m_thread = new std::thread(&CBodyTracking::Update_Internal, this);

    return hr;
}

/// <summary>
/// Handle new body data
/// <param name="nTime">timestamp of frame</param>
/// <param name="nBodyCount">body data count</param>
/// <param name="ppBodies">body data in frame</param>
/// <param name="msg">Out message to be sent over named pipe to the main component</param>
/// <returns>TRUE if body is found, FALSE otherwise</returns>
/// </summary>
bool CBodyTracking::ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies, BodyEventMsg_t* pMsg)
{

    if (m_pCoordinateMapper)
    {
        for (int i = 0; i < nBodyCount; ++i)
        {
            IBody* pBody = ppBodies[i];
            if (pBody)
            {
                BOOLEAN bTracked = false;
                HRESULT hr = pBody->get_IsTracked(&bTracked);

                if (SUCCEEDED(hr) && bTracked)
                {
                    Joint joints[JointType_Count];

                    HandState leftHandState = HandState_Unknown;
                    HandState rightHandState = HandState_Unknown;

                    pBody->get_HandLeftState(&leftHandState);
                    pBody->get_HandRightState(&rightHandState);

                    pMsg->HandLeftState = leftHandState;
                    pMsg->HandRightState = leftHandState;

                    hr = pBody->GetJoints(_countof(joints), joints);
                    if (SUCCEEDED(hr))
                    {
                        JointOrientation jointOrientations[JointType_Count];
                        pBody->GetJointOrientations(_countof(joints), jointOrientations);

                        /* Update positions */
                        pMsg->WaistPos.X = joints[JointType_SpineBase].Position.X;
                        pMsg->WaistPos.Y = joints[JointType_SpineBase].Position.Y;
                        pMsg->WaistPos.Z = joints[JointType_SpineBase].Position.Z;

                        pMsg->FootLeftPos.X = joints[JointType_FootLeft].Position.X;
                        pMsg->FootLeftPos.Y = joints[JointType_FootLeft].Position.Y;
                        pMsg->FootLeftPos.Z = joints[JointType_FootLeft].Position.Z;

                        pMsg->FootRightPos.X = joints[JointType_FootRight].Position.X;
                        pMsg->FootRightPos.Y = joints[JointType_FootRight].Position.Y;
                        pMsg->FootRightPos.Z = joints[JointType_FootRight].Position.Z;

                        pMsg->HandLeftPos.X = joints[JointType_HandLeft].Position.X;
                        pMsg->HandLeftPos.Y = joints[JointType_HandLeft].Position.Y;
                        pMsg->HandLeftPos.Z = joints[JointType_HandLeft].Position.Z;

                        pMsg->HandRightPos.X = joints[JointType_HandRight].Position.X;
                        pMsg->HandRightPos.Y = joints[JointType_HandRight].Position.Y;
                        pMsg->HandRightPos.Z = joints[JointType_HandRight].Position.Z;

                        pMsg->HeadPos.X = joints[JointType_Head].Position.X;
                        pMsg->HeadPos.Y = joints[JointType_Head].Position.Y;
                        pMsg->HeadPos.Z = joints[JointType_Head].Position.Z;

                        /* Update orientations */
                        pMsg->WaistRot.X = jointOrientations[JointType_SpineBase].Orientation.x;
                        pMsg->WaistRot.Y = jointOrientations[JointType_SpineBase].Orientation.y;
                        pMsg->WaistRot.Z = jointOrientations[JointType_SpineBase].Orientation.z;
                        pMsg->WaistRot.W = jointOrientations[JointType_SpineBase].Orientation.w;

                        /* Feet # Ankles */
                        pMsg->FootLeftRot.X = jointOrientations[JointType_AnkleLeft].Orientation.x;
                        pMsg->FootLeftRot.Y = jointOrientations[JointType_AnkleLeft].Orientation.y;
                        pMsg->FootLeftRot.Z = jointOrientations[JointType_AnkleLeft].Orientation.z;
                        pMsg->FootLeftRot.W = jointOrientations[JointType_AnkleLeft].Orientation.w;

                        pMsg->FootRightRot.X = jointOrientations[JointType_AnkleRight].Orientation.x;
                        pMsg->FootRightRot.Y = jointOrientations[JointType_AnkleRight].Orientation.y;
                        pMsg->FootRightRot.Z = jointOrientations[JointType_AnkleRight].Orientation.z;
                        pMsg->FootRightRot.W = jointOrientations[JointType_AnkleRight].Orientation.w;

                        /* Hands # Wrists */
                        pMsg->HandLeftRot.X = jointOrientations[JointType_WristLeft].Orientation.x;
                        pMsg->HandLeftRot.Y = jointOrientations[JointType_WristLeft].Orientation.y;
                        pMsg->HandLeftRot.Z = jointOrientations[JointType_WristLeft].Orientation.z;
                        pMsg->HandLeftRot.W = jointOrientations[JointType_WristLeft].Orientation.w;

                        pMsg->HandRightRot.X = jointOrientations[JointType_WristRight].Orientation.x;
                        pMsg->HandRightRot.Y = jointOrientations[JointType_WristRight].Orientation.y;
                        pMsg->HandRightRot.Z = jointOrientations[JointType_WristRight].Orientation.z;
                        pMsg->HandRightRot.W = jointOrientations[JointType_WristRight].Orientation.w;

                        /* Neck # Head */
                        pMsg->HeadRot.X = jointOrientations[JointType_Neck].Orientation.x;
                        pMsg->HeadRot.Y = jointOrientations[JointType_Neck].Orientation.y;
                        pMsg->HeadRot.Z = jointOrientations[JointType_Neck].Orientation.z;
                        pMsg->HeadRot.W = jointOrientations[JointType_Neck].Orientation.w;

                        return true;
                    }
                }
            }
        }
    }

    return false;
}
