// TrackingWorker.cpp : Ce fichier contient la fonction 'main'. L'exécution du programme commence et se termine à cet endroit.
//

#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <openvr.h>

#include "HandTracking.h"
#include "BodyTracking.h"

using namespace vr;

#define BUF_SIZE sizeof(BodyEventMsg_t) + sizeof(HandEventMsg_t)

TCHAR szName[] = TEXT("Global\HybridDriverSHM");

HANDLE g_hSharedMem = NULL;
void *g_SharedBuf = NULL;

int main()
{
    EVRInitError error = VRInitError_None;

    std::cout << "=== Tracking Worker Console ===" << std::endl << std::endl;

    // Specify SteamVR this is an Overlay so the process never gets killed
    VR_Init(&error, vr::VRApplication_Overlay);
    if (error != 0)
        std::cout << "Could not initialize worker process as an overlay: " << VR_GetVRInitErrorAsSymbol(error) << std::endl;

    std::cout << "[*] Creating a SHM of size " << BUF_SIZE << std::endl;

    g_hSharedMem = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, BUF_SIZE, szName);
    if (g_hSharedMem == NULL)
    {
        std::cout << "CreateFileMapping failed: " << GetLastError() << std::endl;
        return 1;
    }

    g_SharedBuf = MapViewOfFile(g_hSharedMem, FILE_MAP_ALL_ACCESS, 0, 0, BUF_SIZE);
    if (g_SharedBuf == NULL)
    {
        std::cout << "MapViewOfFile failed: " << GetLastError() << std::endl;
        CloseHandle(g_hSharedMem);
        return 1;
    }

    // Wait for SteamVR to be initialized
    Sleep(5000);

    std::cout << "[*] Initializing Workers" << std::endl;

    CHandTracking handTracking;
    CBodyTracking bodyTracking;

    handTracking.InitializeDefaultSensor();
    bodyTracking.InitializeDefaultSensor();

    while (true) Sleep(1000);
}
