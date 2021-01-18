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

#define BUF_SIZE 256 //sizeof(BodyEventMsg_t) + sizeof(HandEventMsg_t)
TCHAR szName[] = TEXT("Global\HybridDriverSHM");

HANDLE hMapFile;
LPCTSTR pBuf;

void check_error(int line, EVRInitError error) { if (error != 0) printf("%d: error %s\n", line, VR_GetVRInitErrorAsSymbol(error)); }


int main()
{
    EVRInitError error;
    VR_Init(&error, vr::VRApplication_Overlay);

    check_error(__LINE__, error);

    VROverlayHandle_t handle;
    VROverlay()->CreateOverlay("image", "image", &handle); /* key has to be unique, name doesn't matter */
    VROverlay()->SetOverlayFromFile(handle, "/path/to/image.png");
    VROverlay()->SetOverlayWidthInMeters(handle, 3);
    VROverlay()->ShowOverlay(handle);

    vr::HmdMatrix34_t transform = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f, -2.0f
    };
    VROverlay()->SetOverlayTransformAbsolute(handle, TrackingUniverseStanding, &transform);

    /*std::cout << "Creating a SHM of size " << BUF_SIZE << std::endl;
    hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, BUF_SIZE, szName);

    if (hMapFile == NULL)
    {
        _tprintf(TEXT("Could not create file mapping object (%d).\n"),
            GetLastError());
        return 1;
    }
    pBuf = (LPTSTR)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, BUF_SIZE);

    if (pBuf == NULL)
    {
        _tprintf(TEXT("Could not map view of file (%d).\n"),
            GetLastError());
        CloseHandle(hMapFile);
        return 1;
    }*/
    Sleep(20000);
    CHandTracking handTracking;
    CBodyTracking bodyTrakcing;
    std::cout << "Starting Hand Tracking" << std::endl;
    handTracking.InitializeDefaultSensor();
    //bodyTrakcing.InitializeDefaultSensor();
    while (true) Sleep(1000);
}

// Exécuter le programme : Ctrl+F5 ou menu Déboguer > Exécuter sans débogage
// Déboguer le programme : F5 ou menu Déboguer > Démarrer le débogage

// Astuces pour bien démarrer : 
//   1. Utilisez la fenêtre Explorateur de solutions pour ajouter des fichiers et les gérer.
//   2. Utilisez la fenêtre Team Explorer pour vous connecter au contrôle de code source.
//   3. Utilisez la fenêtre Sortie pour voir la sortie de la génération et d'autres messages.
//   4. Utilisez la fenêtre Liste d'erreurs pour voir les erreurs.
//   5. Accédez à Projet > Ajouter un nouvel élément pour créer des fichiers de code, ou à Projet > Ajouter un élément existant pour ajouter des fichiers de code existants au projet.
//   6. Pour rouvrir ce projet plus tard, accédez à Fichier > Ouvrir > Projet et sélectionnez le fichier .sln.
