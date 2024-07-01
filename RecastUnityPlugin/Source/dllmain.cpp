#include <windows.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "Recast.h"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
	
    return TRUE;
}

#define DllExport __declspec(dllexport)

extern "C"
{
	DllExport void CalcGridSize(const float* minBounds, const float* maxBounds, const float cellSize, int* sizeX, int* sizeZ)
	{
		rcCalcGridSize(minBounds, maxBounds, cellSize, sizeX, sizeZ);
	}
}