#include "camera_custom_fd.h"

void get_fd_CustomizeData(FD_Customize_PARA  *FDDataOut)
{    
    FDDataOut->FDThreadNum = 2;
    FDDataOut->FDThreshold = 32;
    FDDataOut->MajorFaceDecision = 1;
    FDDataOut->OTRatio = 1088;
    FDDataOut->SmoothLevel = 1;
    FDDataOut->FDSkipStep = 4;
    FDDataOut->FDRectify = 10;	//ALPS00943937
    FDDataOut->FDRefresh = 60;
    FDDataOut->SDThreshold = 69;
    FDDataOut->SDMainFaceMust = 1;
    FDDataOut->SDMaxSmileNum = 3;	//ALPS00943937
    FDDataOut->GSensor = 1;
}


