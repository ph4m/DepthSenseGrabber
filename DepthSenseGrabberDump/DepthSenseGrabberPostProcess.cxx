// DepthSenseGrabber
// http://github.com/ph4m

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <vector>
#include <exception>

#include "DepthSenseGrabberPostProcess.hxx"
#include "../DepthSenseGrabberCore/DepthSenseGrabberCore.hxx"
#include "../shared/ConversionTools.hxx"
#include "../shared/AcquisitionParameters.hxx"

#include "Frame.hxx"



int framerateDepth = 60;
int framerateColor = 30;


/**********************************************************
 * Filter out inaccurate measurements with confidence map *
 * Fix finger doubling in color map                       *
 **********************************************************/

#define DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP
#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
const uint16_t confidenceThreshold = 100;

#endif // DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP


/*******************************************************
 * Spatial smoothing with Gaussian blur                *
 * Could be improved by only filtering up to the edges *
 *******************************************************/

#define DEPTHSENSEGRABBER_SMOOTH_SPATIAL

int kernel_length = 3;

/*******************************************
 * Temporal smoothing with low-pass filter *
 * http://www.exstrom.com/journal/sigproc/ *
 *******************************************/

#define DEPTHSENSEGRABBER_SMOOTH_TEMPORAL

uint16_t maxDeltaDepth = 50;

const int filterOrder = 2;
const int filterSize = filterOrder/2;
const int filterSamplingFreq = framerateDepth;
const int filterCornerFreq = 15;

int widthDepth, heightDepth;
int widthColor, heightColor, nPixelsColorAcq;


float* filterA;
float* filterD1;
float* filterD2;

float* w0All;
float* w1All;
float* w2All;

void initFilterWeights(float* A, float* d1, float* d2, int n, int s, int f) {
    float a = tan(M_PI*f/s);
    float a2 = a*a;
    for (int i = 0; i < n; i++) {
        float r = sin(M_PI*(2.0*i+1.0)/(4.0*n));
        float t = a2 + 2.0*a*r + 1.0;
        A[i] = a2/t;
        d1[i] = 2.0*(1-a2)/t;
        d2[i] = -(a2 - 2.0*a*r + 1.0)/t;
    }
}

uint16_t filterNew(uint16_t sample, float* w0, float* w1, float* w2, int n, float* A, float* d1, float* d2) {
    float x = static_cast<float>(sample);
    for(int i=0; i < n; ++i){
        w0[i] = d1[i]*w1[i] + d2[i]*w2[i] + x;
        x = A[i]*(w0[i] + 2.0*w1[i] + w2[i]);
        w2[i] = w1[i];
        w1[i] = w0[i];
    }
    uint16_t filteredVal = static_cast<uint16_t>(x);
    bool isInRange = abs(filteredVal - sample) < maxDeltaDepth;
    if (not isInRange) return sample;
    return static_cast<uint16_t>(x);
}



int main(int argc, char* argv[])
{
    widthDepth = FORMAT_QVGA_WIDTH;
    heightDepth = FORMAT_QVGA_HEIGHT;

    filterA = new float[filterSize];
    filterD1 = new float[filterSize];
    filterD2 = new float[filterSize];

    w0All = new float[heightDepth*widthDepth*filterSize];
    w1All = new float[heightDepth*widthDepth*filterSize];
    w2All = new float[heightDepth*widthDepth*filterSize];


    /*
    bool interpolateDepthFlag = 0;

    bool saveColorAcqFlag   = 1;
    bool saveDepthAcqFlag   = 0;
    bool saveColorSyncFlag  = 0;
    bool saveDepthSyncFlag  = 0;
    bool saveConfidenceFlag = 0;

    bool buildColorSyncFlag = saveColorSyncFlag;
    bool buildDepthSyncFlag = saveDepthSyncFlag;
    bool buildConfidenceFlag = saveConfidenceFlag;

    int flagColorFormat = FORMAT_WXGA_ID; // VGA, WXGA or NHD

    int widthColor, heightColor;
    switch (flagColorFormat) {
        case FORMAT_VGA_ID:
            widthColor = FORMAT_VGA_WIDTH;
            heightColor = FORMAT_VGA_HEIGHT;
            break;
        case FORMAT_WXGA_ID:
            widthColor = FORMAT_WXGA_WIDTH;
            heightColor = FORMAT_WXGA_HEIGHT;
            break;
        case FORMAT_NHD_ID:
            widthColor = FORMAT_NHD_WIDTH;
            heightColor = FORMAT_NHD_HEIGHT;
            break;
        default:
            printf("Unknown flagColorFormat");
            exit(EXIT_FAILURE);
    }

    int widthDepthAcq, heightDepthAcq;
    if (interpolateDepthFlag) {
        widthDepthAcq = FORMAT_VGA_WIDTH;
        heightDepthAcq = FORMAT_VGA_HEIGHT;
    } else {
        widthDepthAcq = FORMAT_QVGA_WIDTH;
        heightDepthAcq = FORMAT_QVGA_HEIGHT;
    }


    char fileNameColorAcq[50];
    char fileNameDepthAcq[50];
    char fileNameColorSync[50];
    char fileNameDepthSync[50];
    char fileNameConfidence[50];

    char baseNameColorAcq[20] = "colorFrame_0_";
    char baseNameDepthAcq[20] = "depthAcqFrame_0_";
    char baseNameColorSync[20] = "colorSyncFrame_0_";
    char baseNameDepthSync[20] = "depthFrame_0_";
    char baseNameConfidence[30] = "depthConfidenceFrame_0_";

    start_capture(flagColorFormat,
                  interpolateDepthFlag,
                  buildColorSyncFlag, buildDepthSyncFlag, buildConfidenceFlag);

    uint16_t* pixelsDepthAcq;
    uint8_t* pixelsColorSync;
    uint8_t* pixelsColorAcq = getPixelsColorsAcq();
    uint16_t* pixelsDepthSync = getPixelsDepthSync();
    uint16_t* pixelsConfidenceQVGA = getPixelsConfidenceQVGA();
    if (interpolateDepthFlag) {
        pixelsDepthAcq = getPixelsDepthAcqVGA();
        pixelsColorSync = getPixelsColorSyncVGA();
    } else {
        pixelsDepthAcq = getPixelsDepthAcqQVGA();
        pixelsColorSync = getPixelsColorSyncQVGA();
    }

    int frameCountPrevious = -1;
    while (true) {
        int frameCount = getFrameCount();
        int timeStamp = getTimeStamp();
        if (frameCount > frameCountPrevious) {
            frameCountPrevious = frameCount;
            printf("%d\n", frameCount);

            if (saveDepthAcqFlag) {
                sprintf(fileNameDepthAcq,"%s%05u.pnm",baseNameDepthAcq,frameCount);
                saveDepthFramePNM(fileNameDepthAcq, pixelsDepthAcq, widthDepthAcq, heightDepthAcq, timeStamp);
            }
            if (saveColorAcqFlag) {
                sprintf(fileNameColorAcq,"%s%05u.pnm",baseNameColorAcq,frameCount);
                saveColorFramePNM(fileNameColorAcq, pixelsColorAcq, widthColor, heightColor, timeStamp);
            }
            if (saveDepthSyncFlag) {
                sprintf(fileNameDepthSync,"%s%05u.pnm",baseNameDepthSync,frameCount);
                saveDepthFramePNM(fileNameDepthSync, pixelsDepthSync, widthColor, heightColor, timeStamp);
            }
            if (saveColorSyncFlag) {
                sprintf(fileNameColorSync,"%s%05u.pnm",baseNameColorSync,frameCount);
                saveColorFramePNM(fileNameColorSync, pixelsColorSync, widthDepthAcq, heightDepthAcq, timeStamp);
            }
            if (saveConfidenceFlag) {
                sprintf(fileNameConfidence,"%s%05u.pnm",baseNameConfidence,frameCount);
                saveDepthFramePNM(fileNameConfidence, pixelsConfidenceQVGA, FORMAT_QVGA_WIDTH, FORMAT_QVGA_HEIGHT, timeStamp);
            }
        }
    }
    */


    return 0;
}
