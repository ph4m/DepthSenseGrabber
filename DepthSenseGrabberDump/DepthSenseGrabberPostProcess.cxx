// DepthSenseGrabber
// http://github.com/ph4m

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <exception>

#include "DepthSenseGrabberPostProcess.hxx"
#include "../DepthSenseGrabberCore/DepthSenseGrabberCore.hxx"
#include "../shared/ConversionTools.hxx"
#include "../shared/AcquisitionParameters.hxx"

#include "Frame.hxx"
#include <opencv2/opencv.hpp>



int framerateDepth = 60;
int framerateColor = 30;

const uint8_t noDepthRGB[3] = {255, 255, 255};
const uint16_t depthDefault = 0;
const uint16_t depthDeltaSync = 132; // DS325


/**********************************************************
 * Filter out inaccurate measurements with confidence map *
 * Fix finger doubling in color map                       *
 **********************************************************/

#define DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP
#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
const uint16_t confidenceThreshold = 50;

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
int colorWidth, colorHeight, nPixelsColorAcq;


float* filterA;
float* filterD1;
float* filterD2;

float* w0All;
float* w1All;
float* w2All;

void cvMatToArray(cv::Mat mat, uint16_t* depthArray,
        int depthWidth, int depthHeight) {
    for (int row = 0; row < depthHeight; row++) {
        for (int col = 0; col < depthWidth; col++) {
            depthArray[row*depthWidth+col] = mat.at<uint16_t>(cv::Point(col, row));
        }
    }
}

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
    if (!isInRange) return sample;
    return static_cast<uint16_t>(x);
}


void postProcess(Report reportColor, Report reportDepth) {
    int nFramesColor = reportColor.size();
    int nFramesDepth = reportDepth.size();
    for (Report::size_type iFrameDepth = 0; iFrameDepth < nFramesDepth; iFrameDepth++) {
        int indexFrameDepth   =  reportDepth[iFrameDepth].indexFrame;
        int timestamp         =  reportDepth[iFrameDepth].timestamp;
        int correspFrameColor = reportDepth[iFrameDepth].correspFrame;
        if (correspFrameColor >= nFramesColor) {
            cout << "Color frame " << correspFrameColor
                 << " corresponding to depth frame " << indexFrameDepth
                 << " is out of range" << endl;
            break;
        }
        string filenameFrameColor = FrameColor::formatFilenameFrame(correspFrameColor);
        FrameColor frameColor(filenameFrameColor);
        uint8_t* colorArray = frameColor.getRGB();
        int colorWidth = frameColor.getWidth();
        int colorHeight = frameColor.getHeight();

        string filenameFrameDepth = FrameDepth::formatFilenameFrame(indexFrameDepth);
        FrameDepth frameDepth(filenameFrameDepth);
        uint16_t* depthArray = frameDepth.getDepth();
        uint16_t* confidenceArray = frameDepth.getConfidence();
        float* uvArray = frameDepth.getUV();
        int depthWidth = frameDepth.getWidth();
        int depthHeight = frameDepth.getHeight();

        // Initialize color map corresponding to raw depth measurements
        //uint8_t colorSyncRGB[3 * depthWidth*depthHeight];
		uint8_t* colorSyncRGB = new uint8_t[3 * depthWidth*depthHeight];
        for (int iPixelDepth = 0; iPixelDepth < depthWidth*depthHeight; iPixelDepth++) {
            for (int iDim = 0; iDim < 3; iDim++) {
                colorSyncRGB[iPixelDepth*3 + iDim] = noDepthRGB[iDim];
            }
        }

        cv::Mat depthMatAcq = cv::Mat(depthHeight, depthWidth, CV_16UC1, cv::Scalar(0));
        cv::Mat uvMatAcq = cv::Mat(depthHeight, depthWidth, CV_32FC2, cv::Scalar(0));

        for (int rowDepth = 0; rowDepth < depthHeight; rowDepth++) {
            for (int colDepth = 0; colDepth < depthWidth; colDepth++) {
                int iPixelDepth = rowDepth*depthWidth + colDepth;
                uint16_t depthVal = depthArray[iPixelDepth];
                float uvValU = uvArray[2*iPixelDepth];
                float uvValV = uvArray[2*iPixelDepth+1];
                depthMatAcq.at<uint16_t>(cv::Point(colDepth, rowDepth)) = depthVal;
                uvMatAcq.at<cv::Vec2f>(cv::Point(colDepth, rowDepth))[0] = uvValU;
                uvMatAcq.at<cv::Vec2f>(cv::Point(colDepth, rowDepth))[1] = uvValV;
            }
        }

#if defined(DEPTHSENSEGRABBER_SMOOTH_SPATIAL)
        cv::GaussianBlur( depthMatAcq, depthMatAcq, cv::Size( kernel_length, kernel_length ), 0, 0 );
#endif // DEPTHSENSEGRABBER_SMOOTH_SPATIAL


        for (int rowDepth = 0; rowDepth < depthHeight; rowDepth++) {
            for (int colDepth = 0; colDepth < depthWidth; colDepth++) {

                int iPixelDepth = rowDepth*depthWidth + colDepth;
                //uint16_t depthVal = depthArray[iPixelDepth];
                uint16_t depthVal = depthMatAcq.at<uint16_t>(cv::Point(colDepth, rowDepth));
#if defined(DEPTHSENSEGRABBER_SMOOTH_TEMPORAL)
                float* w0 = w0All + iPixelDepth*filterSize;
                float* w1 = w1All + iPixelDepth*filterSize;
                float* w2 = w2All + iPixelDepth*filterSize;
                depthVal = filterNew(depthVal, w0, w1, w2, filterSize, filterA, filterD1, filterD2);
#endif // DEPTHSENSEGRABBER_SMOOTH_TEMPORAL

#if defined(DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP)
                uint16_t confidenceVal = confidenceArray[iPixelDepth];
                bool flagConfidence = (confidenceVal > confidenceThreshold);
#else
                bool flagConfidence = true;
#endif // DEPTHSENSEGRABBER_USE_CONFIDENCE_MAP

                // Fetch synchronized color from UV map and raw color
                float uvValU = uvMatAcq.at<cv::Vec2f>(cv::Point(colDepth, rowDepth))[0];
                float uvValV = uvMatAcq.at<cv::Vec2f>(cv::Point(colDepth, rowDepth))[1];
                int rowColor = round(uvValV * colorHeight); ///< -1??
                int colColor = round(uvValU * colorWidth); ///< -1??
                bool flagRange = (colColor < colorWidth) && (colColor >= 0) &&
                                 (rowColor < colorHeight) && (rowColor >= 0);

                if (flagConfidence && flagRange) {
                    int correspPixelColor = rowColor*colorWidth + colColor;
                    for (int iDim = 0; iDim < 3; iDim++) {
                        colorSyncRGB[3*iPixelDepth+iDim] = colorArray[3*correspPixelColor+iDim];
                    }
                }

                depthMatAcq.at<uint16_t>(cv::Point(colDepth, rowDepth)) = depthVal;

            }
        }

        cv::Mat depthMatInterpolated = cv::Mat(colorHeight, colorWidth, CV_16UC1, cv::Scalar(0));
        cv::Mat uvMatInterpolated = cv::Mat(colorHeight, colorWidth, CV_32FC2, cv::Scalar(0));
        cv::resize(depthMatAcq, depthMatInterpolated, depthMatInterpolated.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(uvMatAcq, uvMatInterpolated, uvMatInterpolated.size(), 0, 0, cv::INTER_LINEAR);

        bool* hasData = new bool[colorWidth*colorHeight];
        cv::Mat depthMatSync = cv::Mat(colorHeight, colorWidth, CV_16UC1, cv::Scalar(0));
        for (int rowColor = 0; rowColor < colorHeight; rowColor++) {
            for (int colColor = 0; colColor < colorWidth; colColor++) {
                depthMatSync.at<uint16_t>(cv::Point(colColor, rowColor)) = depthDefault;
                hasData[rowColor*colorWidth+colColor] = 0;
            }
        }
        for (int rowDepthInterpolated = 0; rowDepthInterpolated < colorHeight; rowDepthInterpolated++) {
            for (int colDepthInterpolated = 0; colDepthInterpolated < colorWidth; colDepthInterpolated++) {
                float uvValU = uvMatInterpolated.at<cv::Vec2f>(cv::Point(colDepthInterpolated, rowDepthInterpolated))[0];
                float uvValV = uvMatInterpolated.at<cv::Vec2f>(cv::Point(colDepthInterpolated, rowDepthInterpolated))[1];
                int rowColor = round(uvValV * colorHeight); ///< -1??
                int colColor = round(uvValU * colorWidth); ///< -1??
                bool flagRange = (colColor < colorWidth) && (colColor >= 0) &&
                                 (rowColor < colorHeight) && (rowColor >= 0);
                if (flagRange) {
                    uint16_t depthVal = depthMatInterpolated.at<uint16_t>(cv::Point(colDepthInterpolated, rowDepthInterpolated));
                    depthMatSync.at<uint16_t>(cv::Point(colColor, rowColor)) = depthVal;
                    hasData[rowColor*colorWidth+colColor] = 1;
                }
            }
        }


        uint16_t* pixelsDepthAcq = new uint16_t[depthWidth*depthHeight];
        cvMatToArray(depthMatAcq, pixelsDepthAcq, depthWidth, depthHeight);
        uint16_t* pixelsDepthSync = new uint16_t[colorWidth*colorHeight];
        cvMatToArray(depthMatSync, pixelsDepthSync, colorWidth, colorHeight);

        // Fill holes from vicinity
        int deltaPixelsIndAround[8] = {-colorWidth-1,-colorWidth,-colorWidth+1,-1,
                                       1,colorWidth-1,colorWidth,colorWidth+1};

        for (int currentRow = 1; currentRow < colorHeight-1; currentRow++) {
            for (int currentCol = 1; currentCol < colorWidth-1; currentCol++) {
                int currentPixelInd = currentRow*colorWidth+currentCol;
                int countValidAround = 0;
                uint16_t depthValidAround = 0;
                if (hasData[currentPixelInd] == 0) {
                    for (int indDeltaPixel = 0; indDeltaPixel < 8; indDeltaPixel++) {
                        if (hasData[currentPixelInd+deltaPixelsIndAround[indDeltaPixel]]) {
                            countValidAround++;
                            depthValidAround = depthValidAround + pixelsDepthSync[currentPixelInd+deltaPixelsIndAround[indDeltaPixel]];
                        }
                    }
                    if (countValidAround > 1) {
                        pixelsDepthSync[currentPixelInd] = depthValidAround / countValidAround;
                    }
                }
            }
        }

        // Save raw color
        string prefixColorAcq = "colorAcq";
        string filenameColorAcq = Frame::formatFilenamePNM(indexFrameDepth, prefixColorAcq);
        saveColorFramePNM(filenameColorAcq, colorArray,
                          colorWidth, colorHeight, timestamp);
        /*
        // Save synchronized color
        string prefixColorSync = "colorSync";
        string filenameColorSync = Frame::formatFilenamePNM(indexFrameDepth, prefixColorSync);
        saveColorFramePNM(filenameColorSync, colorSyncRGB,
                          depthWidth, depthHeight, timestamp);
        // Save raw depth
        string prefixDepthAcq = "depthAcq";
        string filenameDepthAcq = Frame::formatFilenamePNM(indexFrameDepth, prefixDepthAcq);
        saveDepthFramePNM(filenameDepthAcq, pixelsDepthAcq,
                          depthWidth, depthHeight, timestamp);
        */
        // Save synchronized depth
        string prefixDepthSync = "depthSync";
        string filenameDepthSync = Frame::formatFilenamePNM(indexFrameDepth, prefixDepthSync);
        saveDepthFramePNM(filenameDepthSync, pixelsDepthSync,
                          colorWidth, colorHeight, timestamp);

        delete[] pixelsDepthAcq;
		delete[] colorSyncRGB;
		delete[] hasData;

    }
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

    string filenameReportColor = FrameColor::formatFilenameReport();
    string filenameReportDepth = FrameDepth::formatFilenameReport();

    int nFramesDepth;
    Report reportColor;
    Report reportDepth;

    Frame::importReport(filenameReportColor, &reportColor);
    Frame::importReport(filenameReportDepth, &reportDepth);

    postProcess(reportColor, reportDepth);

    return 0;
}
