
#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <time.h>

#include <boost/thread.hpp>

#include <chrono>

#include <vector>
#include <exception>

#include "DepthSense.hxx"

#include "AcquisitionParameters.hxx"

#include "FrameColor.hxx"

using namespace DepthSense;
using namespace std;

FrameColor::FrameColor(int width, int height) {
    setWidth(width);
    setHeight(height);
    m_rgb = new uint8_t[3*width*height];
}

void FrameColor::setWidth(int width) {
    m_width = width;
}
void FrameColor::setHeight(int height) {
    m_height = height;
}
void FrameColor::setTimeStamp(int timeStamp) {
    m_timeStamp = timeStamp;
}
void FrameColor::setIndexFrameColor(int indexFrameColor) {
    m_indexFrameColor = indexFrameColor;
}
void FrameColor::setCorrespFrameDepth(int correspFrameDepth) {
    m_correspFrameDepth = correspFrameDepth;
}

void FrameColor::importColorMap(ColorNode::NewSampleReceivedData data) {
    for (int currentPixelInd = 0; currentPixelInd < m_width*m_height; currentPixelInd++)
    {
        m_rgb[3*currentPixelInd]   = data.colorMap[3*currentPixelInd+2];
        m_rgb[3*currentPixelInd+1] = data.colorMap[3*currentPixelInd+1];
        m_rgb[3*currentPixelInd+2] = data.colorMap[3*currentPixelInd];
    }
}

void FrameColor::write(string pathFrame, string pathReport) {
    FILE* pFile;
    pFile = fopen(pathFrame.c_str(), "wb");
    fwrite(m_rgb, sizeof(uint8_t), m_width*m_height, pFile);
    fclose(pFile);
    FILE* pFileReport;
    pFileReport = fopen(pathReport.c_str(), "a");
    fprintf(pFileReport, "%d, %d, %d",
            m_indexFrameColor, m_timeStamp, m_correspFrameDepth);
    fclose(pFileReport);
}

string FrameColor::formatFilename(int indexFrameColor) {
    char buff[50];
    snprintf(buff, sizeof(buff), "color_%05d.dat", indexFrameColor);
    string filename = buff;
    return filename;
}
