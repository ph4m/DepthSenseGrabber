
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

#include "FrameDepth.hxx"

using namespace DepthSense;
using namespace std;

FrameDepth::FrameDepth(int width, int height) {
    setWidth(width);
    setHeight(height);
    m_rgb = new uint8_t[3*width*height];
}

void FrameDepth::setWidth(int width) {
    m_width = width;
}
void FrameDepth::setHeight(int height) {
    m_height = height;
}
void FrameDepth::setTimeStamp(int timeStamp) {
    m_timeStamp = timeStamp;
}
void FrameDepth::setIndexFrameDepth(int indexFrameDepth) {
    m_indexFrameDepth = indexFrameDepth;
}
void FrameDepth::setCorrespFrameDepth(int correspFrameDepth) {
    m_correspFrameDepth = correspFrameDepth;
}

void FrameDepth::importColorMap(ColorNode::NewSampleReceivedData data) {
    for (int currentPixelInd = 0; currentPixelInd < m_width*m_height; currentPixelInd++)
    {
        m_rgb[3*currentPixelInd]   = data.colorMap[3*currentPixelInd+2];
        m_rgb[3*currentPixelInd+1] = data.colorMap[3*currentPixelInd+1];
        m_rgb[3*currentPixelInd+2] = data.colorMap[3*currentPixelInd];
    }
}

void FrameDepth::write(string pathFrame, string pathReport) {
    FILE* pFile;
    pFile = fopen(pathFrame.c_str(), "wb");
    fwrite(m_rgb, sizeof(uint8_t), m_width*m_height, pFile);
    fclose(pFile);
    FILE* pFileReport;
    pFileReport = fopen(pathReport.c_str(), "a");
    fprintf(pFileReport, "%d, %d\n",
            m_indexFrameDepth, m_timeStamp);
    fclose(pFileReport);
}

string FrameDepth::formatFilenameFrame(int indexFrameDepth) {
    char buff[50];
    snprintf(buff, sizeof(buff), "depth_%05d.dat", indexFrameDepth);
    string filename = buff;
    return filename;
}

string FrameDepth::formatFilenameReport() {
    string filename = "depth_report.txt";
    return filename;
}
