
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

#include "Frame.hxx"

using namespace DepthSense;
using namespace std;

Frame::Frame(int width, int height) {
    setWidth(width);
    setHeight(height);
}

void Frame::setWidth(int width) {
    m_width = width;
}
void Frame::setHeight(int height) {
    m_height = height;
}
void Frame::setTimeStamp(int timeStamp) {
    m_timeStamp = timeStamp;
}
void Frame::setIndexFrame(int indexFrame) {
    m_indexFrame = indexFrame;
}

FrameColor::FrameColor(int width, int height): Frame(width, height) {
    m_rgb = new uint8_t[3*width*height];
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
    fprintf(pFileReport, "%d, %d, %d\n",
            m_indexFrame, m_timeStamp, m_correspFrameDepth);
    fclose(pFileReport);
}

string FrameColor::formatFilenameFrame(int indexFrame) {
    char buff[50];
    snprintf(buff, sizeof(buff), "color_%05d.dat", indexFrame);
    string filename = buff;
    return filename;
}

string FrameColor::formatFilenameReport() {
    string filename = "color_report.txt";
    return filename;
}
