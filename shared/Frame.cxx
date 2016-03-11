
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
void Frame::setCorrespFrame(int correspFrame) {
    m_correspFrame = correspFrame;
}

string Frame::formatFilenameFrame(int indexFrame, string prefix) {
    char buff[50];
    snprintf(buff, sizeof(buff), "%s_%05d.dat", prefix.c_str(), indexFrame);
    string filename = buff;
    return filename;
}

string Frame::formatFilenameReport(string prefix) {
    char buff[50];
    snprintf(buff, sizeof(buff), "%s_report.txt", prefix.c_str());
    string filename = buff;
    return filename;
}

void Frame::updateReport(string pathReport) {
    FILE* pFileReport;
    pFileReport = fopen(pathReport.c_str(), "a");
    fprintf(pFileReport, "%d, %d, %d\n",
            m_indexFrame, m_timeStamp, m_correspFrame);
    fclose(pFileReport);
}


/*********
 * Color *
 *********/

FrameColor::FrameColor(int width, int height): Frame(width, height) {
    m_rgb = new uint8_t[3*width*height];
}
const string FrameColor::m_prefix = "color";

void FrameColor::importColorMap(ColorNode::NewSampleReceivedData data) {
    for (int currentPixelInd = 0; currentPixelInd < m_width*m_height; currentPixelInd++)
    {
        m_rgb[3*currentPixelInd]   = data.colorMap[3*currentPixelInd+2];
        m_rgb[3*currentPixelInd+1] = data.colorMap[3*currentPixelInd+1];
        m_rgb[3*currentPixelInd+2] = data.colorMap[3*currentPixelInd];
    }
}

void FrameColor::writeFrame(string pathFrame) {
    FILE* pFile;
    pFile = fopen(pathFrame.c_str(), "wb");
    int nPixels = m_width*m_height;
    fwrite(&nPixels, sizeof(int), 1, pFile);
    fwrite(m_rgb, sizeof(uint8_t), 3*nPixels, pFile);
    fclose(pFile);
}

string FrameColor::formatFilenameFrame(int indexFrame) {
    return Frame::formatFilenameFrame(indexFrame, m_prefix);
}

string FrameColor::formatFilenameReport() {
    return Frame::formatFilenameReport(m_prefix);
}


/*********
 * Depth *
 *********/

FrameDepth::FrameDepth(int width, int height): Frame(width, height) {
    m_depth = new uint16_t[width*height];
    m_confidence = new uint16_t[width*height];
    m_uv = new float[2*width*height];
}
const string FrameDepth::m_prefix = "depth";




void FrameDepth::importDepthMap(DepthNode::NewSampleReceivedData data) {
    for (int currentPixelInd = 0; currentPixelInd < m_width*m_height; currentPixelInd++)
    {
        m_depth[currentPixelInd] = data.depthMap[currentPixelInd];
        m_confidence[currentPixelInd] = data.confidenceMap[currentPixelInd];
        m_uv[2*currentPixelInd] = data.uvMap[currentPixelInd].u;
        m_uv[2*currentPixelInd+1] = data.uvMap[currentPixelInd].v;
    }
}


void FrameDepth::writeFrame(string pathFrame) {
    FILE* pFile;
    pFile = fopen(pathFrame.c_str(), "wb");
    int nPixels = m_width*m_height;
    fwrite(&nPixels, sizeof(int), 1, pFile);
    fwrite(m_depth, sizeof(uint16_t), nPixels, pFile);
    fwrite(m_confidence, sizeof(uint16_t), nPixels, pFile);
    fwrite(m_uv, sizeof(float), 2*nPixels, pFile);
    fclose(pFile);
}


string FrameDepth::formatFilenameFrame(int indexFrame) {
    return Frame::formatFilenameFrame(indexFrame, m_prefix);
}

string FrameDepth::formatFilenameReport() {
    return Frame::formatFilenameReport(m_prefix);
}


