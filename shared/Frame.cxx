
#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <time.h>

#include <boost/thread.hpp>

#include <chrono>

#include <vector>
#include <exception>
#include <fstream>
#include <sstream>
#include <DepthSense.hxx>

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
void Frame::setTimestamp(int timestamp) {
    m_timestamp = timestamp;
}
void Frame::setIndexFrame(int indexFrame) {
    m_indexFrame = indexFrame;
}
void Frame::setCorrespFrame(int correspFrame) {
    m_correspFrame = correspFrame;
}

string Frame::formatFilenameFrame(int indexFrame, string prefix) {
    char buff[100];
#ifdef _MSC_VER
    _snprintf(buff, sizeof(buff), "%s_%05d.dat", prefix.c_str(), indexFrame);
#else
	snprintf(buff, sizeof(buff), "%s_%05d.dat", prefix.c_str(), indexFrame);
#endif
    string filename = buff;
    return filename;
}

string Frame::formatFilenamePNM(int indexFrame, string prefix) {
    char buff[100];
#ifdef _MSC_VER
    _snprintf(buff, sizeof(buff), "%s_%05d.pnm", prefix.c_str(), indexFrame);
#else
	snprintf(buff, sizeof(buff), "%s_%05d.pnm", prefix.c_str(), indexFrame);
#endif
    string filename = buff;
    return filename;
}


string Frame::formatFilenameReport(string prefix) {
    char buff[100];
#ifdef _MSC_VER
    _snprintf(buff, sizeof(buff), "%s_report.txt", prefix.c_str());
#else
	snprintf(buff, sizeof(buff), "%s_report.txt", prefix.c_str());
#endif
    string filename = buff;
    return filename;
}

void Frame::updateReport(string pathReport) {
    FILE* pFileReport;
    pFileReport = fopen(pathReport.c_str(), "a");
    fprintf(pFileReport, "%d %d %d\n",
            m_indexFrame, m_timestamp, m_correspFrame);
    fclose(pFileReport);
}

void Frame::importReport(string pathReport,
                         Report* p_report) {
    ifstream fileStream(pathReport.c_str());
    if (!fileStream.is_open()) {
        cout << "Could not open report: " << pathReport<< endl;
        exit(EXIT_FAILURE);
    }
    string line;
    while (getline(fileStream, line)) {
        istringstream iss(line);
        int indexFrame, timestamp, correspFrame;
        if (!(iss >> indexFrame >> timestamp >> correspFrame)) {
            cout << "Skipping line: " << iss.str() << endl;
        }
        else {
            FrameCorrespondence frameCorrespondence;
            frameCorrespondence.indexFrame = indexFrame;
            frameCorrespondence.timestamp = timestamp;
            frameCorrespondence.correspFrame = correspFrame;
            p_report->push_back(frameCorrespondence);
        }
    }

}


/*********
 * Color *
 *********/

FrameColor::FrameColor(int width, int height): Frame(width, height) {
    m_rgb = new uint8_t[3*width*height];
}

const string FrameColor::m_prefix = "color";

FrameColor::FrameColor(string pathFrame): Frame(0, 0) {
    ifstream fileStream(pathFrame.c_str(), ios::binary);
    if (!fileStream.is_open()) {
        cout << "Could not open frame: " << pathFrame.c_str() << endl;
        exit(EXIT_FAILURE);
    }
    int width, height;
    fileStream.read(reinterpret_cast < char * > (&width), sizeof(int)/sizeof(char));
    fileStream.read(reinterpret_cast < char * > (&height), sizeof(int)/sizeof(char));
    setWidth(width);
    setHeight(height);
    int nPixels = width*height;
    m_rgb = new uint8_t[3*nPixels];
    fileStream.read(reinterpret_cast < char * > (m_rgb), 3*width*height*sizeof(uint8_t)/sizeof(char));
}

FrameColor::~FrameColor() {
    delete [] m_rgb;
}


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
    fwrite(&m_width, sizeof(int), 1, pFile);
    fwrite(&m_height, sizeof(int), 1, pFile);
    int nPixels = m_width*m_height;
    fwrite(m_rgb, sizeof(uint8_t), 3*nPixels, pFile);
    fclose(pFile);
}


string FrameColor::formatFilenameFrame(int indexFrame) {
    return Frame::formatFilenameFrame(indexFrame, getPrefix());
}

string FrameColor::formatFilenameReport() {
    return Frame::formatFilenameReport(getPrefix());
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

FrameDepth::FrameDepth(string pathFrame): Frame(0, 0) {
    ifstream fileStream(pathFrame.c_str(), ios::binary);
    if (!fileStream.is_open()) {
        cout << "Could not open frame: " << pathFrame.c_str() << endl;
        exit(EXIT_FAILURE);
    }
    int width, height;
    fileStream.read(reinterpret_cast < char * > (&width), sizeof(int)/sizeof(char));
    fileStream.read(reinterpret_cast < char * > (&height), sizeof(int)/sizeof(char));
    setWidth(width);
    setHeight(height);
    int nPixels = width*height;
    m_depth = new uint16_t[nPixels];
    m_confidence = new uint16_t[width*height];
    m_uv = new float[2*width*height];
    fileStream.read(reinterpret_cast < char * > (m_depth), nPixels*sizeof(uint16_t)/sizeof(char));
    fileStream.read(reinterpret_cast < char * > (m_confidence), nPixels*sizeof(uint16_t)/sizeof(char));
    fileStream.read(reinterpret_cast < char * > (m_uv), 2*nPixels*sizeof(float)/sizeof(char));
}

FrameDepth::~FrameDepth() {
    delete [] m_depth;
    delete [] m_confidence;
    delete [] m_uv;
}




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
    fwrite(&m_width, sizeof(int), 1, pFile);
    fwrite(&m_height, sizeof(int), 1, pFile);
    int nPixels = m_width*m_height;
    fwrite(m_depth, sizeof(uint16_t), nPixels, pFile);
    fwrite(m_confidence, sizeof(uint16_t), nPixels, pFile);
    fwrite(m_uv, sizeof(float), 2*nPixels, pFile);
    fclose(pFile);
}


string FrameDepth::formatFilenameFrame(int indexFrame) {
    return Frame::formatFilenameFrame(indexFrame, getPrefix());
}

string FrameDepth::formatFilenameReport() {
    return Frame::formatFilenameReport(getPrefix());
}


