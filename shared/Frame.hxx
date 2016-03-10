#ifndef SHARED_FRAME_H_
#define SHARED_FRAME_H_

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <time.h>

#include <boost/thread.hpp>

#include <chrono>

#include <vector>
#include <exception>

#include <DepthSense.hxx>

#include "AcquisitionParameters.hxx"

using namespace DepthSense;
using namespace std;

class Frame {
    protected:
        int m_width;
        int m_height;
        int m_timeStamp;
        int m_indexFrame;

    public:
        Frame(int width, int height);

        void setWidth(int width);
        void setHeight(int height);
        void setTimeStamp(int timeStamp);
        void setIndexFrame(int indexFrame);

        int getWidth() { return m_width; }
        int getHeight() { return m_height; }
        int getTimeStamp() { return m_timeStamp; }
        int getIndexFrame() { return m_indexFrame; }


};

class FrameColor: public Frame {
    protected:
        int m_correspFrameDepth;
        uint8_t* m_rgb; // RGB format

    public:
        FrameColor(int width, int height);

        void setCorrespFrameDepth(int correspFrameDepth);
        int getCorrespFrameDepth() { return m_correspFrameDepth; }
        void importColorMap(ColorNode::NewSampleReceivedData data);
        void write(string pathFrame, string pathReport);

        static string formatFilenameFrame(int indexFrame);
        static string formatFilenameReport();
};

class FrameDepth: public Frame {
    protected:
        uint16_t* m_depth;
        uint16_t* m_confidence;
        float* m_uv;

    public:
        FrameDepth(int width, int height);

        void importDepthMap(DepthNode::NewSampleReceivedData data);
        void write(string pathFrame, string pathReport);

        static string formatFilenameFrame(int indexFrame);
        static string formatFilenameReport();
};






#endif
