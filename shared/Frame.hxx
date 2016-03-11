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
        int m_correspFrame;

    public:
        Frame(int width, int height);

        void setWidth(int width);
        void setHeight(int height);
        void setTimeStamp(int timeStamp);
        void setIndexFrame(int indexFrame);
        void setCorrespFrame(int correspFrame);

        int getWidth() { return m_width; }
        int getHeight() { return m_height; }
        int getTimeStamp() { return m_timeStamp; }
        int getIndexFrame() { return m_indexFrame; }
        int getCorrespFrame() { return m_correspFrame; }

        void updateReport(string pathReport);

        static string formatFilenameFrame(int indexFrame, string prefix);
        static string formatFilenameReport(string prefix);

};

class FrameColor: public Frame {
    protected:
        static const string m_prefix;
        uint8_t* m_rgb; // RGB format

    public:
        FrameColor(int width, int height);

        void importColorMap(ColorNode::NewSampleReceivedData data);
        void writeFrame(string pathFrame);

        static string formatFilenameFrame(int indexFrame);
        static string formatFilenameReport();
};

class FrameDepth: public Frame {
    protected:
        static const string m_prefix;
        uint16_t* m_depth;
        uint16_t* m_confidence;
        float* m_uv;

    public:
        FrameDepth(int width, int height);

        void importDepthMap(DepthNode::NewSampleReceivedData data);
        void writeFrame(string pathFrame);

        static string formatFilenameFrame(int indexFrame);
        static string formatFilenameReport();
};






#endif
