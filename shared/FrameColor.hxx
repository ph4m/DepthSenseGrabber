#ifndef SHARED_FRAMECOLOR_H_
#define SHARED_FRAMECOLOR_H_

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

class FrameColor {
    private:
        int m_width;
        int m_height;
        int m_timeStamp;
        int m_indexFrameColor;
        int m_correspFrameDepth;
        uint8_t* m_rgb; // RGB format

    public:
        FrameColor(int width, int height);

        void setWidth(int width);
        void setHeight(int height);
        void setTimeStamp(int timeStamp);
        void setIndexFrameColor(int indexFrameColor);
        void setCorrespFrameDepth(int correspFrameDepth);

        int getWidth() { return m_width; }
        int getHeight() { return m_height; }
        int getTimeStamp() { return m_timeStamp; }
        int getIndexFrameColor() { return m_indexFrameColor; }
        int getCorrespFrameDepth() { return m_correspFrameDepth; }

        void importColorMap(ColorNode::NewSampleReceivedData data);
        //void write(string pathFrame, string pathReport);

#endif
