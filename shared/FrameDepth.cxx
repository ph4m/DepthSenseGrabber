
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

class FrameDepth {
    public:
        int timeStamp;
        int width;
        int height;
        uint16_t* depth;
        uint16_t* confidence;
        UV* uv;
};
