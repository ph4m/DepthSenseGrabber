// DepthSenseGrabber
// http://github.com/ph4m

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <signal.h>
#include <stdio.h>
#include <time.h>

#include <boost/thread.hpp>

#include <chrono>

#include <vector>
#include <exception>

#include <DepthSense.hxx>

#include "DepthSenseGrabberDump.hxx"
#include "../shared/ConversionTools.hxx"
#include "../shared/AcquisitionParameters.hxx"

#include "Frame.hxx"

using namespace DepthSense;
using namespace std;

#define DEPTHSENSEGRABBER_DUMP_DELAY
#if defined(DEPTHSENSEGRABBER_DUMP_DELAY)
vector<FrameColor*> vFrameColor;
vector<FrameDepth*> vFrameDepth;
#endif

string filenameReportColor, filenameReportDepth;

long seconds, useconds;
std::chrono::high_resolution_clock::time_point timeStart, timeCurrent;

bool usingUSB30Flag = true; // if the camera is plugged on a USB 3.0 port

int waitSecondsBeforeGrab = 1;

int framerateDepth = 60;
int framerateColor = 30;


FrameFormat frameFormatDepth = FRAME_FORMAT_QVGA; // Depth QVGA

// Color map configuration, comment out undesired parameters

FrameFormat frameFormatColor;
int widthDepth, heightDepth;
int widthColor, heightColor;


int timestamp;
clock_t clockStartGrab;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
    g_aFrames++;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
/* Comments from SoftKinetic

   From data you can get

   ::DepthSense::Pointer< uint8_t > colorMap
   The color map. If captureConfiguration::compression is
   DepthSense::COMPRESSION_TYPE_MJPEG, the output format is BGR, otherwise
   the output format is YUY2.
   */
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
    if (g_dFrames < 1) {
        return ;
    }
    int indexFrameColor = g_cFrames;
    int correspFrameDepth = g_dFrames-1;
    FrameColor* pFrameColor = new FrameColor(widthColor, heightColor);
    timeCurrent = std::chrono::high_resolution_clock::now();
    timestamp = (int) std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeStart).count();
    pFrameColor->setTimestamp(timestamp);
    pFrameColor->setIndexFrame(indexFrameColor);
    pFrameColor->setCorrespFrame(correspFrameDepth);
    pFrameColor->importColorMap(data);

#if defined(DEPTHSENSEGRABBER_DUMP_DELAY)
    vFrameColor.push_back(pFrameColor);
#else
    string filenameFrameColor = FrameColor::formatFilenameFrame(indexFrameColor);
    pFrameColor->writeFrame(filenameFrameColor);
    pFrameColor->updateReport(filenameReportColor);
#endif

    g_cFrames++;

}

/*----------------------------------------------------------------------------*/
// New depth sample event handler

/* From SoftKinetic

   ::DepthSense::Pointer< int16_t > depthMap
   The depth map in fixed point format. This map represents the cartesian depth of each
   pixel, expressed in millimeters. Valid values lies in the range [0 - 31999]. Saturated
   pixels are given the special value 32002.
    ::DepthSense::Pointer< float > depthMapFloatingPoint
   The depth map in floating point format. This map represents the cartesian depth of
   each pixel, expressed in meters. Saturated pixels are given the special value -2.0.
   */

void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    int indexFrameDepth = g_dFrames;
    int correspFrameColor = g_cFrames;
    FrameDepth* pFrameDepth = new FrameDepth(widthDepth, heightDepth);
    timeCurrent = std::chrono::high_resolution_clock::now();
    timestamp = (int) std::chrono::duration_cast<std::chrono::milliseconds>(timeCurrent - timeStart).count();
    pFrameDepth->setTimestamp(timestamp);
    pFrameDepth->setIndexFrame(indexFrameDepth);
    pFrameDepth->setCorrespFrame(correspFrameColor);
    pFrameDepth->importDepthMap(data);


#if defined(DEPTHSENSEGRABBER_DUMP_DELAY)
    vFrameDepth.push_back(pFrameDepth);
#else
    string filenameFrameDepth = FrameDepth::formatFilenameFrame(indexFrameDepth);
    pFrameDepth->writeFrame(filenameFrameDepth);
    pFrameDepth->updateReport(filenameReportDepth);
#endif

    g_dFrames++;

}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
    g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

    AudioNode::Configuration config = g_anode.getConfiguration();
    config.sampleRate = 44100;

    try
    {
        g_context.requestControl(g_anode,0);

        g_anode.setConfiguration(config);

        g_anode.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}




/*----------------------------------------------------------------------------*/

void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
    DepthNode::Configuration configRef(frameFormatDepth, framerateDepth, DepthNode::CAMERA_MODE_CLOSE_MODE, true);
    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = frameFormatDepth;
    config.framerate = framerateDepth;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    g_dnode.setEnableDepthMap(true);
    g_dnode.setEnableUvMap(true);
    g_dnode.setEnableConfidenceMap(true);

    try
    {
        g_context.requestControl(g_dnode,0);
        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("DEPTH Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("DEPTH Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("DEPTH IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("DEPTH Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("DEPTH Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("DEPTH Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("DEPTH TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = frameFormatColor;
    config.compression = COMPRESSION_TYPE_MJPEG; // can also be COMPRESSION_TYPE_YUY2
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = framerateColor;
    g_cnode.setEnableColorMap(true);


    try
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
        g_cnode.setBrightness(0);
        g_cnode.setContrast(5);
        g_cnode.setSaturation(5);
        g_cnode.setHue(0);
        g_cnode.setGamma(3);
        g_cnode.setWhiteBalance(4650);
        g_cnode.setSharpness(5);
        g_cnode.setWhiteBalanceAuto(true);
    }
    catch (ArgumentException& e)
    {
        printf("COLOR Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("COLOR Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("COLOR IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("COLOR Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("COLOR Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("COLOR Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("COLOR TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        printf("Configuring depth node\n");
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        printf("Configuring color node\n");
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

    if ((node.is<AudioNode>())&&(!g_anode.isSet()))
    {
        printf("Configuring audio node\n");
        g_anode = node.as<AudioNode>();
        configureAudioNode();
        if (!usingUSB30Flag) g_context.registerNode(node); // switch this off to save bandwidth
    }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
        g_anode.unset();
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
void capture()
{

    printf("Starting capture\n");
    g_context = Context::create("localhost");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        vector<Node> na = da[0].getNodes();

        printf("Found %lu nodes\n",na.size());

        for (int n = 0; n < (int)na.size(); n++)
            configureNode(na[n]);
    }


    printf("DepthSenseGrabber, Feb. 2014. (thp@pham.in)\n");

    clockStartGrab = clock()+CLOCKS_PER_SEC*waitSecondsBeforeGrab;

    g_context.startNodes();

    printf("Waiting %i seconds before grabbing...\n",waitSecondsBeforeGrab);
    while (clock() < clockStartGrab);
    printf("Now grabbing!\n");

    timeStart = std::chrono::high_resolution_clock::now();

    g_context.run();

}

void stop_capture(int s)
{
    cout << "Stopping capture" << endl;

    g_context.stopNodes();

    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    //if (g_anode.isSet()) g_context.unregisterNode(g_anode);

    if (g_pProjHelper)
        delete g_pProjHelper;


#if defined(DEPTHSENSEGRABBER_DUMP_DELAY)
    for (int iFrame = 0; iFrame < vFrameColor.size(); iFrame++) {
        cout << "Processing color frame " << iFrame << endl;
        string filenameFrameColor = FrameColor::formatFilenameFrame(iFrame);
        vFrameColor[iFrame]->writeFrame(filenameFrameColor);
        vFrameColor[iFrame]->updateReport(filenameReportColor);
        delete vFrameColor[iFrame];
    }
    for (int iFrame = 0; iFrame < vFrameDepth.size(); iFrame++) {
        cout << "Processing depth frame " << iFrame << endl;
        string filenameFrameDepth = FrameDepth::formatFilenameFrame(iFrame);
        vFrameDepth[iFrame]->writeFrame(filenameFrameDepth);
        vFrameDepth[iFrame]->updateReport(filenameReportDepth);
        delete vFrameDepth[iFrame];
    }
#endif


}

#ifdef _MSC_VER
BOOL WINAPI consoleHandler(DWORD signal) {

    if (signal == CTRL_C_EVENT) {
        printf("Ctrl-C handled\n"); // do cleanup
        stop_capture(0);
    }

    return TRUE;
}
#endif

int main(int argc, char* argv[]) {
#if defined(DEPTHSENSEGRABBER_DUMP_DELAY)
    cout << "Delaying writing to capture end" << endl;
#else
    cout << "Writing on the fly" << endl;
#endif

    int flagColorFormat = FORMAT_VGA_ID;

    widthDepth = FORMAT_QVGA_WIDTH;
    heightDepth = FORMAT_QVGA_HEIGHT;

    switch (flagColorFormat) {
        case FORMAT_VGA_ID:
            frameFormatColor = FRAME_FORMAT_VGA;
            widthColor = FORMAT_VGA_WIDTH;
            heightColor = FORMAT_VGA_HEIGHT;
            break;
        case FORMAT_WXGA_ID:
            frameFormatColor = FRAME_FORMAT_WXGA_H;
            widthColor = FORMAT_WXGA_WIDTH;
            heightColor = FORMAT_WXGA_HEIGHT;
            break;
        case FORMAT_NHD_ID:
            frameFormatColor = FRAME_FORMAT_NHD;
            widthColor = FORMAT_NHD_WIDTH;
            heightColor = FORMAT_NHD_HEIGHT;
            break;
        default:
            printf("Unknown flagColorFormat");
            exit(EXIT_FAILURE);
    }

    filenameReportColor = FrameColor::formatFilenameReport();
    FILE* pFileReportColor;
    pFileReportColor = fopen(filenameReportColor.c_str(), "w");
    fprintf(pFileReportColor, "#color frame, timestamp, corresponding depth frame, %d, %d\n",
            widthColor, heightColor);
    fclose(pFileReportColor);

    filenameReportDepth = FrameDepth::formatFilenameReport();
    FILE* pFileReportDepth;
    pFileReportDepth = fopen(filenameReportDepth.c_str(), "w");
    fprintf(pFileReportDepth, "#depth frame, timestamp, corresponding color frame, %d, %d\n",
            widthDepth, heightDepth);
    fclose(pFileReportDepth);


    boost::thread capture_thread(capture);
    printf("Starting capture thread - Ctrl-C to stop\n");

#ifdef _MSC_VER
    if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
        printf("\nERROR: Could not set control handler");
        return 1;
    }
#else
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = stop_capture;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
#endif

    //while (1);
    pause();

    return 0;

}

