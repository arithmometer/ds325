#include <iostream>
#include <vector>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <DepthSense.hxx>

using namespace DepthSense;
using namespace cv;
using namespace std;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

int thresh = 5000;

Mat frame;
Mat frame_d;
Mat mask = Mat::ones(480, 640, CV_8UC1);
Mat inv_mask = Mat::zeros(480, 640, CV_8UC1);

// audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
    // Do nothing for now
}

/* Comments from SoftKinetic
From data you can get
::DepthSense::Pointer< uint8_t > colorMap
The color map. If captureConfiguration::compression is
DepthSense::COMPRESSION_TYPE_MJPEG, the output format is BGR, otherwise
the output format is YUY2.
 */
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);

    frame = Mat(h, w, CV_8UC3);
    memcpy(frame.data, &*data.colorMap, h * w * 3);

    Mat res(h ,w, CV_8UC3);

    Mat gray = Mat(h, w, CV_8UC3);
    Mat gray_part = Mat(h, w, CV_8UC3);
    Mat color_part = Mat(h, w, CV_8UC3);
    cvtColor(frame, gray, COLOR_BGR2GRAY);
    frame.copyTo(color_part, inv_mask);

#if 0
    gray.copyTo(gray_part, mask);
    vector<Mat> plan;
    plan.push_back(gray_part);
    plan.push_back(gray_part);
    plan.push_back(gray_part);
    merge(plan, gray_part);
#endif
    gray_part = Mat::zeros(h, w, CV_8UC3);

    addWeighted(gray_part, 1, color_part, 1, 0.0, res);

    imshow("Captured video", res);
    if(waitKey(30) >= 0) {
        g_context.quit();
    };
}

/* Comments from SoftKinetic
::DepthSense::Pointer< int16_t > depthMap
The depth map in fixed point format. This map represents the cartesian depth of each
pixel, expressed in millimeters. Valid values lies in the range [0 - 31999]. Saturated
pixels are given the special value 32002.
::DepthSense::Pointer< float > depthMapFloatingPoint
The depth map in floating point format. This map represents the cartesian depth of
each pixel, expressed in meters. Saturated pixels are given the special value -2.0.
*/
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
//    // Project some 3D points in the Color Frame
//    if (!g_pProjHelper)
//    {
//        g_pProjHelper = new ProjectionHelper(data.stereoCameraParameters);
//        g_scp = data.stereoCameraParameters;
//    }
//    else if (g_scp != data.stereoCameraParameters)
//    {
//        g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
//        g_scp = data.stereoCameraParameters;
//    }
//
//    int32_t w, h;
//    FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
//    int cx = w/2;
//    int cy = h/2;
//
//    Vertex p3DPoints[4];
//
//    p3DPoints[0] = data.vertices[(cy - h / 4) * w + cx - w / 4];
//    p3DPoints[1] = data.vertices[(cy - h / 4) * w + cx + w / 4];
//    p3DPoints[2] = data.vertices[(cy + h / 4) * w + cx + w / 4];
//    p3DPoints[3] = data.vertices[(cy + h / 4) * w + cx - w / 4];
//
//    Point2D p2DPoints[4];
//    g_pProjHelper->get2DCoordinates(p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);

    frame_d = Mat(h, w, CV_16U, (void *)&*data.depthMap);
    pyrUp(frame_d, frame_d, Size(frame_d.cols * 2, frame_d.rows * 2));

    blur(frame, frame, Size(3,3));

//    mask = Mat::zeros(2 * h, 2 * w, CV_8UC1);
    MatIterator_<uint8_t> itm = mask.begin<uint8_t>();
    MatIterator_<uint8_t> iitm = inv_mask.begin<uint8_t>();
    for(MatIterator_<uint16_t> it = frame_d.begin<uint16_t>(); it != frame_d.end<uint16_t>(); ++it, ++itm, ++iitm) {
        if(*it < thresh) {
            *itm = 0;
            *iitm = 1;
        }
    }

    imshow("Depth", frame_d);

    if(waitKey(30) >= 0) {
        g_context.quit();
    };
}

void configureAudioNode()
{
    g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

    AudioNode::Configuration config = g_anode.getConfiguration();
    config.sampleRate = 44100;

    try {
        g_context.requestControl(g_anode,0);

        g_anode.setConfiguration(config);

        g_anode.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e) {
        cerr << "Argument Exception: " << e.what() << endl;
    }
    catch (UnauthorizedAccessException& e) {
        cerr << "Unauthorized Access Exception: " << e.what() << endl;
    }
    catch (ConfigurationException& e) {
        cerr << "Configuration Exception: " << e.what() << endl;
    }
    catch (StreamingException& e) {
        cerr << "Streaming Exception: " << e.what() << endl;
    }
    catch (TimeoutException&) {
        cerr << "TimeoutException" << endl;
    }
}

void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA;
    config.framerate = 25;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    g_dnode.setEnableVertices(true);

    try {
        g_context.requestControl(g_dnode,0);

        g_dnode.setEnableDepthMap(true);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e) {
        cerr << "Argument Exception: " << e.what() << endl;
    }
    catch (UnauthorizedAccessException& e) {
        cerr << "Unauthorized Access Exception: " << e.what() << endl;
    }
    catch (IOException& e) {
        cerr << "IO Exception: " << e.what() << endl;
    }
    catch (InvalidOperationException& e) {
        cerr << "Invalid Operation Exception: " << e.what() << endl;
    }
    catch (ConfigurationException& e) {
        cerr << "Configuration Exception: " << e.what() << endl;
    }
    catch (StreamingException& e) {
        cerr << "Streaming Exception: " << e.what() << endl;
    }
    catch (TimeoutException&) {
        cerr << "TimeoutException" << endl;
    }

}

void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_VGA;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 25;

    g_cnode.setEnableColorMap(true);

    try {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e) {
        cerr << "Argument Exception: " << e.what();
    }
    catch (UnauthorizedAccessException& e) {
        cerr << "Unauthorized Access Exception: " << e.what();
    }
    catch (IOException& e) {
        cerr << "IO Exception: " << e.what();
    }
    catch (InvalidOperationException& e) {
        cerr << "Invalid Operation Exception: " << e.what();
    }
    catch (ConfigurationException& e) {
        cerr << "Configuration Exception: " << e.what();
    }
    catch (StreamingException& e) {
        cerr << "Streaming Exception: " << e.what();
    }
    catch (TimeoutException&) {
        cerr << "TimeoutException" << endl;
    }
}

void configureNode(DepthSense::Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet())) {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet())) {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

#if 0
    if ((node.is<AudioNode>())&&(!g_anode.isSet())) {
        g_anode = node.as<AudioNode>();
        configureAudioNode();
        g_context.registerNode(node);
    }
#endif
}

void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode)) {
        g_anode.unset();
    }
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode)) {
        g_cnode.unset();
    }
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode)) {
        g_dnode.unset();
    }
    cerr << "Node disconnected" << endl;
}

void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound) {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    cerr << "Device disconnected" << endl;
}

int main(int argc, char** argv)
{
    g_context = Context::create("localhost");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1) {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        vector<DepthSense::Node> na = da[0].getNodes();

        cerr << "Found " << na.size() << " nodes" << endl;

        for (int n = 0; n < (int) na.size(); n++) {
            configureNode(na[n]);
        }
    }

    namedWindow("Depth", WINDOW_AUTOSIZE);
    namedWindow("Captured video", WINDOW_AUTOSIZE);

    g_context.startNodes();
    g_context.run();
    g_context.stopNodes();

    if (g_cnode.isSet()) {
        g_context.unregisterNode(g_cnode);
    }
    if (g_dnode.isSet()) {
        g_context.unregisterNode(g_dnode);
    }
    if (g_anode.isSet()) {
        g_context.unregisterNode(g_anode);
    }

    if (g_pProjHelper) {
        delete g_pProjHelper;
    }

    return(EXIT_SUCCESS);
}
