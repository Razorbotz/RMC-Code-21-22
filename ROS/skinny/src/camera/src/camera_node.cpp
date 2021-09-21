#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>

#include <glib.h>
#include <gst/gst.h>

#include <messages/Camera.h>

std::string getAddress(std::string network){
    return "192.168.1.102";
}


int main(int argc, char **argv){

    ros::init(argc,argv,"camera");
    gst_init (&argc, &argv);

    ros::NodeHandle nodeHandleP("~");
    ros::NodeHandle nodeHandle;

    messages::Camera camera;

    bool success;
    std::string cameraTopic;
    success=nodeHandleP.getParam("camera_topic", cameraTopic);
    std::cout << success << " camera_topic: " << cameraTopic << std::endl;

    std::string cameraPath;
    success=nodeHandleP.getParam("camera_path", cameraPath);
    std::cout << success << " camera_path: " << cameraPath << std::endl;

    std::string hostAddress;
    success=nodeHandleP.getParam("host_address", hostAddress);
    std::cout << success << " host_address: " << hostAddress << std::endl;

    int port=0;
    success=nodeHandleP.getParam("port", port);
    std::cout << success << " port: " << port << std::endl;

    if(hostAddress=="0.0.0.0"){
	hostAddress=getAddress("wlan0");
    }
    camera.address=hostAddress;
    camera.port=port;

    ros::Publisher cameraPublisher=nodeHandle.advertise<messages::Camera>(cameraTopic.c_str(),1);

    GMainLoop* loop = g_main_loop_new (NULL, FALSE);
    GstElement* pipeline = gst_pipeline_new("cameraPipeline");

    GstElement* v4l2src = gst_element_factory_make("v4l2src", "v4l2src");
    if (v4l2src == NULL) std::cout << "Could not create 'v4l2src' element" << std::endl;
    g_object_set(G_OBJECT(v4l2src), "device", cameraPath, NULL);
    GstElement* videoconvert = gst_element_factory_make("videoconvert","videoconvert");
    if (videoconvert == NULL) std::cout << "Could not create 'videoconvert' element" << std::endl;
    GstElement* videoscale = gst_element_factory_make("videoscale","videoscale");
    if (videoscale == NULL) std::cout << "Could not create 'videoscale' element" << std::endl;
    GstElement* capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    g_assert(capsfilter != NULL); /* should always exist */
    GstCaps* capsfilterParameters = gst_caps_new_simple("video/x-raw",
               "width", G_TYPE_INT, 320,
               "height", G_TYPE_INT, 240,
               NULL);
    g_object_set(G_OBJECT(capsfilter), "caps", capsfilterParameters, NULL);
    gst_caps_unref(capsfilterParameters);
    GstElement* theoraenc = gst_element_factory_make("theoraenc","theoraenc");
    if (theoraenc == NULL) std::cout << "Could not create 'theoraenc' element" << std::endl;
    GstElement* oggmux = gst_element_factory_make("oggmux","oggmux");
    if (oggmux == NULL) std::cout << "Could not create 'oggmux' element" << std::endl;
    GstElement* tcpserversink = gst_element_factory_make("tcpserversink","tcpserversink");
    if (tcpserversink == NULL) std::cout << "Could not create 'tcpserversink' element" << std::endl;
    g_object_set(G_OBJECT(tcpserversink), "host", hostAddress.c_str(), NULL);
    g_object_set(G_OBJECT(tcpserversink), "port", (gint)port, NULL);

    gst_bin_add_many(GST_BIN(pipeline), v4l2src, videoconvert, videoscale, capsfilter, theoraenc, oggmux, tcpserversink, NULL);
    gst_element_link_many(v4l2src, videoconvert, videoscale, capsfilter, theoraenc, oggmux, tcpserversink, NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    std::cout << "begining loop " << std::endl;
    g_main_loop_run (loop);
    while(ros::ok()){
        while(g_main_context_pending(g_main_context_default())){
            g_main_context_iteration(g_main_context_default(), false);	
        }
	cameraPublisher.publish(camera);
        ros::spinOnce();
    }

    g_main_loop_unref(loop);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    gst_object_unref(v4l2src);
    gst_object_unref(videoconvert);
    gst_object_unref(videoscale);
    gst_object_unref(theoraenc);
    gst_object_unref(oggmux);
    gst_object_unref(tcpserversink);
}
