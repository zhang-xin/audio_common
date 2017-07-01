#include <stdio.h>
#include <string>
#include <gst/gst.h>
#include <gst/audio/audio-format.h>
#include <gst/app/gstappsink.h>
#include <boost/thread.hpp>

#include <ros/ros.h>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{

class RosGstCapture
{
public:
    RosGstCapture()
    {
        _bitrate = 192;

        std::string dst_type;

        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~format", _format, "mp3");

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "appsink");

        // The source file of the audio, empty for autoaudiosrc
        std::string src;
        ros::param::param<std::string>("~src", src, "");

        // The source of the audio
        std::string device;
        ros::param::param<std::string>("~device", device, "");

        _pub = _nh.advertise<audio_common_msgs::AudioData>("audio", 10, true);

        _loop = g_main_loop_new(NULL, false);
        _pipeline = gst_pipeline_new("ros_pipeline");
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        gst_bus_add_signal_watch(_bus);
        g_signal_connect(_bus, "message::error", G_CALLBACK(onMessage), this);
        g_object_unref(_bus);

        // We create the sink first, just for convenience
        if (dst_type == "appsink") {
            _sink = gst_element_factory_make("appsink", "sink");
            g_object_set(G_OBJECT(_sink), "emit-signals", true, NULL);
            g_object_set(G_OBJECT(_sink), "max-buffers", 100, NULL);
            g_object_set(G_OBJECT(_sink), "sync", true, NULL);
            g_signal_connect(G_OBJECT(_sink), "new-sample", G_CALLBACK(onNewBuffer), this);
        } else {
            printf("file sink\n");
            _sink = gst_element_factory_make("filesink", "sink");
            g_object_set(G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
        }

        if (src == "") {
            _source = gst_element_factory_make("autoaudiosrc", "source");
            // if device isn't specified, it will use the default which is
            // the alsa default source.
            // A valid device will be of the foram hw:0,0 with other numbers
            // than 0 and 0 as are available.
            if (device != "")
                g_object_set(G_OBJECT(_source), "device", device.c_str(), NULL);
        } else {
            _source = gst_element_factory_make("filesrc", "source");
            g_object_set(G_OBJECT(_source), "location", src.c_str(), NULL);
        }

        gboolean link_ok;

        if (_format == "mp3") {
            if (src == "") {
                _convert = gst_element_factory_make("audioconvert", "convert");
                if (!_convert) {
                    ROS_ERROR_STREAM("Failed to create audioconvert element");
                    exitOnMainThread(1);
                }

                _encode = gst_element_factory_make("lamemp3enc", "encoder");
                if (!_encode) {
                    ROS_ERROR_STREAM("Failed to create encoder element");
                    exitOnMainThread(1);
                }
                g_object_set(G_OBJECT(_encode), "quality", 2.0, NULL);
                g_object_set(G_OBJECT(_encode), "bitrate", _bitrate, NULL);

                gst_bin_add_many(GST_BIN(_pipeline), _source, _convert, _encode, _sink, NULL);
                link_ok = gst_element_link_many(_source, _convert, _encode, _sink, NULL);
            } else {
                _parse = gst_element_factory_make("mpegaudioparse", "parse");
                gst_bin_add_many(GST_BIN(_pipeline), _source, _parse, _sink, NULL);
                link_ok = gst_element_link_many(_source, _parse, _sink, NULL);
            }
        } else if (_format == "raw") {
            _filter = gst_element_factory_make("capsfilter", "filter");
            std::string format = std::string("S") + std::to_string(_depth) + "LE";
            GstCaps *caps = gst_caps_new_simple("audio/x-raw",
                                                "format",   G_TYPE_STRING, format.c_str(),
                                                "channels", G_TYPE_INT, _channels,
                                                "rate",     G_TYPE_INT, _sample_rate,
                                                NULL);
            g_object_set(G_OBJECT(_filter), "caps", caps, NULL);
            gst_caps_unref(caps);

            auto f = gst_audio_format_from_string(format.c_str());
            _parse = gst_element_factory_make("audioparse", "parse");
            g_object_set(G_OBJECT(_parse), "raw-format", f, NULL);
            g_object_set(G_OBJECT(_parse), "channels", _channels, NULL);
            g_object_set(G_OBJECT(_parse), "rate", _sample_rate, NULL);

            gst_bin_add_many(GST_BIN(_pipeline), _source, _filter, _parse, _sink, NULL);
            link_ok = gst_element_link_many(_source, _filter, _parse, _sink, NULL);
        } else {
            ROS_ERROR_STREAM("format must be \"raw\" or \"mp3\"");
            exitOnMainThread(1);
        }

        if (!link_ok) {
            ROS_ERROR_STREAM("Unsupported media type.");
            exitOnMainThread(1);
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _gst_thread = boost::thread(boost::bind(g_main_loop_run, _loop));
    }

    ~RosGstCapture()
    {
        g_main_loop_quit(_loop);
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        gst_object_unref(_pipeline);
        g_main_loop_unref(_loop);
    }

    void exitOnMainThread(int code)
    {
        exit(code);
    }

    void publish(const audio_common_msgs::AudioData &msg)
    {
        _pub.publish(msg);
    }

    static GstFlowReturn onNewBuffer(GstAppSink *appsink, gpointer userData)
    {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GstMapInfo map;

        GstSample *sample;
        g_signal_emit_by_name(appsink, "pull-sample", &sample);

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        audio_common_msgs::AudioData msg;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize( map.size );

        memcpy(&msg.data[0], map.data, map.size);

        server->publish(msg);

        return GST_FLOW_OK;
    }

    static gboolean onMessage(GstBus *bus, GstMessage *message, gpointer userData)
    {
        RosGstCapture *server = reinterpret_cast<RosGstCapture*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        ROS_ERROR_STREAM("gstreamer: " << err->message);
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;

    boost::thread _gst_thread;

    GstElement *_pipeline, *_source, *_filter, *_parse, *_sink, *_convert, *_encode;
    GstBus *_bus;
    int _bitrate, _channels, _depth, _sample_rate;
    GMainLoop *_loop;
    std::string _format;
};

}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "audio_capture");
    gst_init(&argc, &argv);

    audio_transport::RosGstCapture server;
    ros::spin();
}
