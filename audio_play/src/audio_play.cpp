#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{

class RosGstPlay
{
public:
    RosGstPlay()
    {
        GstPad *audiopad;

        std::string dst_type;
        std::string format;
        int channels, depth, sample_rate;

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "autoaudiosink");
        ros::param::param<std::string>("~format", format, "mp3");
        ros::param::param<int>("~depth", depth, 16);
        ros::param::param<int>("~sample_rate", sample_rate, 16000);
        ros::param::param<int>("~channels", channels, 1);

        _sub = _nh.subscribe("audio", 10, &RosGstPlay::onAudio, this);

        gboolean link_ok = false;

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");

        g_signal_connect(_source, "need-data", G_CALLBACK(cb_need_data), this);

        if (dst_type == "autoaudiosink") {
            _sink = gst_element_factory_make("autoaudiosink", "sink");

            if (format == "raw") {
                _filter = gst_element_factory_make("capsfilter", "filter");
                std::string f = std::string("S") + std::to_string(depth) + "LE";
                GstCaps *caps = gst_caps_new_simple("audio/x-raw",
                                                    "format",   G_TYPE_STRING, f.c_str(),
                                                    "channels", G_TYPE_INT, channels,
                                                    "rate",     G_TYPE_INT, sample_rate,
                                                    NULL);

                g_object_set(G_OBJECT(_filter), "caps", caps, NULL);
                gst_caps_unref(caps);
                gst_bin_add_many(GST_BIN(_pipeline), _source, _filter, _sink, NULL);
                link_ok = gst_element_link_many(_source, _filter, _sink, NULL);
            } else {
                _decoder = gst_element_factory_make("decodebin", "decoder");
                g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad), this);
                gst_bin_add_many(GST_BIN(_pipeline), _source, _decoder, NULL);
                link_ok = gst_element_link(_source, _decoder);
                _audio = gst_bin_new("audiobin");
                _convert = gst_element_factory_make("audioconvert", "convert");
                audiopad = gst_element_get_static_pad(_convert, "sink");
                gst_bin_add_many(GST_BIN(_audio), _convert, _sink, NULL);
                link_ok = gst_element_link(_convert, _sink);
                gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
                gst_object_unref(audiopad);
                gst_bin_add(GST_BIN(_pipeline), _audio);
            }

        } else {
            _sink = gst_element_factory_make("filesink", "sink");
            g_object_set(G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
            gst_bin_add_many(GST_BIN(_pipeline), _source, _sink, NULL);
            link_ok = gst_element_link(_source, _sink);
        }

        if (!link_ok) {
            ROS_ERROR_STREAM("Invalid parameters.");
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);

        _loop = g_main_loop_new(NULL, false);
        _gst_thread = boost::thread(boost::bind(g_main_loop_run, _loop));

        _paused = false;
    }

private:

    void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
    {
        if(_paused) {
            gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
            _paused = false;
        }

        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
    }

    static void cb_newpad(GstElement *decodebin, GstPad *pad, gpointer data)
    {
        RosGstPlay *client = reinterpret_cast<RosGstPlay*>(data);

        GstCaps *caps;
        GstStructure *str;
        GstPad *audiopad;

        /* only link once */
        audiopad = gst_element_get_static_pad(client->_audio, "sink");
        if (GST_PAD_IS_LINKED(audiopad)) {
            g_object_unref(audiopad);
            return;
        }

        /* check media type */
        caps = gst_pad_query_caps(pad, NULL);
        str = gst_caps_get_structure(caps, 0);
        if (!g_strrstr(gst_structure_get_name(str), "audio")) {
            gst_caps_unref(caps);
            gst_object_unref(audiopad);
            return;
        }

        gst_caps_unref(caps);

        /* link'n'play */
        gst_pad_link(pad, audiopad);

        g_object_unref(audiopad);
    }

    static void cb_need_data(GstElement *appsrc, guint unused_size, gpointer user_data)
    {
        //ROS_WARN("need-data signal emitted! Pausing the pipeline");
        RosGstPlay *client = reinterpret_cast<RosGstPlay*>(user_data);
        gst_element_set_state(GST_ELEMENT(client->_pipeline), GST_STATE_PAUSED);
        client->_paused = true;
    }

    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    boost::thread _gst_thread;

    GstElement *_pipeline, *_source, *_sink, *_filter, *_decoder, *_convert, *_audio;
    GMainLoop *_loop;

    bool _paused;
};

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "audio_play");
    gst_init(&argc, &argv);

    audio_transport::RosGstPlay client;

    ros::spin();
}
