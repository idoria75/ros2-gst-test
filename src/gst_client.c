#include <gst/gst.h>
#include <stdio.h>

// Custom callback function to handle messages from the GStreamer bus
static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    switch (GST_MESSAGE_TYPE(message))
    {
    case GST_MESSAGE_ERROR:
    {
        GError *err;
        gchar *debug_info;
        gst_message_parse_error(message, &err, &debug_info);
        g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(message->src), err->message);
        g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
        g_error_free(err);
        g_free(debug_info);
        break;
    }
    case GST_MESSAGE_WARNING:
    {
        GError *err;
        gchar *debug_info;
        gst_message_parse_warning(message, &err, &debug_info);
        g_printerr("Warning received from element %s: %s\n", GST_OBJECT_NAME(message->src), err->message);
        g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
        g_error_free(err);
        g_free(debug_info);
        break;
    }
    case GST_MESSAGE_EOS:
        g_print("End of stream reached.\n");
        g_main_loop_quit((GMainLoop *)data);
        break;
    default:
        // Handle other message types here, if needed
        break;
    }
    return TRUE;
}

int main(int argc, char *argv[])
{
    // Initialize GStreamer
    gst_init(&argc, &argv);

    // Create the main GStreamer loop
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    // Create the elements
    GstElement *pipeline, *udpsrc, *depayloader, *capsfilter, *h264parse, *videoconvert, *decoder, *videosink;

    // Create the pipeline
    pipeline = gst_pipeline_new("my-pipeline");

    // Create the elements
    udpsrc = gst_element_factory_make("udpsrc", "udpsrc");
    g_object_set(udpsrc, "port", 5000, NULL);
    g_object_set(udpsrc, "address", "0.0.0.0", NULL);
    g_object_set(udpsrc, "caps", gst_caps_from_string("application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96"), NULL);

    depayloader = gst_element_factory_make("rtph264depay", "depayloader");

    // Create the caps filter for the RTP data format
    // GstCaps *caps = gst_caps_new_simple(
    //     "application/x-rtp",
    //     "media", G_TYPE_STRING, "video",
    //     "clock-rate", G_TYPE_INT, 90000,
    //     "encoding-name", G_TYPE_STRING, "H264",
    //     "payload", G_TYPE_INT, 96,
    //     NULL);

    // Create the caps filter element
    // capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
    // g_object_set(capsfilter, "caps", caps, NULL);
    // gst_caps_unref(caps);

    // Create the h264parse and videoconvert elements
    h264parse = gst_element_factory_make("h264parse", "h264parse");
    videoconvert = gst_element_factory_make("videoconvert", "videoconvert");

    // Create the decoder element
    decoder = gst_element_factory_make("decodebin", "decoder");
    g_object_set(decoder, "use-buffering", FALSE, NULL);

    // Create the videosink element
    videosink = gst_element_factory_make("autovideosink", "videosink");

    // Check if all elements were created successfully
    if (!pipeline || !udpsrc || !depayloader || !h264parse || !videoconvert || !decoder || !videosink)
    {
        g_printerr("Not all elements could be created. Exiting.\n");
        return -1;
    }

    // Set up the pipeline
    gst_bin_add_many(GST_BIN(pipeline), udpsrc, depayloader, h264parse, decoder, videoconvert, videosink, NULL);
    gst_element_link_many(udpsrc, depayloader, h264parse, decoder, videoconvert, videosink, NULL);

    printf("Pipeline built!\n");

    // Watch for messages on the bus
    GstBus *bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, (GstBusFunc)bus_callback, loop);
    gst_object_unref(bus);

    // Start the pipeline
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    printf("Pipeline started!\n");

    // Run the main GStreamer loop
    g_main_loop_run(loop);

    // Stop and clean up the pipeline and elements
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_main_loop_unref(loop);

    return 0;
}
