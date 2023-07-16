#include <gst/gst.h>
#include <stdio.h>

int main(int argc, char *argv[])
{
    GstElement *pipeline, *source, *udp_sink, *videoconvert, *x264enc, *rtph264pay;
    GstBus *bus;
    GstMessage *msg;
    GstStateChangeReturn ret;
    printf("Created pointers for basic elements!");

    /* Initialize GStreamer */
    gst_init(&argc, &argv);
    printf("Started GStreamer\n!");

    /* Create the elements */
    source = gst_element_factory_make("videotestsrc", "source");
    printf("Created test source\n");

    // sink = gst_element_factory_make("autovideosink", "sink");
    udp_sink = gst_element_factory_make("udpsink", "sink");
    printf("Created UDP sink\n");

    videoconvert = gst_element_factory_make("videoconvert", "convert");
    x264enc = gst_element_factory_make("x264enc", "encoder");
    rtph264pay = gst_element_factory_make("rtph264pay", "payloader"); // must configure payload?

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new("test-pipeline");

    if (!pipeline || !source || !udp_sink || !videoconvert || !x264enc || !rtph264pay)
    {
        g_printerr("Not all elements could be created.\n");
        return -1;
    }

    gst_bin_add_many(GST_BIN(pipeline), source, videoconvert, x264enc, rtph264pay, udp_sink, NULL);

    /* Build the pipeline */
    // gst_bin_add_many(GST_BIN(pipeline), source, udp_sink, NULL);
    if (gst_element_link_many(source, videoconvert, x264enc, rtph264pay, udp_sink, NULL) != TRUE)
    {
        g_printerr("Elements could not be linked.\n");
        gst_object_unref(pipeline);
        return -1;
    }

    printf("Pipeline built!\n");

    /* Modify the source's properties */
    g_object_set(source, "pattern", 0, NULL);
    g_object_set(udp_sink, "host", "0.0.0.0", "port", 5000, NULL);

    /* Start playing */
    ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        g_printerr("Unable to set the pipeline to the playing state.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    printf("Started playing!\n");

    /* Wait until error or EOS */
    bus = gst_element_get_bus(pipeline);
    msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                     GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

    // TO-DO: Is this part necessary?
    /* Parse message */
    if (msg != NULL)
    {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE(msg))
        {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(msg, &err, &debug_info);
            g_printerr("Error received from element %s: %s\n",
                       GST_OBJECT_NAME(msg->src), err->message);
            g_printerr("Debugging information: %s\n",
                       debug_info ? debug_info : "none");
            g_clear_error(&err);
            g_free(debug_info);
            break;
        case GST_MESSAGE_EOS:
            g_print("End-Of-Stream reached.\n");
            break;
        default:
            /* We should not reach here because we only asked for ERRORs and EOS */
            g_printerr("Unexpected message received.\n");
            break;
        }
        gst_message_unref(msg);
    }

    /* Free resources */
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}