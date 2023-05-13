// image debugging stuff
// ideally be able to repeatedly call a function with an image and have it accessable
// via a stream on a web interface, may also want to log things
// TODO: better name for this file

#include "streamer.hpp"


Streamer& Streamer::getInstance(){
    // https://stackoverflow.com/a/1008086
    static Streamer inst;
    return inst;
}

void Streamer::imshow(std::string name, const cv::Mat& image){
    Streamer inst = Streamer::getInstance();
    inst.showImage(name, image);
}

Streamer::Streamer(){
    GstElement *pipeline, *source, *convert, *enc, *sink;

    source = gst_element_factory_make ("v4l2src", "source");
    convert = gst_element_factory_make("videoconvert", "converter");
    enc = gst_element_factory_make("", "mux");
    sink = gst_element_factory_make ("hlsksink", "sink");

    /* Create the empty pipeline */
    pipeline = gst_pipeline_new ("test-pipeline");

    GstElement* els[] = {pipeline, source, sink, enc, timer, mux};
    for(int i = 0 ; i < 6; i++){
        if(!els[i]){
            printf("couldn't create el %d", i);
            return;
        }
    }
    if (!pipeline || !source || !sink || !timer || !enc || !mux) {
        g_printerr ("Not all elements could be created.\n");
        return;
    }
    gst_bin_add_many (GST_BIN (pipeline), source, timer, sink, NULL);

    if (
           !gst_element_link (source, timer)
        || !gst_element_link (timer, enc)
        || !gst_element_link (enc, mux)
        || !gst_element_link (mux, sink)
    ) {
        g_printerr ("Couldn't do links\n");
        gst_object_unref (pipeline);
        return;
    }

    g_object_set (source, "pattern", 0, "is-live", true, NULL);
    // g_object_set(sink, "port", 9000, NULL);

    /* Start playing */
    puts("starting");
    GstStateChangeReturn ret = gst_element_set_state (pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr ("Unable to set the pipeline to the playing state.\n");
        gst_object_unref (pipeline);
        return;
    }

    /* Wait until error or EOS */
    GstBus* bus = gst_element_get_bus (pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered (
        bus,
        GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)
    );
    
    
    if (msg != NULL) {
        GError *err;
        gchar *debug_info;

        switch (GST_MESSAGE_TYPE (msg)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error (msg, &err, &debug_info);
            g_printerr ("Error received from element %s: %s\n",
                GST_OBJECT_NAME (msg->src), err->message);
            g_printerr ("Debugging information: %s\n",
                debug_info ? debug_info : "none");
            g_clear_error (&err);
            g_free (debug_info);
            break;
        case GST_MESSAGE_EOS:
            g_print ("End-Of-Stream reached.\n");
            break;
        default:
            /* We should not reach here because we only asked for ERRORs and EOS */
            g_printerr ("Unexpected message received.\n");
            break;
        }
        gst_message_unref (msg);
    }

    /* Free resources */
    gst_object_unref (bus);
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);
}