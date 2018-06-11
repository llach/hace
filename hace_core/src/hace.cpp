#include "ros/ros.h"

// OpenPose dependencies
#include <openpose/headers.hpp>

// custom ros image producer that listens to a sensor_msgs/Image topic
#include "RosImageProducer.h"

// takes care of ros output
#include "RosWorkerOutput.h"

// program flags are out-sourced for readability
#include "flag_defines.cpp"


int main (int argc, char** argv){
    ros::init(argc, argv, "hace");
    ros::NodeHandle n("~");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    std::string rgb_topic, depth_topic, output_topic;

    n.param("rgb_topic", rgb_topic, std::string("/image_raw"));
    n.param("depth_topic", depth_topic, std::string("/camera/depth_registered/image_raw"));
    n.param("output_topic", output_topic, std::string("/hace/image"));

    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    // Applying user defined configuration - Google flags to program variables
    // outputSize
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");

    // create ros producer
    std::shared_ptr<op::Producer> producer_ptr;
    producer_ptr.reset(new op::RosImageProducer(rgb_topic));

    // poseModel
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    // JSON saving
    const auto writeJson = (!FLAGS_write_json.empty() ? FLAGS_write_json : FLAGS_write_keypoint_json);
    if (!FLAGS_write_keypoint.empty() || !FLAGS_write_keypoint_json.empty())
        op::log("Flags `write_keypoint` and `write_keypoint_json` are deprecated and will eventually be removed."
                " Please, use `write_json` instead.", op::Priority::Max);
    // keypointScale
    const auto keypointScale = op::flagsToScaleMode(FLAGS_keypoint_scale);
    // heatmaps to add
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScale = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    // >1 camera view?
    const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
    // Enabling Google Logging
    const bool enableGoogleLogging = true;
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    // OpenPose wrapper
    op::log("Configuring OpenPose wrapper...", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    op::Wrapper<std::vector<op::Datum>> opWrapper;
    // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
    const op::WrapperStructPose wrapperStructPose{!FLAGS_body_disable, netInputSize, outputSize, keypointScale,
                                                  FLAGS_num_gpu, FLAGS_num_gpu_start, FLAGS_scale_number,
                                                  (float)FLAGS_scale_gap,
                                                  op::flagsToRenderMode(FLAGS_render_pose, multipleView),
                                                  poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose,
                                                  (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, FLAGS_model_folder,
                                                  heatMapTypes, heatMapScale, FLAGS_part_candidates,
                                                  (float)FLAGS_render_threshold, FLAGS_number_people_max,
                                                  enableGoogleLogging, FLAGS_3d, FLAGS_3d_min_views,
                                                  FLAGS_identification, FLAGS_tracking};

    // Producer (use default to disable any input)
    const op::WrapperStructInput wrapperStructInput{producer_ptr, FLAGS_frame_first, FLAGS_frame_last,
                                                    FLAGS_process_real_time, FLAGS_frame_flip, FLAGS_frame_rotate,
                                                    FLAGS_frames_repeat};

    // Consumer (comment or use default argument to disable any output)
    const op::WrapperStructOutput wrapperStructOutput{op::DisplayMode::NoDisplay,
                                                      !FLAGS_no_gui_verbose, FLAGS_fullscreen, FLAGS_write_keypoint,
                                                      op::stringToDataFormat(FLAGS_write_keypoint_format),
                                                      writeJson, FLAGS_write_coco_json,
                                                      FLAGS_write_images, FLAGS_write_images_format, FLAGS_write_video,
                                                      FLAGS_camera_fps, FLAGS_write_heatmaps,
                                                      FLAGS_write_heatmaps_format, FLAGS_write_coco_foot_json};


    // Initializing ros output
    auto rosOutput = std::make_shared<op::RosWorkerOutput>();
    rosOutput->init(output_topic);

    // Add custom processing
    const auto workerOutputOnNewThread = true;
    opWrapper.setWorkerOutput(rosOutput, workerOutputOnNewThread);


    // Configure wrapper
    opWrapper.configure(wrapperStructPose, op::WrapperStructFace{}, op::WrapperStructHand{}, wrapperStructInput,
                         wrapperStructOutput);


    op::log("Starting thread(s)...", op::Priority::High);

    // Start, run & stop threads
    opWrapper.exec();  // It blocks this thread until all threads have finished


    op::log("Hace done.", op::Priority::High);

}