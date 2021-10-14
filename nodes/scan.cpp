#include <coro_eyes_sdk.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <thread>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "coro_eyes_ros/Scan.h"


const std::string param_config_path = "~config_path";
const std::string param_show_camera_feed = "~show_camera_feed";
const std::string param_left_image_topic = "~left_image_topic";
const std::string param_right_image_topic = "~right_image_topic";

const float image_scale = 0.4;
const unsigned int num_required_cameras = 2;

ros::Publisher point_cloud_pub;

dlp::LCr4500 projector; // Instance of the projector (DLP)
unsigned int proj_sequence_duration = 0;
StructuredLight *ref_structured_light = nullptr;

Camera camera[num_required_cameras];
unsigned int num_avail_cameras = 0;
int camL_index, camR_index;     // Index of the left and right camera
bool show_camera_feed;

cv::Size image_size;
Calibration::StereoData stereo_calib_data;
Calibration::ReprojMaps reprojection_maps[num_required_cameras];

const float ambient_light_shutter_speed = 10.0;
float proj_light_shutter_speed[num_required_cameras];


/**
 * @brief Prints errors and warnings if there is any.
 * @param err The dlp::ReturnCode to print
 */
void print_dlp_errors(const dlp::ReturnCode &err)
{
    unsigned int i;
    if(err.hasErrors()) {
        for(i=0; i<err.GetErrorCount(); i++) {
            ROS_ERROR("Error: %s\n", err.GetErrors().at(i).c_str());
        }
    }

    if(err.hasWarnings()) {
        for(i=0; i<err.GetWarningCount(); i++) {
            ROS_ERROR("Warning: %s\n", err.GetWarnings().at(i).c_str());
        }
    }
}


/**
 *  @brief Callback that publishes an OpenCV image.
 * 	@details Use cv_bridge to convert the OpenCV image to ROS image message.
 *  @param frame: The image to publish
 *  @param callback_data: Publisher of type image_transport::Publisher
 */
void image_publisher(cv::Mat frame, void *callback_data)
{
    // Callback data
    image_transport::Publisher pub = *(image_transport::Publisher*)callback_data;

    // Resize the image
    cv::Mat resized_frame;
    cv::resize(frame, resized_frame, cv::Size(0,0), image_scale, image_scale);

    // Convert OpenCV to ROS image
    cv_bridge::CvImage cv_frame(std_msgs::Header(), resized_frame.channels() == 1 ? "mono8" : "rgb8", resized_frame);

    // Publish the image
    pub.publish(cv_frame.toImageMsg());
}


/**
 *  @brief Scan service callback.
 */
bool scan(coro_eyes_ros::Scan::Request &req, coro_eyes_ros::Scan::Response &res)
{
    // Timing the execution of the scan
    ros::WallTime start, end;
    start = ros::WallTime::now();

    // If we were showing the camera feed, change settings to see with projector light
    if(show_camera_feed) {
        for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
            camera[i_cam].set_camera_trigger(true);
            camera[i_cam].set_shutter_speed(proj_light_shutter_speed[i_cam]);
        }
    }

    ROS_INFO("Starting a new scan...");

    // Make sure the first buffered image will be dark (IMPORTANT)
    print_dlp_errors(projector.ProjectSolidBlackPattern());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Start camera buffering
    for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
        camera[i_cam].start_buffering();
    }

    // Start pattern sequence projection
    projector.StartPatternSequence(0, ref_structured_light->get_nb_patterns(), false);

    // Wait for completion
    std::this_thread::sleep_for(std::chrono::microseconds((unsigned int)(proj_sequence_duration * 1.2)));

    // Stop camera buffering
    for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
        camera[i_cam].stop_buffering();
    }

    // Stop projection
    print_dlp_errors(projector.ProjectSolidBlackPattern());
    print_dlp_errors(projector.StopPatternSequence());

    // Retrieve camera images from buffer
    std::vector<std::vector<cv::Mat> > captured_patterns;
    captured_patterns.resize(num_avail_cameras);
    for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
        if(!ref_structured_light->extract_pattern_images(camera[i_cam].get_image_buffer_content(), captured_patterns[i_cam])) {
            ROS_ERROR("Couldn't find all the patterns in the captured images.\n");
            return false;
        }
    }

    // Remap images
    std::vector<std::vector<cv::Mat> > remapped_images;
    remapped_images.resize(num_avail_cameras);
    if(!Calibration::remap_images(captured_patterns[camR_index], reprojection_maps[camR_index], remapped_images[0])) {
        ROS_ERROR("Error while remapping the captured patterns of the right camera.\n");
        return false;
    }
    if(!Calibration::remap_images(captured_patterns[camL_index], reprojection_maps[camL_index], remapped_images[1])) {
        ROS_ERROR("Error while remapping the captured patterns of the left camera.\n");
        return false;
    }

    // Compute disparity
    cv::Mat disparity_map;
    if(!ref_structured_light->compute_disparity_map(remapped_images, disparity_map)) {
        ROS_ERROR("Error while computing the disparity map.\n");
        return false;
    }

    // Compute point cloud
    std::vector<cv::Point3f> point_cloud = ref_structured_light->compute_point_cloud(disparity_map, stereo_calib_data.Q);

    // Convert OpenCV point cloud to ROS sensor_msgs/PointCloud2
    res.point_cloud.header = std_msgs::Header();
    res.point_cloud.header.frame_id = "coro_eyes";
    res.point_cloud.height = 1;
    res.point_cloud.width  = point_cloud.size();
    res.point_cloud.is_bigendian = false;
    res.point_cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier pcd_modifier(res.point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(res.point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(res.point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(res.point_cloud, "z");

    for(auto pt=point_cloud.begin(); pt!=point_cloud.end(); ++pt, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = (*pt).x;
        *iter_y = (*pt).y;
        *iter_z = (*pt).z;
    }

    // If we were showing the camera feed, change settings to see with ambient light
    if(show_camera_feed) {
        for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
            camera[i_cam].set_camera_trigger(false);
            camera[i_cam].set_shutter_speed(ambient_light_shutter_speed);
        }
    }

    // Timing the execution of the scan
    end = ros::WallTime::now();
    double execution_time = (end - start).toNSec() * 1e-6;
    ROS_INFO("Scan completed in %.2fms.\n", execution_time);

    point_cloud_pub.publish(res);

    return true;
}


int main(int argc, char** argv) {
    std::cout << std::endl;

    // Variables
    std::string config_path;
    std::string left_image_topic;
    std::string right_image_topic;

    // Init ROS
    ros::init(argc, argv, "scan");
    ros::NodeHandle node;
    point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("point_cloud_raw", 1);


    // Load values from param server
    if(!ros::param::get(param_config_path, config_path)) {
        ROS_ERROR("Couldn't find the config path parameter (%s).\n", param_config_path.c_str());
        ros::shutdown();
        return -1;
    }

    if(!ros::param::get(param_show_camera_feed, show_camera_feed)) {
        ROS_ERROR("Couldn't find the show camera feed parameter (%s).\n", param_show_camera_feed.c_str());
        ros::shutdown();
        return -1;
    }

    if(!ros::param::get(param_left_image_topic, left_image_topic)) {
        ROS_ERROR("Couldn't find the left image topic parameter (%s).\n", param_left_image_topic.c_str());
        ros::shutdown();
        return -1;
    }

    if(!ros::param::get(param_right_image_topic, right_image_topic)) {
        ROS_ERROR("Couldn't find the right image topic parameter (%s).\n", param_right_image_topic.c_str());
        ros::shutdown();
        return -1;
    }


    // Connect to the projector
    dlp::ReturnCode ret;    // Return variable of all DLP's methods
    ROS_INFO("Connecting to the projector...\n");
    ret = projector.Connect("");
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        ros::shutdown();
        return -1;
    }


    // Setup the projector
    ROS_INFO("Loading projector parameters...\n");
    dlp::Parameters param;
    std::string proj_param_file = config_path + "/dlp_platforms/projector_settings.txt";
    ret = param.Load(proj_param_file);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        ros::shutdown();
        return -1;
    }

    ROS_INFO("Setting up projector...\n");
    ret = projector.Setup(param);
    print_dlp_errors(ret);
    if(ret.hasErrors()) {
        ros::shutdown();
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    print_dlp_errors(projector.ProjectSolidBlackPattern());
    print_dlp_errors(projector.StopPatternSequence());


    // Connect to the cameras
    ROS_INFO("Connecting to the cameras...");
    num_avail_cameras = Camera::get_num_available_cameras();
    ROS_INFO("Number of available cameras: %i/%i", num_avail_cameras, num_required_cameras);
    if(num_avail_cameras != num_required_cameras) {
        ROS_ERROR("Please make sure that only the CoRo Eyes' cameras are connected.\n");
        ros::shutdown();
        return -1;
    }

    FlyCapture2::PGRGuid guid[num_avail_cameras];
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        if(Camera::get_guid(i_cam, &guid[i_cam])) {
            ROS_ERROR("Can't get GUID of camera %i.\n", i_cam);
            ros::shutdown();
            return -1;
        }

        if(camera[i_cam].connect(&guid[i_cam]) != FlyCapture2::PGRERROR_OK) {
            ROS_ERROR("Can't connect to camera %i.\n", i_cam);
            ros::shutdown();
            return -1;

        }
        ROS_INFO("[Camera %i] SN = %i", i_cam, camera[i_cam].get_serial_number());
    }
    ROS_INFO("Cameras connected.\n");


    // Configure the cameras
    ROS_INFO("Configuring cameras...");
    Camera::CameraPosition camera_position = Camera::CameraPosition::Undefined;
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        if(camera[i_cam].configure()) {
            ROS_ERROR("Can't configure camera %i.\n", i_cam);
            ros::shutdown();
            return -1;
        }

        switch(camera[i_cam].get_serial_number()) {
            case 19153384:  // Serial number of the left camera
                camera_position = Camera::CameraPosition::Left;
                camL_index = i_cam;
                break;

            case 19305617:  // Serial number of the right camera
                camera_position = Camera::CameraPosition::Right;
                camR_index = i_cam;
                break;

            default:
                ROS_ERROR("Unrecognized camera (%i).\n", camera[i_cam].get_serial_number());
                ros::shutdown();
                return -1;

        }

        if(camera[i_cam].set_properties_for_coro_eyes(camera_position)) {
            ROS_ERROR("Can't configure camera %i.\n", i_cam);
            ros::shutdown();
            return -1;
        }

        // Get projector light shutter speed
        FlyCapture2::Property property;
        camera[i_cam].get_property(FlyCapture2::PropertyType::SHUTTER, &property);
        proj_light_shutter_speed[i_cam] = property.absValue;

        // If we show camera feed, make sure the camera can work without the projector
        if(show_camera_feed) {
            camera[i_cam].set_camera_trigger(false);
            camera[i_cam].set_shutter_speed(10.0);
        }
    }
    ROS_INFO("Configuration complete.\n");


    // Load calibration data
    Calibration::Data camera_calib_data[num_avail_cameras];
    std::string file_name;
    std::string cam_calib_data_file_path = config_path + "/calibration/data/camera_";  // Path to the cameras calibration data files
    std::string stereo_calib_data_file_path = config_path + "/calibration/data/stereo_";   // Path to stereo camera calibration data file
    std::string calib_data_file_extension = ".xml";

    // Load camera calibration data
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        file_name = cam_calib_data_file_path + std::to_string(camera[i_cam].get_serial_number()) + calib_data_file_extension;
        if(!Calibration::load_camera_calibration(file_name, camera_calib_data[i_cam])) {
            ROS_ERROR("Error loading camera calibration data: %s\n", file_name.c_str());
            ros::shutdown();
            return -1;
        }
    }

    // Load stereo calibration data
    if(camL_index == 0) {
        file_name = stereo_calib_data_file_path + std::to_string(camera[0].get_serial_number()) + "_" +
                    std::to_string(camera[1].get_serial_number()) + calib_data_file_extension;
    }

    else {
        file_name = stereo_calib_data_file_path + std::to_string(camera[1].get_serial_number()) + "_" +
                    std::to_string(camera[0].get_serial_number()) + calib_data_file_extension;
    }

    if(!Calibration::load_stereo_calibration(file_name, stereo_calib_data)) {
        ROS_ERROR("Error loading stereo calibration data: %s\n", file_name.c_str());
        ros::shutdown();
        return -1;
    }


    // Compute reprojection maps
    image_size = cv::Size(camera[0].get_camera_width(), camera[0].get_camera_height());
    Calibration::calculate_stereo_reproj_maps(camera_calib_data[camL_index], camera_calib_data[camR_index], stereo_calib_data,
                                              image_size, reprojection_maps[camL_index], reprojection_maps[camR_index]);


    // Generate projected patterns
    ROS_INFO("Generating projected patterns...\n");
    bool upload_patterns = false;
    dlp::Parameters upload_patterns_param;
    upload_patterns_param.Set(dlp::DLP_Platform::Parameters::SequencePrepared(!upload_patterns));
    projector.Setup(upload_patterns_param);

    unsigned int proj_height;
    projector.GetRows(&proj_height);
    unsigned int proj_width;
    projector.GetColumns(&proj_width);
    StructuredLight structured_light(proj_height, proj_width);
    structured_light.generate_gray_code_patterns();
    ref_structured_light = &structured_light;

    dlp::Pattern::Sequence dlp_pattern_sequence;
    dlp_pattern_sequence = convert_gray_code_cv_patterns_to_dlp(structured_light.get_pattern_images());
    print_dlp_errors(projector.PreparePatternSequence(dlp_pattern_sequence));

    // Get how long it takes to project all patterns
    dlp::DLP_Platform::Parameters::SequencePeriod sequence_period;
    param.Get(&sequence_period);
    proj_sequence_duration = sequence_period.Get() * structured_light.get_nb_patterns();


    // Create image publishers and start camera capture
    ROS_INFO("Starting camera capture...\n");
    image_transport::ImageTransport it(node);
    image_transport::Publisher camera_new_frame_pub[num_avail_cameras];
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        // Camera feed
        if(show_camera_feed) {
            camera_new_frame_pub[i_cam] = it.advertise((i_cam == camL_index) ? left_image_topic : right_image_topic, 1);
            camera[i_cam].set_new_frame_callback(image_publisher, &camera_new_frame_pub[i_cam]);
        }
        camera[i_cam].start_capture();
    }


    // Set up the cameras' buffer
    unsigned int nb_images = structured_light.get_nb_patterns() * 1.2;
    for(unsigned int i_cam=0; i_cam<num_avail_cameras; i_cam++) {
        camera[i_cam].set_image_buffer(nb_images);
    }


    // Create service now that the CoRo Eyes is ready
    ros::ServiceServer service = node.advertiseService("Scan", scan);
    ROS_INFO("CoRo Eyes is ready to scan.\n");


    // Loop
    ros::spin();


    // Disconnect from projector
    ROS_INFO("Disconnecting from projector...\n");
    print_dlp_errors(projector.Disconnect());


    // Stop camera capture
    ROS_INFO("Stopping camera capture...\n");
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        camera[i_cam].stop_capture();
    }


    // Disconnect from cameras
    ROS_INFO("Disconnecting from the cameras...\n");
    for(unsigned int i_cam=0; i_cam < num_avail_cameras; i_cam++) {
        camera[i_cam].disconnect();
    }


    // Close the node and exit
    ros::shutdown();
    return 0;
}