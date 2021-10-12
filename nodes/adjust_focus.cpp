#include <coro_eyes_sdk.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>


const std::string param_config_path = "~config_path";
const std::string param_left_image_topic = "~left_image_topic";
const std::string param_right_image_topic = "~right_image_topic";

const float image_scale = 1;


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


int main(int argc, char** argv)
{
    std::cout << std::endl;

    // Variables
    std::string config_path;
    std::string left_image_topic;
    std::string right_image_topic;

    // Init ROS
    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle node;


    // Load values from param server
    if(!ros::param::get(param_config_path, config_path)) {
        ROS_ERROR("Couldn't find the config path parameter (%s).\n", param_config_path.c_str());
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
    dlp::LCr4500 projector; // Instance of the projector (DLP)
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

    dlp::Pattern::Sequence dlp_pattern_sequence;
    dlp_pattern_sequence = convert_gray_code_cv_patterns_to_dlp(structured_light.get_pattern_images());
    projector.PreparePatternSequence(dlp_pattern_sequence);


    // Project a pattern to help adjust focus
    ROS_INFO("Projecting a pattern...\n");
    projector.StartPatternSequence(12, 1, true);


    // Connect to the cameras
    ROS_INFO("Connecting to the cameras...");
    unsigned int num_cameras = Camera::get_num_available_cameras();
    ROS_INFO("Number of available cameras: %i/2", num_cameras);
    if(num_cameras != 2) {
        ROS_ERROR("Please make sure the CoRo Eyes' cameras are connected.\n");
        ros::shutdown();
        return -1;
    }

    FlyCapture2::PGRGuid guid[num_cameras];
    Camera camera[num_cameras];
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
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
    int camL_index, camR_index;     // Index of the left and right camera
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
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
    }
    ROS_INFO("Configuration complete.\n");


    // Create image publishers and start camera capture
    ROS_INFO("Starting camera capture...\n");
    image_transport::ImageTransport it(node);
    image_transport::Publisher camera_new_frame_pub[num_cameras];
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
        // Camera feed
        camera_new_frame_pub[i_cam] = it.advertise((i_cam == camL_index) ? left_image_topic : right_image_topic, 1);
        camera[i_cam].set_new_frame_callback(image_publisher, &camera_new_frame_pub[i_cam]);
        camera[i_cam].start_capture();
    }


    // Wait for user input
    std::cout << "Press 'Enter' to quit...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    // Disconnect from projector
    ROS_INFO("Disconnecting from projector...\n");
    print_dlp_errors(projector.StopPatternSequence());
    print_dlp_errors(projector.Disconnect());


    // Stop camera capture
    ROS_INFO("Stopping camera capture...\n");
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
        camera[i_cam].stop_capture();
    }


    // Disconnect from cameras
    ROS_INFO("Disconnecting from the cameras...\n");
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
        camera[i_cam].disconnect();
    }


    // Close the node and exit
    ros::shutdown();
    return 0;
}