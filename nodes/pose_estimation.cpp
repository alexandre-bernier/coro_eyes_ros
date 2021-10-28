#include <coro_eyes_sdk.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


const std::string param_config_path = "~config_path";
const std::string param_left_image_topic = "~left_image_topic";

const float image_scale = 0.5;


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

    // Making sure that opencv windows don't hang
    cv::waitKey(1);
}


int main(int argc, char** argv)
{
    std::cout << std::endl;

    // Variables
    std::string config_path;
    std::string left_image_topic;

    // Init ROS
    ros::init(argc, argv, "pose_estimation");
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


    // Load calibration configuration
    ROS_INFO("Loading the calibration configuration...");
    std::string calib_settings_file = config_path + "/calibration/camera_calibration_settings.xml";
    cv::FileStorage fs(calib_settings_file, cv::FileStorage::READ);
    if(!fs.isOpened()) {
        ROS_ERROR("Could not open the calibration configuration file: %s.\n", calib_settings_file.c_str());
        ros::shutdown();
        return -1;
    }

    Calibration::Settings calib_settings;
    fs["Settings"] >> calib_settings;
    fs.release();
    if(!calib_settings.goodInput) {
        ROS_ERROR("Invalid configuration detected.\n");
        ros::shutdown();
        return -1;
    }
    ROS_INFO("Configuration loaded.\n");


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


    // Make sure the projector isn't projecting
    ROS_INFO("Stopping projection...\n");
    print_dlp_errors(projector.StopPatternSequence());


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

        camera[i_cam].set_camera_trigger(false);
        camera[i_cam].set_shutter_speed(10.0);
    }
    ROS_INFO("Configuration complete.\n");


    // Load calibration data
    int pose_cam_index = camL_index;
    Calibration::Data camera_calib_data[num_cameras];
    Calibration::StereoData stereo_calib_data;
    std::string file_name;
    std::string cam_calib_data_file_path = config_path + "/calibration/data/camera_";  // Path to the cameras' calibration data files
    std::string stereo_calib_data_file_path = config_path + "/calibration/data/stereo_";   // Path to stereo camera calibration data file
    std::string calib_data_file_extension = ".xml";

    // Load camera calibration data
    for(unsigned int i_cam=0; i_cam < num_cameras; i_cam++) {
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


    // Create image publishers and start camera capture
    ROS_INFO("Starting camera capture...\n");
    image_transport::ImageTransport it(node);
    image_transport::Publisher camera_new_frame_pub;
    // Camera feed
    camera_new_frame_pub = it.advertise(left_image_topic, 1);
    camera[pose_cam_index].set_new_frame_callback(image_publisher, &camera_new_frame_pub);
    camera[pose_cam_index].start_capture();


    // Capture chessboard
    bool good_image = false;
    cv::Mat captured_image;
    std::vector<cv::Point2f> temp_image_points;
    std::vector<cv::Point2f> image_points;

    // OpenCV window name for overlaid chessboard corners
    std::string cv_window_name_overlay;
    std::string cv_window_name = "Chessboard corners for camera " + std::to_string(pose_cam_index);

    // Until the captured calibration image is good
    while(!good_image && ros::ok()) {
        // Wait for user to press a key
        std::cout << "Press 'Enter' to capture an image...\n";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // Exit if the node was killed
        if(!ros::ok()) {
            ros::shutdown();
            return 0;
        }

        ROS_INFO("Trying to find the chessboard...");

        // Save last frame
        captured_image= camera[pose_cam_index].get_last_frame();

        // Undistorted the captured image
        cv::Mat undistorted_captured_image;
        cv::undistort(captured_image, undistorted_captured_image, camera_calib_data[pose_cam_index].intrinsic, camera_calib_data[pose_cam_index].distorsion);
        captured_image = undistorted_captured_image;

        // Try to find the chessboard corners
        good_image = Calibration::find_corners(calib_settings, captured_image, temp_image_points);
        ROS_INFO("[Camera %i] Chessboard found: %s", pose_cam_index, good_image ? "True" : "False");

        // If the chessboard can't be found in one of the camera's capture image, drop all of them and retry
        if(!good_image) {
            break;
        }
    }

    // If images are good for all cameras
    if(good_image) {
        ROS_INFO("Drawing chessboard corners...\n");
        // Select the chessboard corners
        image_points.push_back(temp_image_points[0]);
        image_points.push_back(temp_image_points[calib_settings.boardSize.width-1]);
        image_points.push_back(temp_image_points[(calib_settings.boardSize.width*calib_settings.boardSize.height)-calib_settings.boardSize.width]);
        image_points.push_back(temp_image_points[(calib_settings.boardSize.width*calib_settings.boardSize.height)-1]);

        // Convert the captured image to RGB
        cv::cvtColor(captured_image, captured_image, cv::COLOR_GRAY2RGB);

        // Draw the selected chessboard corners
        cv::Scalar red(0, 0, 255);
        for(unsigned int i_point=0; i_point<image_points.size(); i_point++) {
            cv::circle(captured_image, image_points[i_point], 15, red, 3);
            cv::putText(captured_image, std::to_string(i_point+1), cv::Point(image_points[i_point].x+20, image_points[i_point].y-20), cv::FONT_HERSHEY_DUPLEX, 2, red, 2);
        }

        // Rescale image
        cv::Mat rescaled_image;
        cv::resize(captured_image, rescaled_image, cv::Size(0,0), image_scale, image_scale);

        // Display the overlaid image in a new window
        cv::imshow(cv_window_name_overlay, rescaled_image);
        cv::waitKey(100);
    }


    // Get corner coordinates
    std::cout << "This script will find a transform (translation and rotation) from a user-defined reference frame to the CoRo Eyes right camera." << std::endl;
    std::cout << "\t1- Fix a reference frame of your choosing in the camera's environment (it can be anything)." << std::endl;
    std::cout << "\t2- Determine the orientation of that frame (i.e. in which direction does each axis point)." << std::endl;
    std::cout << "\t3- Using that new reference frame axes, measure the distance from that frame to all the chessboard corners identified in the image." << std::endl;

    unsigned int coord_scale = 1;
    std::string coord_scale_units = "m";
    unsigned int coord_scale_resp;
    bool resp_ok = false;

    while(!resp_ok) {
        std::cout << "In which units do you wish to enter the coordinates?" << std::endl;
        std::cout << "\t [1]: Millimeters (mm)" << std::endl;
        std::cout << "\t [2]: Centimeters (cm)" << std::endl;
        std::cout << "\t [3]: Decimeters (dm)" << std::endl;
        std::cout << "\t [4]: Meters (m)" << std::endl;
        std::cout << "--> ";
        std::cin >> coord_scale_resp;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if(coord_scale_resp == 1 || coord_scale_resp == 2 || coord_scale_resp == 3 || coord_scale_resp == 4)
            resp_ok = true;
    }

    switch(coord_scale_resp){
        case 1:
            coord_scale = 1000;
            coord_scale_units = "mm";
            break;
        case 2:
            coord_scale = 100;
            coord_scale_units = "cm";
            break;
        case 3:
            coord_scale = 10;
            coord_scale_units = "dm";
            break;
        case 4:
            coord_scale = 1;
            coord_scale_units = "m";
            break;
    }

    std::vector<cv::Point3f> object_points;
    float corner_x, corner_y, corner_z;

    for(unsigned int i_corner=0; i_corner<image_points.size(); i_corner++) {
        std::cout << std::endl << "Using your user-defined reference frame, what is the distance (" << coord_scale_units << ") between that frame and the corner " << i_corner+1 << std::endl;

        std::cout << "X: ";
        std::cin >> corner_x;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "Y: ";
        std::cin >> corner_y;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::cout << "Z: ";
        std::cin >> corner_z;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        object_points.push_back(cv::Point3f(corner_x/coord_scale, corner_y/coord_scale, corner_z/coord_scale));
    }
    std::cout << std::endl;


    // Run pose estimation
    ROS_INFO("Running pose estimation...\n");
    cv::Size image_size(camera[pose_cam_index].get_camera_width(), camera[pose_cam_index].get_camera_height());
    Calibration::Pose pose_data;
    bool pose_estimation_successful;
    // Pose estimation
    pose_estimation_successful = Calibration::run_pose_estimation(camera_calib_data[pose_cam_index], stereo_calib_data, object_points, image_points,
                                                                  pose_data);
    // Add frame id to pose data
    pose_data.frame = "coro_eyes";


    // Save pose estimation
    if(pose_estimation_successful) {
        std::cout << std::endl << "--- Results ---" << std::endl;
        std::cout << "Translation:" << std::endl;
        std::cout << "\tX = " << pose_data.translation[0] << std::endl;
        std::cout << "\tY = " << pose_data.translation[1] << std::endl;
        std::cout << "\tZ = " << pose_data.translation[2] << std::endl;
        std::cout << "Quaternions:" << std::endl;
        std::cout << "\tX = " << pose_data.quaternions[0] << std::endl;
        std::cout << "\tY = " << pose_data.quaternions[1] << std::endl;
        std::cout << "\tZ = " << pose_data.quaternions[2] << std::endl;
        std::cout << "\tW = " << pose_data.quaternions[3] << std::endl;
        std::cout << std::endl;

        ROS_INFO("Saving pose estimation result to file...\n");
        std::string pose_estimation_file;

        // Save calibration data
        pose_estimation_file = config_path + "/calibration/data/pose_estimation_" + std::to_string(camera[pose_cam_index].get_serial_number()) + ".xml";
        Calibration::save_pose_estimation(pose_estimation_file, pose_data);
        ROS_INFO("Pose estimation: %s\n", pose_estimation_file.c_str());
    }


    // Wait for user input
    std::cout << "Press 'Enter' to quit...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    // Disconnect from projector
    ROS_INFO("Disconnecting from projector...\n");
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