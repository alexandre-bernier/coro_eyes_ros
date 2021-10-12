#include <coro_eyes_sdk.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


const std::string param_config_path = "~config_path";
const std::string param_left_image_topic = "~left_image_topic";
const std::string param_right_image_topic = "~right_image_topic";

const float image_scale = 0.4;


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
	
	
	// Capture all chessboards
	bool good_image = false;
	cv::Mat captured_images[num_cameras];
	std::vector<cv::Point2f> temp_image_points[num_cameras];
	std::vector<std::vector<cv::Point2f>> image_points[num_cameras];

    // OpenCV window name for overlaid chessboard corners
    std::string cv_window_name_overlay[num_cameras];
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
        cv_window_name_overlay[i_cam] = (i_cam == camL_index) ? "Chessboard corners (left)" : "Chessboard corners (right)";
    }
	
	// For every required calibration images...
	for(int i_image=0; i_image<calib_settings.nrFrames; i_image++) {
		good_image = false;
		
		// Until the captured calibration image is good for all cameras
		while(!good_image && ros::ok()) {
			// Wait for user to press a key
			std::cout << "Press 'Enter' to capture an image...\n";
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            // Exit if the node was killed
            if(!ros::ok()) {
                ros::shutdown();
                return 0;
            }
			
			// For every camera...
			ROS_INFO("Trying to find the chessboard...");
			
			for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
				// Save last frame
				captured_images[i_cam] = camera[i_cam].get_last_frame();
				
				// Try to find the chessboard corners
				good_image = Calibration::find_corners(calib_settings, captured_images[i_cam], temp_image_points[i_cam]);
				
				ROS_INFO("[Camera %i] Chessboard found: %s", i_cam, good_image ? "True" : "False");
				
				// If the chessboard can't be found in one of the camera's capture image, drop all of them and retry
				if(!good_image) {
					break;
				}
				
			}
			
		}
		
		// If images are good for all cameras
		if(good_image) {
			ROS_INFO("Drawing chessboard corners...\n");
			for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
				// Save the chessboard corners
				image_points[i_cam].push_back(temp_image_points[i_cam]);
				
				// Convert the captured image to RGB
				cv::cvtColor(captured_images[i_cam], captured_images[i_cam], cv::COLOR_GRAY2RGB);
				
				// Draw the chessboard corners
				cv::drawChessboardCorners(captured_images[i_cam], calib_settings.boardSize, cv::Mat(temp_image_points[i_cam]), good_image);

                // Rescale image
                cv::Mat rescaled_image;
                cv::resize(captured_images[i_cam], rescaled_image, cv::Size(0,0), image_scale, image_scale);

                // Display the overlaid image in a new window
                cv::imshow(cv_window_name_overlay[i_cam], rescaled_image);
                cv::waitKey(100);
			}
			
			ROS_INFO("Calibration image %i of %i acquired.\n", i_image+1, calib_settings.nrFrames);
		}
	}
    cv::destroyAllWindows();


    // Computing single camera calibrations
    ROS_INFO("Running camera calibration...\n");
    cv::Size image_size(camera[0].get_camera_width(), camera[0].get_camera_height());
    Calibration::Data camera_calib_data[num_cameras];
    bool camera_calib_successful[num_cameras];

    // Find the cameras' intrinsic matrix, distortion coefficients and extrinsic matrices
    bool all_camera_calibrated = true;
    for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
        // Calibration
        camera_calib_successful[i_cam] = Calibration::run_camera_calibration(calib_settings, image_size, camera_calib_data[i_cam], image_points[i_cam]);

        // Display the calibration result
        ROS_INFO("Camera %i calibration\n\t%s Avg. re-projection error = %f\n", i_cam,
                 (camera_calib_successful[i_cam] ? "Calibration succeeded." : "Calibration failed."),
                 camera_calib_data[i_cam].total_avg_error);

        // Save calibration if the calibration succeeded for all cameras
        all_camera_calibrated &= camera_calib_successful[i_cam];
    }


    // Save single camera calibrations
    if(all_camera_calibrated) {
        ROS_INFO("Saving camera calibration to file...\n");
        std::string calibration_data_file;

        // For all cameras...
        for(unsigned int i_cam=0; i_cam<num_cameras; i_cam++) {
            // Save calibration data
            calibration_data_file = config_path + "/calibration/data/camera_" + std::to_string(camera[i_cam].get_serial_number()) + ".xml";
            Calibration::save_camera_calibration(calibration_data_file, calib_settings, image_size, camera_calib_data[i_cam], image_points[i_cam]);
            ROS_INFO("Camera %i calibration data: %s\n", i_cam, calibration_data_file.c_str());
        }
    }


    // Compute stereo calibration
    Calibration::StereoData stereo_calib_data;
    bool stereo_calib_successful = false;

    if(all_camera_calibrated) {
        if(num_cameras == 2) {
            ROS_INFO("Running stereo camera calibration...\n");
            // Stereo calibration
            stereo_calib_successful = Calibration::run_stereo_calibration(calib_settings, image_size, stereo_calib_data,
                                                                          camera_calib_data[camL_index], image_points[camL_index],
                                                                          camera_calib_data[camR_index], image_points[camR_index]);

            // Display the calibration result
            ROS_INFO("Stereo calibration\n\t%s Avg. re-projection error = %f\n",
                     (stereo_calib_successful ? "Calibration succeeded." : "Calibration failed."),
                     stereo_calib_data.total_avg_error);
        }
    }


    // Save stereo calibration
    if(stereo_calib_successful) {
        ROS_INFO("Saving stereo camera calibration to file...\n");
        std::string calibration_data_file;

        // Save calibration data
        calibration_data_file = config_path + "/calibration/data/stereo_" +
                                std::to_string(camera[camL_index].get_serial_number()) + "_" +
                                std::to_string(camera[camR_index].get_serial_number()) + ".xml";
        Calibration::save_stereo_calibration(calibration_data_file, calib_settings, image_size, stereo_calib_data, image_points[camL_index], image_points[camR_index]);
        ROS_INFO("Stereo calibration data: %s\n", calibration_data_file.c_str());
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
