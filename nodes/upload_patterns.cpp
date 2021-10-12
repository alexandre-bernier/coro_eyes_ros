#include <coro_eyes_sdk.h>
#include <ros/ros.h>
#include <thread>


const std::string param_config_path = "~config_path";


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
 * @brief Prints the firmware upload progress.
 * @details Needs to be called in a separate thread before starting the firmware upload.
 * @param projector Pointer to a dlp::LCr4500 projector
 */
void print_firmware_upload_progress(dlp::LCr4500 *projector)
{
    // Write first message
    std::cout << "Uploading: 0%" << std::flush;

    // Give time for the firmware upload to start
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int progress = 0;
    do {
        // Sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Print progress
        std::cout << "\rUploading: " << projector->GetFirmwareUploadPercentComplete() << "% " << std::flush;
        switch(progress++) {
            case 0:
                std::cout << "|" << std::flush;
                break;
            case 1:
                std::cout << "/" << std::flush;
                break;
            case 2:
                std::cout << "—" << std::flush;
                break;
            case 3:
                std::cout << "\\" << std::flush;
                progress = 0;
                break;
        }
    } while(projector->FirmwareUploadInProgress());

    // Upload complete
    std::cout << "\nUpload done.\n\n" << std::flush;
}


int main(int argc, char** argv)
{
    std::cout << std::endl;

    // Variables
    std::string config_path = "/home/alexandre/catkin_ws/src/coro_eyes_ros/config";

    // Init ROS
    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle node;


    // Load values from param server
//    if(!ros::param::get(param_config_path, config_path)) {
//        ROS_ERROR("Couldn't find the config path parameter (%s).\n", param_config_path.c_str());
//        ros::shutdown();
//        return -1;
//    }


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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    print_dlp_errors(projector.StopPatternSequence());


    // Generate projected patterns
    ROS_INFO("Generating projected patterns...\n");
    bool upload_patterns = true;
    dlp::Parameters upload_patterns_param;
    upload_patterns_param.Set(dlp::DLP_Platform::Parameters::SequencePrepared(!upload_patterns));
    projector.Setup(upload_patterns_param);

    unsigned int proj_height;
    projector.GetRows(&proj_height);
    unsigned int proj_width;
    projector.GetColumns(&proj_width);
    StructuredLight structured_light(proj_height, proj_width);
    structured_light.generate_gray_code_patterns();


    // Visualize projected patterns
    int ans;
    do {
        std::cout << "Visualize generated patterns? (Y/N)";
        std::cin.clear();
        ans = std::cin.get();
    } while(ans != 'y' && ans != 'Y' && ans != 'n' && ans != 'N');
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << std::endl;

    if(ans == 'y' || ans == 'Y')
        structured_light.visualize_patterns();


    // Upload projected patterns
    ROS_INFO("Uploading projected patterns to the projector...\n");
    std::thread print_progress_thread(&print_firmware_upload_progress, &projector);
    print_progress_thread.detach();

    dlp::Pattern::Sequence dlp_pattern_sequence;
    dlp_pattern_sequence = convert_gray_code_cv_patterns_to_dlp(structured_light.get_pattern_images());
    print_dlp_errors(projector.PreparePatternSequence(dlp_pattern_sequence));
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));    // Wait to allow the print_progress_thread to finish properly


    // Project patterns
    ans = 0;
    do {
        std::cout << "Project pattern sequence? (Y/N)";
        std::cin.clear();
        ans = std::cin.get();
    } while(ans != 'y' && ans != 'Y' && ans != 'n' && ans != 'N');
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << std::endl;

    if(ans == 'y' || ans == 'Y') {
        ROS_INFO("Starting projection...");
        projector.StartPatternSequence(0, structured_light.get_nb_patterns(), false);

        dlp::DLP_Platform::Parameters::SequencePeriod sequence_period;
        param.Get(&sequence_period);
        unsigned int sequence_duration = sequence_period.Get() * structured_light.get_nb_patterns();
        std::this_thread::sleep_for(std::chrono::microseconds((unsigned int)((float)sequence_duration*1.2)));
        ROS_INFO("Done.\n");

        print_dlp_errors(projector.ProjectSolidBlackPattern());
        print_dlp_errors(projector.StopPatternSequence());
    }


    // Wait for user input
    std::cout << "Press 'Enter' to quit...\n";
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');


    // Disconnect from projector
    ROS_INFO("Disconnecting from projector...\n");
    print_dlp_errors(projector.StopPatternSequence());
    print_dlp_errors(projector.Disconnect());


    // Close the node and exit
    ros::shutdown();
    return 0;
}