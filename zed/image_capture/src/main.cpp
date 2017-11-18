 // Standard includes
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include (for display)
#include "opencv2/opencv.hpp"




// Using std and sl namespaces
using namespace std;
using namespace sl;

int main(int argc, char **argv) {
    //check if dir exists
    struct stat st = {0};
    if (stat("../images", &st) == -1) {
        mkdir("../images", 0700);
    }


    // Create a ZED Camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    //init_params.camera_resolution = RESOLUTION_VGA; // HD1080, HD720, VGA
    init_params.camera_resolution = RESOLUTION_HD720; // HD1080, HD720, VGA

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << errorCode2str(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Print camera information
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create a Mat to store images
    Mat zed_image_left;
    Mat zed_image_right;


    // Check that grab() is successful
    if (zed.grab() == SUCCESS) {
        // Retrieve image
        zed.retrieveImage(zed_image_left, VIEW_LEFT);
        zed.retrieveImage(zed_image_right, VIEW_RIGHT);
        
        time_t rawtime;
        time (&rawtime);
        std::string str = string("../images/") + std::to_string(rawtime);
//        std::cout << str << std::endl;
        // Save image with OpenCV
        cv::imwrite(str + "_left.jpg", cv::Mat((int) zed_image_left.getHeight(), (int) zed_image_left.getWidth(), CV_8UC4, zed_image_left.getPtr<sl::uchar1>(sl::MEM_CPU)));
        cv::imwrite(str + "_right.jpg", cv::Mat((int) zed_image_right.getHeight(), (int) zed_image_right.getWidth(), CV_8UC4, zed_image_right.getPtr<sl::uchar1>(sl::MEM_CPU)));
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}


