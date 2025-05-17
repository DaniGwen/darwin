/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Object Detection via external Python script
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <vector>     // For storing detection results (parsed from Python)
#include <string>     // For handling labels and filenames
#include <iostream>   // For std::cerr, std::cout
#include <fstream>    // For file operations (saving image)
#include <algorithm>  // For std::max, std::min
#include <cstdio>     // For popen, pclose, fgets, mkstemp, unlink
#include <sstream>  // For parsing string output from Python

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h" // Assuming this provides basic robot control structures

// --- Configuration ---
#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

const char *PYTHON_SCRIPT_PATH = "../../../../aiy-maker-kit/examples/custom_detect_objects.py";
const char *PYTHON_INTERPRETER = "python3"; // Or just "python" depending on your system

// Structure to hold parsed detection results (assuming Python outputs at least label/ID)
// Add more fields (score, bounding box) if your Python script will output them
struct ParsedDetection {
    std::string label;  // Detected class label
    int class_id = -1;  // Detected class ID (if Python outputs ID or you can map label to ID)
    float score = 0.0f; // Confidence score (if Python outputs score)
    // Add bounding box if Python outputs it and you need it for drawing/tracking:
    // float xmin, ymin, xmax, ymax; // Bounding box (normalized 0.0-1.0)
};

void change_current_dir()
{
    char exepath[1024] = {0};
    if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// TODO: Implement a function to save the current camera frame to a temporary file.
// This is a critical part. Saving in a standard format like JPG or PNG is recommended.
// Requires an image library (e.g., integrate stb_image_write.h or use OpenCV if available).
// This example provides a basic (and inefficient/large) PPM writer.
// Returns the path to the saved file, or an empty string on failure.
std::string save_frame_to_temp_file(Image *frame)
{
    if (!frame || !frame->m_ImageData || frame->m_PixelSize != 3) // Assuming 3 channels (RGB)
    {
        std::cerr << "Error: Invalid frame data or not RGB for saving." << std::endl;
        return "";
    }

    // --- Debugging Steps ---
    // char temp_filename_template[] = "/tmp/frame_XXXXXX.ppm"; // Original template
    char temp_filename_template[] = "/tmp/XXXXXX";         // Try the absolute simplest template
    std::cerr << "DEBUG: Attempting to create temp file with template: '" << temp_filename_template << "'" << std::endl;
    // --- End Debugging Steps ---


    int fd = mkstemp(temp_filename_template); // mkstemp modifies the string IN PLACE
    if (fd < 0)
    {
        int temp_errno = errno; // Capture errno immediately
        std::cerr << "Error creating temp file using template '" << temp_filename_template << "': "
                  << strerror(temp_errno) << " (errno " << temp_errno << ")" << std::endl;
        return "";
    }

    // --- Debugging Step ---
    std::cerr << "DEBUG: Successfully created temp file: '" << temp_filename_template << "' with fd: " << fd << std::endl;
    // --- End Debugging Step ---


    // Associate a FILE* stream with the file descriptor
    FILE *temp_file = fdopen(fd, "wb"); // Open in write binary mode
    if (!temp_file)
    {
        int temp_errno = errno; // Capture errno immediately
        std::cerr << "Error opening temp file stream for '" << temp_filename_template << "': "
                  << strerror(temp_errno) << " (errno " << temp_errno << ")" << std::endl;
        close(fd); // Close the file descriptor obtained from mkstemp
        // Using unlink to clean up the file immediately in case of fopen error
        unlink(temp_filename_template);
        return "";
    }

    // Write PPM header (P6 format)
    fprintf(temp_file, "P6\n%d %d\n255\n", frame->m_Width, frame->m_Height);

    // Write pixel data
    size_t data_size = frame->m_NumberOfPixels * frame->m_PixelSize;
    if (fwrite(frame->m_ImageData, 1, data_size, temp_file) != data_size)
    {
        int temp_errno = errno; // Capture errno immediately
        std::cerr << "Error writing data to temp file '" << temp_filename_template << "': "
                  << strerror(temp_errno) << " (errno " << temp_errno << ")" << std::endl;
        fclose(temp_file);
        // Using unlink to clean up the file immediately in case of write error
        unlink(temp_filename_template);
        return "";
    }

    fclose(temp_file); // Close the FILE* stream (also closes the fd)

    // std::cout << "Saved frame to: " << temp_filename_template << std::endl; // Debug print


    return temp_filename_template; // Return the modified string with the unique name
}

// TODO: Implement a function to parse the output from the Python script.
// This needs to match the EXACT format your MODIFIED Python script prints to stdout.
// Assumes Python prints the label (and optionally score/ID) on a single line and exits.
ParsedDetection parse_python_output(FILE *python_output)
{
    ParsedDetection detection;
    char buffer[256]; // Buffer to read output line

    // Read one line of output from the pipe
    if (fgets(buffer, sizeof(buffer), python_output) != NULL)
    {
        // Attempt to parse the line.
        // This basic example assumes Python prints just the label string.
        // If Python prints more (like "label score"), adjust sscanf or use stringstream.

        // Example parsing for just a label string:
        std::string output_line = buffer;
        // Trim potential trailing newline or whitespace from the label
        size_t last = output_line.find_last_not_of(" \t\n\r\f\v");
        if (std::string::npos != last) {
            output_line = output_line.substr(0, last + 1);
        } else {
            output_line.clear(); // Handle case with only whitespace
        }

        detection.label = output_line;

        // If Python prints "label score":
        // std::stringstream ss(buffer);
        // if (ss >> detection.label >> detection.score) {
        //   // Successfully parsed label and score
        // } else {
        //   std::cerr << "Warning: Could not parse label and score from Python output: " << buffer << std::endl;
        //   detection.label = "Parse Error"; // Indicate parsing failure
        // }

        // If Python prints more, adjust parsing logic here.
    }
    else
    {
        // Failed to read from pipe (e.g., Python script printed nothing or crashed)
        std::cerr << "Warning: No output or error reading from Python script pipe." << std::endl;
        detection.label = "No Detection"; // Indicate no output
    }

    return detection;
}

// Drawing function remains, but we'll use the ParsedDetection struct
// This version assumes we only have label, potentially score, but no bounding box
// If Python outputs bounding boxes, update ParsedDetection and this function.
// This simple version just prints the label.
void DrawDetectionInfo(Image *image, const ParsedDetection &detection)
{
    if (!image || !image->m_ImageData) return;

    // For simplicity, we are just printing to console and streaming the original image.
    // Drawing bounding boxes requires bounding box data from Python output.
    if (!detection.label.empty()) {
        std::cout << "Detected: " << detection.label;
        if (detection.score > 0) { // Check if score was parsed
            printf(" (%.2f)", detection.score);
        }
        std::cout << std::endl;
    } else {
        // std::cout << "No object detected." << std::endl; // Avoid excessive printing
    }

    // If Python script provides bounding boxes, update ParsedDetection and implement drawing here
    // Based on the previous DrawBoundingBox logic, but using ParsedDetection coordinates.
}


int main(void)
{
    printf("\n===== Head tracking with Object Detection via Python Script for DARwIn =====\n\n");

    change_current_dir();

    minIni *ini = new minIni(INI_FILE_PATH);

    // Initialize and configure the camera
    Image *rgb_display_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    // Ensure camera is providing RGB data.

    // Initialize the MJPG streamer
    mjpg_streamer *streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    // --- Robot Framework Initialization ---
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if (MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule *)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
    MotionManager::GetInstance()->SetEnable(true);

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

    // --- Robot Framework Initial Pose/State (Optional) ---
    // MotionManager::GetInstance()->PlayAction(MotionManager::GetInstance()->GetActionIndex("WalkingReady")); // Example
    // usleep(1000000); // Wait for action to complete


    std::cout << "INFO: Starting main loop..." << std::endl;
    while (1)
    {
        // 1. Capture Frame from Camera
        LinuxCamera::GetInstance()->CaptureFrame();
        Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
            usleep(10000); // Wait if frame not ready
            continue;
        }

        // 2. Save the Frame to a Temporary File
        std::string temp_image_path = save_frame_to_temp_file(current_cam_rgb_frame);
        if (temp_image_path.empty())
        {
            std::cerr << "Error saving frame to temp file. Skipping detection for this frame." << std::endl;
            // Decide if you want to continue the loop or break
            continue;
        }

        // 3. Construct the Command to Call the Python Script
        // Example command: "python3 /path/to/script.py /tmp/frame_XXXXXX.ppm"
        // Use snprintf or stringstream for more robust command construction if paths have spaces/special chars.
        std::string command = std::string(PYTHON_INTERPRETER) + " " +
                              std::string(PYTHON_SCRIPT_PATH) + " " +
                              temp_image_path;

        // 4. Execute the Python Script and Open a Pipe to Read its Output
        FILE *python_output_pipe = popen(command.c_str(), "r"); // "r" means read standard output
        if (!python_output_pipe)
        {
            std::cerr << "Error running Python script via popen: " << command << std::endl;
            // Clean up the temporary file even if popen fails
            unlink(temp_image_path.c_str());
            continue; // Skip this frame
        }

        // 5. Read and Parse Output from the Python Script
        // This function reads the script's stdout and extracts the detection info.
        // Assuming the script prints its result(s) and then exits.
        // If the script can detect multiple objects per frame, your parse function
        // needs to read multiple lines/parse a structure. For this simple example,
        // we'll assume it prints one primary result or we only care about the first.
        ParsedDetection latest_detection = parse_python_output(python_output_pipe);

        // 6. Close the Pipe and Wait for the Python Script to Finish
        // It's important to close the pipe and wait for the child process to exit.
        int pclose_status = pclose(python_output_pipe);
        if (pclose_status != 0)
        {
            std::cerr << "Warning: Python script exited with non-zero status " << pclose_status << std::endl;
            // Handle potential errors in the Python script execution
        }

        // 7. Clean up the Temporary Image File
        unlink(temp_image_path.c_str()); // Delete the temp file


        // Copy original camera frame to the display frame for drawing (optional, based on your drawing needs)
        memcpy(rgb_display_frame->m_ImageData, current_cam_rgb_frame->m_ImageData,
               current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);


        // --- Use the Detection Result for Robot Control ---
        bool target_found = false;
        Point2D tracked_object_center_for_head; // Center of the target for head tracking

        // Example: Check if a "person" was detected
        if (!latest_detection.label.empty() && latest_detection.label == "person")
        {
            target_found = true;
            // TODO: If Python outputs bounding box, calculate center here.
            // If Python doesn't output position, you might need a simpler tracking
            // strategy based solely on the presence/absence of the object,
            // or have Python output a simple "center_x, center_y" or just the label.
            // For now, let's assume Python outputs normalized bounding box and we calculate center:
            // tracked_object_center_for_head.X = (latest_detection.xmin + latest_detection.xmax) / 2.0 * Camera::WIDTH;
            // tracked_object_center_for_head.Y = (latest_detection.ymin + latest_detection.ymax) / 2.0 * Camera::HEIGHT;

            // If no bounding box is available from Python, you cannot do precise tracking.
            // You could implement a simple "look around until person is detected" logic.
            // For demonstration, let's assume Python outputs bounding box and we use it.
            // *** If your Python script *only* outputs the label, you need to remove the bounding box logic here and in ParsedDetection struct. ***
        }


        // --- Draw Detection Info (Optional, depends on data from Python) ---
        // This function now just prints info or draws if bounding box is available.
        DrawDetectionInfo(rgb_display_frame, latest_detection);


        // --- Head Tracking ---
        if (target_found)
        {
            // This part requires the target center (tracked_object_center_for_head)
            // which depends on the Python script providing bounding box data.
            // If Python only outputs label, you cannot use this precise tracking.
            // Assuming bounding box is available from Python output:
            Point2D P_err;
            P_err.X = (tracked_object_center_for_head.X - (Camera::WIDTH / 2.0)) / (Camera::WIDTH / 2.0);
            P_err.Y = (tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0)) / (Camera::HEIGHT / 2.0);
            Head::GetInstance()->MoveTracking(P_err);
        }
        else
        {
            // No target found, perhaps implement a search behavior or look forward
            // Head::GetInstance()->MoveTracking(Point2D(0.0, 0.0)); // Example: Stop active tracking
        }

        streamer->send_image(rgb_display_frame);

        // The duration of each loop iteration is now dominated by
        // Camera capture time + Image saving time + Python script execution time + Output parsing time.
        // Remove usleep unless needed to lower the frame rate explicitly.
        // usleep(10000);
    }

    // --- Cleanup ---
    delete rgb_display_frame;
    delete ini;
    delete streamer;
    // MotionManager cleanup if needed

    return 0;
}