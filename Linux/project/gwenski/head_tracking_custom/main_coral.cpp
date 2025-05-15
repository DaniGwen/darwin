/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Edge TPU Object Detection using Coral Task Library
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <vector>     // For storing detection results
#include <memory>     // For std::unique_ptr
#include <iostream>   // For std::cerr, std::cout
#include <fstream>    // For std::ifstream (reading labels, although Task Library can often do this)
#include <algorithm>  // For std::max, std::min

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h" // Assuming this provides basic robot control structures

// Coral C++ Task Library headers
#include "coral/detection/object_detector.h" // For the ObjectDetector class
#include "coral/tflite_utils.h"              // Potentially useful utilities
#include "coral/error_reporter.h"            // For handling errors

// --- Edge TPU Configuration ---
// These should ideally be configurable (e.g., from INI_FILE_PATH or command line)
const char *MODEL_PATH = "../../../../Data/models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"; // IMPORTANT: Path to your Edge TPU model
// LABELS_PATH is often not needed explicitly if labels.txt is next to the model, but can be used
// const char* LABELS_PATH = "../../../../Data/models/imagenet_labels.txt";
const float DETECTION_THRESHOLD = 0.5f;
                                                                    // Minimum confidence score
    const int MAX_DETECTIONS = 10;
                                                                            // Max objects to detect per frame

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

    void
    change_current_dir()
{
        char exepath[1024] = {0};
        if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Basic function to draw bounding boxes on the RGB image
// NOTE: This is a simplified drawing function. You might need a more robust one.
// Using coral::Detection struct from the Task Library
void DrawBoundingBox(Image *image, const coral::Detection &detection, const std::vector<std::string> &labels)
{
        if (!image || !image->m_ImageData) return;

        int img_width = image->m_Width;
        int img_height = image->m_Height;

        // Bounding box coordinates from coral::Detection are normalized [0.0, 1.0]
    int xmin = static_cast<int>(detection.bounding_box.xmin * img_width);
        int ymin = static_cast<int>(detection.bounding_box.ymin * img_height);
        int xmax = static_cast<int>(detection.bounding_box.xmax * img_width);
        int ymax = static_cast<int>(detection.bounding_box.ymax * img_height);

        // Clamp coordinates to image boundaries
    xmin = std::max(0, std::min(xmin, img_width - 1));
        ymin = std::max(0, std::min(ymin, img_height - 1));
        xmax = std::max(0, std::min(xmax, img_width - 1));
        ymax = std::max(0, std::min(ymax, img_height - 1));

        // Draw a simple red box (assuming RGB_PIXEL_SIZE = 3)
    unsigned char r = 255, g = 0, b = 0;

        for (int x = xmin; x <= xmax; ++x)
    { // Top and bottom lines
                if (ymin >= 0 && ymin < img_height)
        {
                        image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 0] = r;
                        image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 1] = g;
                        image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 2] = b;
                   
        }
                if (ymax >= 0 && ymax < img_height)
        {
                        image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 0] = r;
                        image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 1] = g;
                        image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 2] = b;
                   
        }
           
    }
        for (int y = ymin; y <= ymax; ++y)
    { // Left and right lines
                if (xmin >= 0 && xmin < img_width)
        {
                        image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 0] = r;
                        image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 1] = g;
                        image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 2] = b;
                   
        }
                if (xmax >= 0 && xmax < img_width)
        {
                        image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 0] = r;
                        image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 1] = g;
                        image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 2] = b;
                   
        }
           
    }

        // Get label from class ID using the provided labels vector
    std::string label_text = "Unknown";
        if (detection.class_id >= 0 && detection.class_id < labels.size())
    {
                label_text = labels[detection.class_id];
           
    }
    else if (!labels.empty())
    {
                // If labels were loaded but ID is out of bounds
        label_text = "ID_" + std::to_string(detection.class_id);
           
    }
    else
    {
                // If no labels file was loaded
        label_text = "ID_" + std::to_string(detection.class_id);
           
    }

        // For simplicity, printing to console:
    std::cout << "Detected: " << label_text << " (" << detection.score << ") at ["
              << xmin << "," << ymin << "]-[" << xmax << "," << ymax << "]" << std::endl;

        // Drawing the text label on the image is more complex and requires a font rendering library.
    // This example only draws the box.
}

// --- Helper: Image Preprocessing (Resize) ---
// The Task Library often handles resizing internally based on model requirements.
// You primarily need to provide the image data in the correct format (uint8 RGB).
// If your camera provides a different format or you need specific preprocessing,
// you'd implement it here or use a library like OpenCV.
// This placeholder remains, but the *need* for manual resize might be reduced
// if the Task Library handles it when you pass the original image dimensions.
void resize_rgb_image(const unsigned char *in_data, int in_w, int in_h,
                      unsigned char *out_data, int out_w, int out_h)
{
        // Basic nearest-neighbor scaling (example, not recommended for quality)
    if (!in_data || !out_data) return;
        for (int y_out = 0; y_out < out_h; ++y_out)
    {
                for (int x_out = 0; x_out < out_w; ++x_out)
        {
                        int y_in = static_cast<int>((static_cast<float>(y_out) / out_h) * in_h);
                        int x_in = static_cast<int>((static_cast<float>(x_out) / out_w) * in_w);

                        y_in = std::min(y_in, in_h - 1); // Clamp
                        x_in = std::min(x_in, in_w - 1); // Clamp

                        const unsigned char *p_in = &in_data[(y_in * in_w + x_in) * 3]; // Assuming 3 channels (RGB)
                        unsigned char *p_out = &out_data[(y_out * out_w + x_out) * 3];
                        p_out[0] = p_in[0];
                        p_out[1] = p_in[1];
                        p_out[2] = p_in[2];
                   
        }
           
    }
}

int main(void)
{
        printf("\n===== Head tracking with Edge TPU Object Detection for DARwIn =====\n\n");

        change_current_dir();

        minIni *ini = new minIni(INI_FILE_PATH);
        // You could load MODEL_PATH, LABELS_PATH, DETECTION_THRESHOLD from ini here

    // --- Initialize Coral Object Detector ---
    // The ObjectDetector constructor handles loading the model,
    // setting up the TFLite interpreter, applying the Edge TPU delegate,
    // and allocating tensors.
    std::unique_ptr<coral::ObjectDetector> object_detector;
        std::unique_ptr<coral::ErrorReporter> error_reporter =
        std::make_unique<coral::StderrReporter>(); // Use stderr for errors

        // Create options for the detector
    coral::DetectionOptions options;
        options.threshold = DETECTION_THRESHOLD; // Set the confidence threshold
        options.top_k = MAX_DETECTIONS;
            // Set the maximum number of detections
    // You can also specify a device path here if needed, e.g., options.device = ":0";

    object_detector = coral::ObjectDetector::Create(MODEL_PATH, options, error_reporter.get());

        if (!object_detector)
    {
                std::cerr << "ERROR: Failed to create Coral ObjectDetector. Check model path and Edge TPU connection." << std::endl;
                // Detailed error might be in error_reporter
        return -1;
           
    }
        std::cout << "INFO: Coral ObjectDetector initialized successfully." << std::endl;

        // The Task Library can often load labels automatically if a file named
    // the same as the model (but with .txt extension) is in the same directory.
    // If not, you can load them manually or pass a labels file path to options
    // if the API supports it (check Coral docs).
    // For drawing, we'll get labels from the detector if available.
    const std::vector<std::string> &labels = object_detector->GetLabels();
        if (labels.empty())
    {
                std::cerr << "WARNING: Labels could not be loaded by the ObjectDetector. Detections will only have IDs." << std::endl;
           
    }

        // Get model input tensor details from the detector (useful for preprocessing)
    // The Task Library might handle resizing internally, but knowing the expected
    // input size is good if you need custom preprocessing before passing data.
    const auto &input_tensor_shape = object_detector->GetInputTensorShape(); // Assuming typical HWC format
        const int model_input_height = input_tensor_shape[1];
        const int model_input_width = input_tensor_shape[2];
        const int model_input_channels = input_tensor_shape[3];
        std::cout << "INFO: Model expects input HxWxC: " << model_input_height << "x" << model_input_width << "x" << model_input_channels << std::endl;

        // Image buffer for the output frame with detections drawn on it
    Image *rgb_display_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

        LinuxCamera::GetInstance()->Initialize(0);
        LinuxCamera::GetInstance()->LoadINISettings(ini);
        // Ensure camera is providing RGB data. If it's BGR or YUV, you'll need conversion.

    mjpg_streamer *streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

        // --- Remove or Adapt Old Ball Finder/Tracker ---
    // ColorFinder* ball_finder = new ColorFinder(); // Original
    // ball_finder->LoadINISettings(ini);            // Original
    // httpd::ball_finder = ball_finder;             // Original (if mjpg_streamer used this for overlays)
    // BallTracker tracker = BallTracker();          // Original
    Point2D tracked_object_center_for_head; // This will hold the target for head tracking

        //////////////////// Framework Initialize ////////////////////////////
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
        /////////////////////////////////////////////////////////////////////

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
        Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
        Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

        std::cout << "INFO: Starting main loop..." << std::endl;
        while (1)
   
    {
                LinuxCamera::GetInstance()->CaptureFrame();
                Image *current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

                if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData)
        {
                        usleep(10000); // Wait if frame not ready
                        continue;
                   
        }

                // --- Preprocessing for Edge TPU (if needed) ---
        // The Task Library expects uint8_t RGB data.
        // If your camera provides a different format or you need custom steps,
        // perform them here.
        unsigned char *input_image_ptr = current_cam_rgb_frame->m_ImageData;
                int input_width = current_cam_rgb_frame->m_Width;
                int input_height = current_cam_rgb_frame->m_Height;
                // The Task Library handles internal resizing if the input dimensions
        // don't match the model's expected input size.

        // --- Run Inference using ObjectDetector ---
        // Pass the image data and its dimensions.
        std::vector<coral::Detection> detections =
            object_detector->Detect(input_image_ptr, input_width, input_height);

                // --- Post-processing: Get Detection Results (already parsed by ObjectDetector) ---
        // `detections` vector now contains parsed results (bounding box, score, class ID).

        // Copy original camera frame to the display frame for drawing
        memcpy(rgb_display_frame->m_ImageData, current_cam_rgb_frame->m_ImageData,
               current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);

                // --- Draw Detections & Find Target for Head ---
        bool target_found = false;
                float largest_area = 0.0f; // To track the largest "person", for example
                int target_class_id = -1;  // Example: ID for "person" - find this from your labels

                // If labels were loaded by the detector, find the ID for "person"
        if (!labels.empty())
        {
                        for (size_t i = 0; i < labels.size(); ++i)
            {
                                if (labels[i] == "person")
                { // Match your label string here
                                        target_class_id = i;
                                        break;
                                   
                }
                           
            }
                   
        }
        else
        {
                        // Fallback if labels not loaded, assuming 'person' is class 0 (common in COCO)
            target_class_id = 0; // **WARNING: This is a guess. Verify your model's class IDs.**
                   
        }

                for (const auto &det : detections)
        {
                        // The ObjectDetector already filtered by DETECTION_THRESHOLD and MAX_DETECTIONS

            DrawBoundingBox(rgb_display_frame, det, labels); // Draw on the *original resolution* display frame

                        // Example: Track the largest "person" object based on class ID
            if (det.class_id == target_class_id)
            {
                                float area = (det.bounding_box.xmax - det.bounding_box.xmin) * (det.bounding_box.ymax - det.bounding_box.ymin); // Area in normalized coordinates
                                if (area > largest_area)
                {
                                        largest_area = area;
                                        // Calculate center of the bounding box in original image pixel coordinates
                    tracked_object_center_for_head.X = (det.bounding_box.xmin + det.bounding_box.xmax) / 2.0 * Camera::WIDTH;
                                        tracked_object_center_for_head.Y = (det.bounding_box.ymin + det.bounding_box.ymax) / 2.0 * Camera::HEIGHT;
                                        target_found = true;
                      
                }
                           
            }
                   
        }

                // --- Head Tracking ---
        // Use `tracked_object_center_for_head`
        if (target_found)
        {
                        // Convert pixel coordinates to the error signal expected by MoveTracking.
            // (0,0) in P_err usually means "object is centered".
            // X range could be -1 (far left) to 1 (far right).
            // Y range could be -1 (far top) to 1 (far bottom) - Y direction might be inverted.

            Point2D P_err;
                        // Normalize to -1.0 to 1.0 range (approx)
            P_err.X = (tracked_object_center_for_head.X - (Camera::WIDTH / 2.0)) / (Camera::WIDTH / 2.0);
                        P_err.Y = (tracked_object_center_for_head.Y - (Camera::HEIGHT / 2.0)) / (Camera::HEIGHT / 2.0);
                        // You might need to scale P_err.X and P_err.Y by some factor,
            // or your Head::MoveTracking might handle this internally with its P-gains.
            // The original BallTracker might have had more sophisticated logic (smoothing, PID).

            Head::GetInstance()->MoveTracking(P_err);
                   
        }
        else
        {
                        // No target found, perhaps make the head look forward or scan
            // Head::GetInstance()->MoveTracking(Point2D(0.0, 0.0)); // Stop active tracking
       
        }

                streamer->send_image(rgb_display_frame);

                // usleep(10000); // Optional delay
   
    }

        // Cleanup
    delete rgb_display_frame;
        delete ini;
        delete streamer;
        // `object_detector` and `error_reporter` are unique_ptrs, will be auto-deleted.

    return 0;
}