/*
 * main.cpp
 *
 * Created on: 2011. 1. 4.
 * Author: robotis
 * Modified for Edge TPU Object Detection
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include <vector>    // For storing detection results
#include <memory>    // For std::unique_ptr
#include <iostream>  // For std::cerr, std::cout
#include <fstream>   // For std::ifstream (reading labels)
#include <algorithm> // For std::max, std::min

#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

// TensorFlow Lite and Edge TPU Headers
// Ensure these paths are correct for your installation
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/optional_debug_tools.h"

// EdgeTPU delegate header
#if defined(WITH_EDGETPU_DELEGATE) // You might need a conditional compile flag
#include "tflite/public/edgetpu_c.h" // C API for delegate
// Or if using a C++ helper library for the delegate:
// #include "coral/pipeline/delegates/edgetpu_delegate.h"
#endif

// Coral C++ Task Library (optional, but helpful for parsing results)
// If you use it, you'll need to link against it.
// #include "coral/detection/adapter.h" // For coral::GetDetectionResults
// #include "coral/tflite_utils.h"     // For coral::MakeEdgeTpuInterpreter, coral::ReadLabelsFile
// For this example, we'll implement basic parsing manually if not using full Coral Task Lib

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

// --- Edge TPU Configuration ---
// These should ideally be configurable (e.g., from INI_FILE_PATH or command line)
const char* MODEL_PATH = "../../../../Data/models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"; // IMPORTANT: Path to your Edge TPU model
const char* LABELS_PATH = "../../../../Data/models/imagenet_labels.txt";                                   // IMPORTANT: Path to your labels file
const float DETECTION_THRESHOLD = 0.5f;                                                                    // Minimum confidence score
const int MAX_DETECTIONS = 10;                                                                             // Max objects to detect per frame

// Structure to hold detection results
struct DetectionResult {
    float ymin, xmin, ymax, xmax; // Bounding box (normalized 0.0-1.0)
    float score;                  // Confidence score
    int class_id;                 // Detected class ID
    std::string label;            // Detected class label
};

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

// Function to load labels from file
std::vector<std::string> LoadLabels(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open labels file: " << path << std::endl;
        return labels;
    }
    std::string line;
    while (std::getline(file, line)) {
        labels.push_back(line);
    }
    return labels;
}

// Basic function to draw bounding boxes on the RGB image
// NOTE: This is a simplified drawing function. You might need a more robust one.
void DrawBoundingBox(Image* image, const DetectionResult& detection) {
    if (!image || !image->m_ImageData) return;

    int img_width = image->m_Width;
    int img_height = image->m_Height;

    int xmin = static_cast<int>(detection.xmin * img_width);
    int ymin = static_cast<int>(detection.ymin * img_height);
    int xmax = static_cast<int>(detection.xmax * img_width);
    int ymax = static_cast<int>(detection.ymax * img_height);

    // Clamp coordinates to image boundaries
    xmin = std::max(0, std::min(xmin, img_width - 1));
    ymin = std::max(0, std::min(ymin, img_height - 1));
    xmax = std::max(0, std::min(xmax, img_width - 1));
    ymax = std::max(0, std::min(ymax, img_height - 1));

    // Draw a simple red box (assuming RGB_PIXEL_SIZE = 3)
    unsigned char r = 255, g = 0, b = 0;

    for (int x = xmin; x <= xmax; ++x) { // Top and bottom lines
        if (ymin >= 0 && ymin < img_height) {
            image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 0] = r;
            image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 1] = g;
            image->m_ImageData[(ymin * img_width + x) * image->m_PixelSize + 2] = b;
        }
        if (ymax >= 0 && ymax < img_height) {
            image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 0] = r;
            image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 1] = g;
            image->m_ImageData[(ymax * img_width + x) * image->m_PixelSize + 2] = b;
        }
    }
    for (int y = ymin; y <= ymax; ++y) { // Left and right lines
        if (xmin >= 0 && xmin < img_width) {
            image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 0] = r;
            image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 1] = g;
            image->m_ImageData[(y * img_width + xmin) * image->m_PixelSize + 2] = b;
        }
        if (xmax >= 0 && xmax < img_width) {
            image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 0] = r;
            image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 1] = g;
            image->m_ImageData[(y * img_width + xmax) * image->m_PixelSize + 2] = b;
        }
    }
    // You would typically draw the label text here as well.
    // For simplicity, printing to console:
     std::cout << "Detected: " << detection.label << " (" << detection.score << ")" << std::endl;
}

// --- Helper: Image Preprocessing (Resize) ---
// You NEED a robust resizing function. This is a placeholder.
// Consider using a library like OpenCV or stb_image_resize.h
// Takes input image data, and outputs to `out_data` (which should be pre-allocated)
void resize_rgb_image(const unsigned char* in_data, int in_w, int in_h,
                      unsigned char* out_data, int out_w, int out_h) {
    // Basic nearest-neighbor scaling (example, not recommended for quality)
    if (!in_data || !out_data) return;
    for (int y_out = 0; y_out < out_h; ++y_out) {
        for (int x_out = 0; x_out < out_w; ++x_out) {
            int y_in = static_cast<int>((static_cast<float>(y_out) / out_h) * in_h);
            int x_in = static_cast<int>((static_cast<float>(x_out) / out_w) * in_w);

            y_in = std::min(y_in, in_h - 1); // Clamp
            x_in = std::min(x_in, in_w - 1); // Clamp

            const unsigned char* p_in = &in_data[(y_in * in_w + x_in) * 3]; // Assuming 3 channels (RGB)
            unsigned char* p_out = &out_data[(y_out * out_w + x_out) * 3];
            p_out[0] = p_in[0];
            p_out[1] = p_in[1];
            p_out[2] = p_in[2];
        }
    }
}


int main(void)
{
    printf( "\n===== Head tracking with Edge TPU Object Detection for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    // You could load MODEL_PATH, LABELS_PATH, DETECTION_THRESHOLD from ini here

    // --- Initialize Edge TPU ---
    // 1. Load Model
    std::unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile(MODEL_PATH);
    if (!model) {
        std::cerr << "ERROR: Failed to load TFLite model from " << MODEL_PATH << std::endl;
        return -1;
    }

    // 2. Build Interpreter
    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::InterpreterBuilder(*model, resolver)(&interpreter);
    if (!interpreter) {
        std::cerr << "ERROR: Failed to build TFLite interpreter." << std::endl;
        return -1;
    }

    // 3. Apply Edge TPU Delegate
#if defined(WITH_EDGETPU_DELEGATE)
    // Get list of available Edge TPUs
    size_t num_devices;
    std::unique_ptr<edgetpu_device, decltype(&edgetpu_free_devices)> devices(
        edgetpu_list_devices(&num_devices), &edgetpu_free_devices);

    if (num_devices == 0) {
        std::cerr << "WARNING: No Edge TPU devices found. Running on CPU." << std::endl;
    } else {
        const auto& delegate_options = TFLITE_EDGETPU_CREATE_DELEGATE_OPTIONS_DEFAULT;
        // To use a specific device:
        // delegate_options.device_path = devices.get()[0].path; // Use first device
        auto* delegate = edgetpu_create_delegate(devices.get()[0].type, devices.get()[0].path, nullptr, 0);
        // Or simpler, if you have only one and it's USB:
        // auto* delegate = edgetpu_create_delegate(EDGETPU_USB, nullptr, nullptr, 0);


        if (interpreter->ModifyGraphWithDelegate(TfLiteDelegateUniquePtr(delegate, [](TfLiteDelegate* d) { edgetpu_free_delegate(d); })) != kTfLiteOk) {
            std::cerr << "ERROR: Failed to apply Edge TPU delegate." << std::endl;
            // return -1; // You might choose to fallback to CPU or exit
        } else {
            std::cout << "INFO: Edge TPU delegate applied successfully." << std::endl;
        }
    }
#else
    std::cout << "INFO: Not compiled with Edge TPU delegate support. Running on CPU." << std::endl;
#endif

    // 4. Allocate tensors
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "ERROR: Failed to allocate TFLite tensors." << std::endl;
        return -1;
    }

    // 5. Get Input Tensor Details (for preprocessing)
    TfLiteTensor* input_tensor = interpreter->input_tensor(0);
    if (!input_tensor) {
        std::cerr << "ERROR: Cannot get input tensor." << std::endl;
        return -1;
    }
    const int input_height = input_tensor->dims->data[1];
    const int input_width = input_tensor->dims->data[2];
    const int input_channels = input_tensor->dims->data[3]; // Should be 3 for RGB
    std::cout << "INFO: Model expects input HxWxC: " << input_height << "x" << input_width << "x" << input_channels << std::endl;

    // Pre-allocate buffer for resized image if camera output differs from model input
    std::vector<unsigned char> resized_image_data;
    bool requires_resize = !(Camera::WIDTH == input_width && Camera::HEIGHT == input_height);
    if (requires_resize) {
        resized_image_data.resize(input_width * input_height * input_channels);
        std::cout << "INFO: Input images will be resized from " << Camera::WIDTH << "x" << Camera::HEIGHT
                  << " to " << input_width << "x" << input_height << std::endl;
    }


    // 6. Load Labels
    std::vector<std::string> labels = LoadLabels(LABELS_PATH);
    if (labels.empty() && std::string(LABELS_PATH) != "") {
        std::cerr << "WARNING: Labels could not be loaded. Detections will only have IDs." << std::endl;
    }
    // ------------------------------------

    // Image buffer for the output frame with detections drawn on it
    Image* rgb_display_frame = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
    // Ensure camera is providing RGB data. If it's BGR or YUV, you'll need conversion.

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    // --- Remove or Adapt Old Ball Finder/Tracker ---
    // ColorFinder* ball_finder = new ColorFinder(); // Original
    // ball_finder->LoadINISettings(ini);            // Original
    // httpd::ball_finder = ball_finder;             // Original (if mjpg_streamer used this for overlays)
    // BallTracker tracker = BallTracker();          // Original
    Point2D tracked_object_center_for_head; // This will hold the target for head tracking


    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
            return 0;
    }
    MotionManager::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(false);
    MotionManager::GetInstance()->SetEnable(true);
    /////////////////////////////////////////////////////////////////////

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_PAN, 8);
    Head::GetInstance()->m_Joint.SetPGain(JointData::ID_HEAD_TILT, 8);

    std::cout << "INFO: Starting main loop..." << std::endl;
    while(1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        Image* current_cam_rgb_frame = LinuxCamera::GetInstance()->fbuffer->m_RGBFrame;

        if (!current_cam_rgb_frame || !current_cam_rgb_frame->m_ImageData) {
            usleep(10000); // Wait if frame not ready
            continue;
        }

        // --- Preprocessing for Edge TPU ---
        unsigned char* input_image_ptr = current_cam_rgb_frame->m_ImageData;
        if (requires_resize) {
            // IMPORTANT: Implement robust resizing.
            // The `resize_rgb_image` is a placeholder.
            resize_rgb_image(current_cam_rgb_frame->m_ImageData,
                             current_cam_rgb_frame->m_Width, current_cam_rgb_frame->m_Height,
                             resized_image_data.data(),
                             input_width, input_height);
            input_image_ptr = resized_image_data.data();
        }

        // Copy data to input tensor
        // Model might expect float32 input in range [0,1] or [-1,1] or uint8 [0,255]
        // Check your model's requirements (e.g., using Netron app to view model)
        // Common for Edge TPU object detection models is uint8 input.
        if (input_tensor->type == kTfLiteUInt8) {
            memcpy(interpreter->typed_input_tensor<unsigned char>(0), input_image_ptr,
                   input_width * input_height * input_channels);
        } else if (input_tensor->type == kTfLiteFloat32) {
            // Example: Normalize uint8 to float32 [0,1]
            // float* input_f32 = interpreter->typed_input_tensor<float>(0);
            // for (int i = 0; i < input_width * input_height * input_channels; ++i) {
            //     input_f32[i] = static_cast<float>(input_image_ptr[i]) / 255.0f;
            // }
             std::cerr << "ERROR: Model expects float32 input, but this example assumes uint8. Please implement normalization." << std::endl;
             break; // Or handle appropriately
        } else {
            std::cerr << "ERROR: Unsupported input tensor type: " << input_tensor->type << std::endl;
            break;
        }


        // --- Run Inference ---
        if (interpreter->Invoke() != kTfLiteOk) {
            std::cerr << "ERROR: Failed to invoke TFLite interpreter." << std::endl;
            continue;
        }

        // --- Post-processing: Get Detection Results ---
        // Output tensor indices might vary based on your model.
        // Typical for SSD MobileNet:
        // Output 0: Bounding boxes (e.g., [1, 10, 4] -> [ymin, xmin, ymax, xmax])
        // Output 1: Class IDs (e.g., [1, 10])
        // Output 2: Scores (e.g., [1, 10])
        // Output 3: Number of detections (e.g., [1])

        // This part requires knowing your model's output tensor structure.
        // The Coral Task Library (e.g., coral::GetDetectionResults) simplifies this.
        // If doing it manually:
        const float* bboxes   = interpreter->typed_output_tensor<float>(0); // Adjust index and type if needed
        const float* class_ids = interpreter->typed_output_tensor<float>(1); // Often float, cast to int
        const float* scores   = interpreter->typed_output_tensor<float>(2);
        const float* num_detections_ptr = interpreter->typed_output_tensor<float>(3); // Usually a single float value
        int num_detections = static_cast<int>(num_detections_ptr[0]);

        std::vector<DetectionResult> detected_objects;
        for (int i = 0; i < num_detections && i < MAX_DETECTIONS; ++i) {
            if (scores[i] >= DETECTION_THRESHOLD) {
                DetectionResult det;
                det.ymin = bboxes[i * 4 + 0];
                det.xmin = bboxes[i * 4 + 1];
                det.ymax = bboxes[i * 4 + 2];
                det.xmax = bboxes[i * 4 + 3];
                det.score = scores[i];
                det.class_id = static_cast<int>(class_ids[i]);
                if (det.class_id >= 0 && det.class_id < labels.size()) {
                    det.label = labels[det.class_id];
                } else {
                    det.label = "ID_" + std::to_string(det.class_id);
                }
                detected_objects.push_back(det);
            }
        }

        // Copy original camera frame to the display frame
        memcpy(rgb_display_frame->m_ImageData, current_cam_rgb_frame->m_ImageData,
               current_cam_rgb_frame->m_NumberOfPixels * current_cam_rgb_frame->m_PixelSize);

        // --- Draw Detections & Find Target for Head ---
        bool target_found = false;
        float largest_area = 0.0f; // To track the largest "person", for example

        for (const auto& det : detected_objects) {
            DrawBoundingBox(rgb_display_frame, det); // Draw on the *original resolution* display frame

            // Example: Track the largest "person" object
            // You'll need to adjust "person" if your labels are different or you want to track something else.
            if (det.label == "person" || (labels.empty() && det.class_id == 0 /* if person is class 0 and no labels */)) {
                float area = (det.xmax - det.xmin) * (det.ymax - det.ymin); // Area in normalized coordinates
                if (area > largest_area) {
                    largest_area = area;
                    // Calculate center of the bounding box in original image pixel coordinates
                    tracked_object_center_for_head.X = (det.xmin + det.xmax) / 2.0 * Camera::WIDTH;
                    tracked_object_center_for_head.Y = (det.ymin + det.ymax) / 2.0 * Camera::HEIGHT;
                    target_found = true;
                }
            }
        }


        // --- Head Tracking ---
        // The original code used:
        // tracker.Process(ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
        // Head::GetInstance()->MoveTracking(tracker.ball_position);

        // Now, use `tracked_object_center_for_head`
        if (target_found) {
            // Convert pixel coordinates to the error signal expected by MoveTracking.
            // MoveTracking likely expects an error/offset from the center of the view.
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
        } else {
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
    // `interpreter` and `model` are unique_ptrs, will be auto-deleted.

    return 0;
}