#include "httplib.h"
#include "cmd_process.h"
#include "Action.h"
#include "CM730.h"
#include <thread> // Required for background playback

using namespace Robot;

extern Action::PAGE Page;
extern Action::STEP Step; 
extern int indexPage;
extern bool bEdited;
extern CM730 cm730;
extern void ReadStep(CM730 *cm730);

// Grab the timer from main.cpp
extern LinuxMotionTimer *motion_timer_ptr;

int webCurrentStep = 7; 

void RunWebServer() {
    httplib::Server svr;
    svr.set_mount_point("/", "./www");

    // 1. Get complete state
    svr.Get("/api/state", [](const httplib::Request &, httplib::Response &res) {
        std::string json = "{";
        json += "\"page\": " + std::to_string(indexPage) + ",";
        json += "\"step\": " + std::to_string(webCurrentStep) + ",";
        json += "\"speed\": " + std::to_string(Page.header.speed) + ",";
        json += "\"accel\": " + std::to_string(Page.header.accel) + ",";
        json += "\"stepnum\": " + std::to_string(Page.header.stepnum) + ",";
        
        json += "\"joints\": {";
        // ID 21 and 22 are Wrist/Gripper
        for(int id = 1; id <= 22; id++) {
            // If viewing Step 7, show Live memory. Otherwise show Page Step memory.
            int val = (webCurrentStep == 7) ? Step.position[id] : Page.step[webCurrentStep].position[id];
            json += "\"" + std::to_string(id) + "\": " + std::to_string(val);
            if(id < 22) json += ", ";
        }
        json += "} }";
        
        res.set_content(json, "application/json");
    });

    // 2. Move Joint
    svr.Post(R"(/api/joint/(\d+)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int id = std::stoi(req.matches[1]);
        int value = std::stoi(req.matches[2]);
        
        if (webCurrentStep == 7) {
            Step.position[id] = value;
            cm730.WriteWord(id, MX28::P_GOAL_POSITION_L, value, 0); // Move hardware
        } else {
            Page.step[webCurrentStep].position[id] = value; // Edit memory only
        }
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // 3. Toggle Torque
   svr.Post(R"(/api/torque/(\d+)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int id = std::stoi(req.matches[1]);
        int state = std::stoi(req.matches[2]); // 1 or 0
        
        cm730.WriteByte(id, MX28::P_TORQUE_ENABLE, state, 0);
        
        if (state == 1) {
            int val;
            cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &val, 0);
            Step.position[id] = val; // Store current physical angle
        } else {
            Step.position[id] = Action::TORQUE_OFF_BIT_MASK; // Mark as ????
        }
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    svr.Post(R"(/api/torque_all/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int state = std::stoi(req.matches[1]); // 1 or 0
        
        for(int id = 1; id <= 22; id++) {
            cm730.WriteByte(id, MX28::P_TORQUE_ENABLE, state, 0);
            if (state == 1) {
                int val;
                if (cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &val, 0) == CM730::SUCCESS) {
                    Step.position[id] = val;
                }
            } else {
                Step.position[id] = Action::TORQUE_OFF_BIT_MASK;
            }
        }
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // 4. Change Page
    svr.Post(R"(/api/page/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int p = std::stoi(req.matches[1]);
        if(p > 0 && p < Action::MAXNUM_PAGE) {
            indexPage = p;
            Action::GetInstance()->LoadPage(indexPage, &Page);
            ReadStep(&cm730); // Refresh live hardware state
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // 5. Change Viewing Step
    svr.Post(R"(/api/step/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        webCurrentStep = std::stoi(req.matches[1]);
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // 6. Read physical robot state
    svr.Post("/api/read_robot", [](const httplib::Request &, httplib::Response &res) {
        ReadStep(&cm730); // Pulls physical data into Step (STP7)
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // 7. Play Action
    svr.Post("/api/play", [](const httplib::Request &, httplib::Response &res) {
        if (Action::GetInstance()->IsRunning()) {
            res.set_content("{\"status\":\"already_playing\"}", "application/json");
            return;
        }

        // Run the motion loop in a detached thread so the browser doesn't freeze
        std::thread play_thread([]() {
            Action::GetInstance()->m_Joint.SetEnableBody(true, true);
            MotionManager::GetInstance()->SetEnable(true);
            if (motion_timer_ptr) motion_timer_ptr->Start();

            if (Action::GetInstance()->Start(indexPage, &Page)) {
                // Keep the thread alive while the robot moves
                while (Action::GetInstance()->IsRunning()) {
                    usleep(10000);
                }
            }

            // Clean up and stop the heartbeat when finished
            MotionManager::GetInstance()->SetEnable(false);
            if (motion_timer_ptr) motion_timer_ptr->Stop();
        });
        play_thread.detach();

        res.set_content("{\"status\":\"playing\"}", "application/json");
    });

    svr.Post(R"(/api/page_param/(speed|accel)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        std::string param = req.matches[1];
        int val = std::stoi(req.matches[2]);
        
        if (param == "speed") Page.header.speed = val;
        if (param == "accel") Page.header.accel = val;
        
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- NEW: Save Live Robot Pose to a Specific Step ---
    svr.Post(R"(/api/save_live_step/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int target_step = std::stoi(req.matches[1]);
        
        if (target_step >= 0 && target_step <= 6) {
            // 1. Refresh STP7 with the exact current physical hardware angles
            ReadStep(&cm730); 
            
            // 2. Copy the live data into the target step memory
            for(int id = 1; id <= 22; id++) {
                // Only copy valid, torque-on joints
                if (!(Step.position[id] & Action::INVALID_BIT_MASK) && !(Step.position[id] & Action::TORQUE_OFF_BIT_MASK)) {
                    Page.step[target_step].position[id] = Step.position[id];
                }
            }
            Page.step[target_step].pause = Step.pause;
            Page.step[target_step].time = Step.time;
            
            // 3. Automatically increase the total step count if appending a new step
            if (target_step >= Page.header.stepnum) {
                Page.header.stepnum = target_step + 1;
            }
            
            bEdited = true;
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });
    
    std::cout << "\n[INFO] Full Web Editor running on port 8080\n" << std::endl;
    svr.listen("0.0.0.0", 8080);
}