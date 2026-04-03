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
        std::string pName = "";
        for(int n=0; n<Action::MAXNUM_NAME; n++) {
            char c = Page.header.name[n];
            if(c >= ' ' && c <= '~') pName += c; // Only grab printable characters
        }
        json += "\"name\": \"" + pName + "\",";
        json += "\"step\": " + std::to_string(webCurrentStep) + ",";
        json += "\"speed\": " + std::to_string(Page.header.speed) + ",";
        json += "\"accel\": " + std::to_string(Page.header.accel) + ",";
        json += "\"stepnum\": " + std::to_string(Page.header.stepnum) + ",";
        int stepTime = (webCurrentStep == 7) ? Step.time : Page.step[webCurrentStep].time;
        int stepPause = (webCurrentStep == 7) ? Step.pause : Page.step[webCurrentStep].pause;
        json += "\"step_time\": " + std::to_string(stepTime) + ",";
        json += "\"step_pause\": " + std::to_string(stepPause) + ",";
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
            int val = 2048; // Safe fallback in case the read fails!
            if (cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &val, 0) == CM730::SUCCESS) {
                Step.position[id] = val; // Store current physical angle
            } else {
                Step.position[id] = val; // Store the safe fallback
            }
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
                int val = 2048; 
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
    // --- UPDATED: Save Live Robot Pose to a Specific Step ---
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
            // 3. Automatically increase the total step count if appending a new step
            if (target_step >= Page.header.stepnum) {
                Page.header.stepnum = target_step + 1;
            }
            
            bEdited = true;
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- NEW: Save Page to File (motion_4096.bin) ---
    svr.Post("/api/save_file", [](const httplib::Request &, httplib::Response &res) {
        if (bEdited) {
            Action::GetInstance()->SavePage(indexPage, &Page);
            bEdited = false;
            res.set_content("{\"status\":\"saved\"}", "application/json");
        } else {
            res.set_content("{\"status\":\"no_changes\"}", "application/json");
        }
    });

    // --- NEW: Play a Single Step ---
    svr.Post(R"(/api/play_step/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int index = std::stoi(req.matches[1]);
        if (index >= 0 && index < Action::MAXNUM_STEP) {
            int param[JointData::NUMBER_OF_JOINTS * 5];
            int n = 0;
            int wGoalPosition, wStartPosition, wDistance;

            // Build the SyncWrite packet for smooth, synchronized movement
            for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
                if (Page.step[index].position[id] & Action::INVALID_BIT_MASK) continue;
                if (Page.step[index].position[id] & Action::TORQUE_OFF_BIT_MASK) continue;
                if (cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &wStartPosition, 0) != CM730::SUCCESS) continue;

                wGoalPosition = Page.step[index].position[id];
                wDistance = (wStartPosition > wGoalPosition) ? wStartPosition - wGoalPosition : wGoalPosition - wStartPosition;
                wDistance >>= 2;
                if (wDistance < 8) wDistance = 8;

                param[n++] = id;
                param[n++] = CM730::GetLowByte(wGoalPosition);
                param[n++] = CM730::GetHighByte(wGoalPosition);
                param[n++] = CM730::GetLowByte(wDistance);
                param[n++] = CM730::GetHighByte(wDistance);
            }

            // Move the robot!
            if (n > 0) {
                cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, n / 5, param);
                Step = Page.step[index]; // Update live memory to match the new pose
            }
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- NEW: Delete a Step ---
    svr.Post(R"(/api/delete_step/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int index = std::stoi(req.matches[1]);
        if (index >= 0 && index < Action::MAXNUM_STEP) {
            
            // Shift all steps down to fill the gap
            for (int i = index; i < Action::MAXNUM_STEP - 1; i++) {
                Page.step[i] = Page.step[i + 1];
            }
            
            // Wipe the very last step clean
            for (int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
                Page.step[Action::MAXNUM_STEP - 1].position[id] = Action::INVALID_BIT_MASK;
            }
            Page.step[Action::MAXNUM_STEP - 1].pause = 0;
            Page.step[Action::MAXNUM_STEP - 1].time = 0;

            // Reduce the total step count
            if (index < Page.header.stepnum && Page.header.stepnum > 0) {
                Page.header.stepnum--;
            }
            bEdited = true;
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- NEW: Get a list of ALL maximum pages and their names ---
    svr.Get("/api/pages_list", [](const httplib::Request &, httplib::Response &res) {
        std::string json = "[";
        Action::PAGE tempPage;
        
        // DARwIn-OP Action limits max pages to MAXNUM_PAGE (usually 256)
        for(int i = 1; i < Action::MAXNUM_PAGE; i++) {
            std::string pName = "";
            if (Action::GetInstance()->LoadPage(i, &tempPage)) {
                for(int n=0; n<Action::MAXNUM_NAME; n++) {
                    char c = tempPage.header.name[n];
                    if(c >= ' ' && c <= '~') pName += c;
                }
            }
            json += "{\"id\":" + std::to_string(i) + ",\"name\":\"" + pName + "\"}";
            if (i < Action::MAXNUM_PAGE - 1) json += ",";
        }
        json += "]";
        res.set_content(json, "application/json");
    });

    // --- NEW: Rename Current Page ---
    svr.Post("/api/rename_page", [](const httplib::Request &req, httplib::Response &res) {
        std::string new_name = req.body; // Read from raw text body
        
        // Clear old name with zeros
        for(int i=0; i<Action::MAXNUM_NAME; i++) Page.header.name[i] = 0; 
        
        // Write new name
        for(size_t i=0; i < new_name.length() && i < Action::MAXNUM_NAME; i++) {
            Page.header.name[i] = new_name[i];
        }
        
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- NEW: Clear/Initialize a New Page ---
    svr.Post("/api/new_page", [](const httplib::Request &, httplib::Response &res) {
        Action::GetInstance()->ResetPage(&Page); // Wipes all steps and name
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // --- Mirror Joint Position ---
    svr.Post(R"(/api/mirror/(\d+)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int src = std::stoi(req.matches[1]);
        int tgt = std::stoi(req.matches[2]);

        int src_val = 0;
        // Read the true physical position of the source, regardless of whether Torque is ON or OFF!
        if (cm730.ReadWord(src, MX28::P_PRESENT_POSITION_L, &src_val, 0) == CM730::SUCCESS) {
            
            // DARwIn-OP perfect symmetry formula
            int mirrored_val = 4096 - src_val; 

            if (webCurrentStep == 7) {
                // In Live Mode: Turn the target's torque ON so it can physically move, then move it!
                cm730.WriteByte(tgt, MX28::P_TORQUE_ENABLE, 1, 0);
                cm730.WriteWord(tgt, MX28::P_GOAL_POSITION_L, mirrored_val, 0);
                Step.position[tgt] = mirrored_val; 
            } else {
                // In Offline Mode: Just update the memory
                Page.step[webCurrentStep].position[tgt] = mirrored_val;
            }
            bEdited = true;
        }
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });
    
    svr.Post(R"(/api/step_param/(time|pause)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        std::string param = req.matches[1];
        int val = std::stoi(req.matches[2]);

        if (webCurrentStep == 7) {
            // Update Live Memory
            if (param == "time") Step.time = val;
            if (param == "pause") Step.pause = val;
        } else {
            // Update Offline Step Memory
            if (param == "time") Page.step[webCurrentStep].time = val;
            if (param == "pause") Page.step[webCurrentStep].pause = val;
        }
        
        bEdited = true;
        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    std::cout << "\n[INFO] Full Web Editor running on port 8080\n" << std::endl;
    svr.listen("0.0.0.0", 8080);
}