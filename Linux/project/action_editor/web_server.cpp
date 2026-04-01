#include "httplib.h"
#include "cmd_process.h"
#include "Action.h"
#include "CM730.h"

using namespace Robot;

// Grab the existing globals from your original code
extern Action::PAGE Page;
extern int indexPage;
extern bool bEdited;
extern CM730 cm730;

// Track which step the Web UI is currently looking at
int webCurrentStep = 0;

void RunWebServer() {
    httplib::Server svr;

    // Tell the server to host your HTML/JS files from the "www" folder
    svr.set_mount_point("/", "./www");

    // API 1: Get the current page and joint data
    svr.Get("/api/state", [](const httplib::Request &, httplib::Response &res) {
        std::string json = "{ \"page\": " + std::to_string(indexPage) + 
                           ", \"step\": " + std::to_string(webCurrentStep) + 
                           ", \"joints\": {";
        
        // Loop through the 20 Dynamixel IDs
        for(int id = 1; id <= 20; id++) { 
            json += "\"" + std::to_string(id) + "\": " + std::to_string(Page.step[webCurrentStep].position[id]);
            if(id < 20) json += ", ";
        }
        json += "} }";
        
        res.set_content(json, "application/json");
    });

    // API 2: Update a specific joint (e.g., /api/joint/1/2048)
    svr.Post(R"(/api/joint/(\d+)/(\d+))", [](const httplib::Request &req, httplib::Response &res) {
        int id = std::stoi(req.matches[1]);
        int value = std::stoi(req.matches[2]);

        // 1. Update the Action Editor memory
        Page.step[webCurrentStep].position[id] = value;
        bEdited = true;

        // 2. Move the physical hardware instantly
        cm730.WriteWord(id, MX28::P_GOAL_POSITION_L, value, 0);

        res.set_content("{\"status\":\"ok\"}", "application/json");
    });

    // API 3: Play the current page
    svr.Post("/api/play", [](const httplib::Request &, httplib::Response &res) {
        Action::GetInstance()->Start(indexPage);
        res.set_content("{\"status\":\"playing\"}", "application/json");
    });

    std::cout << "\n[INFO] Web UI Server running on http://<RaspberryPi_IP>:8080\n" << std::endl;
    svr.listen("0.0.0.0", 8080);
}