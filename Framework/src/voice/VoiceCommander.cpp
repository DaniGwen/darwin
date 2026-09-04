#include "VoiceCommander.h"
#include "ConsoleColors.h"
#include <fstream>
#include <iostream>
#include <cstdio> // For std::remove

VoiceCommander::VoiceCommander(const std::string& cmd_file_path) 
    : cmd_file_path_(cmd_file_path) {}

void VoiceCommander::RegisterCommand(const std::string& keyword, std::function<void()> action) {
    callbacks_.push_back({keyword, action});
}

std::string VoiceCommander::GetRawCommand() {
    std::ifstream voice_cmd_file(cmd_file_path_);
    std::string cmd = "";
    
    if (voice_cmd_file.is_open()) {
        std::getline(voice_cmd_file, cmd);
        voice_cmd_file.close();
        
        // Delete the file immediately so we don't process it twice
        std::remove(cmd_file_path_.c_str());
    }
    return cmd;
}

void VoiceCommander::ProcessCommands() {
    std::string cmd = GetRawCommand();
    if (cmd.empty()) return;

    while (!cmd.empty() && (cmd.back() == '\n' || cmd.back() == '\r' || cmd.back() == ' ')) {
        cmd.pop_back();
    }

    std::cout << GREEN << "INFO: Voice command received: '" << cmd << "'" << RESET << std::endl;

    bool handled = false;
    
    // Check registered commands in the order they were added
    for (const auto& pair : callbacks_) {
        if (cmd.find(pair.first) != std::string::npos) {
            pair.second(); // Execute the stored lambda function
            handled = true;
            break; // Stop after first match to prevent double triggers
        }
    }
    
   if (!handled) {
        std::cout << BOLDRED << "NO MATCH for: '" << cmd << "'. Registered triggers count: " 
                  << callbacks_.size() << RESET << std::endl;
        for (const auto& pair : callbacks_) {
            std::cout << "  Candidate: '" << pair.first << "'" << std::endl;
        }
    }
}