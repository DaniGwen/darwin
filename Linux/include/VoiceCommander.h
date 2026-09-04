#ifndef VOICE_COMMANDER_H_
#define VOICE_COMMANDER_H_

#include <string>
#include <functional>
#include <vector>

class VoiceCommander {
public:
    // Initializes the commander with the default temp file path
    VoiceCommander(const std::string& cmd_file_path = "/tmp/darwin_voice_cmd.txt");
    
    // Binds a specific spoken phrase to a C++ function
    void RegisterCommand(const std::string& keyword, std::function<void()> action);
    
    // Reads the file, executes the matching action, and deletes the file
    void ProcessCommands();

    // Fetches and clears the file manually (useful for the startup sequence)
    std::string GetRawCommand();

private:
    std::string cmd_file_path_;
    // Using a vector preserves the order so specific commands evaluate before generic ones
    std::vector<std::pair<std::string, std::function<void()>>> callbacks_;
};

#endif