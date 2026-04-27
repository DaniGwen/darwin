#include <iostream>
#include <fstream>
#include <cstring>

#define MAXNUM_STEP 7
#define OLD_JOINTS 22
#define NEW_JOINTS 24

// --- OLD 22-JOINT STRUCTURE ---
struct OLD_STEP {
    unsigned short position[OLD_JOINTS + 1];
    unsigned char pause;
    unsigned char time;
};

struct OLD_PAGE {
    struct {
        unsigned char name[14];
        unsigned char reserved1;
        unsigned char stepnum;
        unsigned char reserved2;
        unsigned char playcnt;
        unsigned short next;
        unsigned char exit;
        unsigned char reserved3;
        unsigned char speed;
        unsigned char reserved4;
        unsigned char accel;
        unsigned char reserved5;
    } header;
    OLD_STEP step[MAXNUM_STEP];
};

// --- NEW 24-JOINT STRUCTURE ---
struct NEW_STEP {
    unsigned short position[NEW_JOINTS + 1];
    unsigned char pause;
    unsigned char time;
};

struct NEW_PAGE {
    struct {
        unsigned char name[14];
        unsigned char reserved1;
        unsigned char stepnum;
        unsigned char reserved2;
        unsigned char playcnt;
        unsigned short next;
        unsigned char exit;
        unsigned char reserved3;
        unsigned char speed;
        unsigned char reserved4;
        unsigned char accel;
        unsigned char reserved5;
    } header;
    NEW_STEP step[MAXNUM_STEP];
};

int main() {
    std::ifstream inFile("motion_4096.bin", std::ios::binary);
    if (!inFile) {
        std::cerr << "Could not open motion_4096.bin!" << std::endl;
        return 1;
    }

    std::ofstream outFile("motion_4096_new.bin", std::ios::binary);
    
    OLD_PAGE oldPage;
    NEW_PAGE newPage;
    
    int count = 0;
    while (inFile.read(reinterpret_cast<char*>(&oldPage), sizeof(OLD_PAGE))) {
        memset(&newPage, 0, sizeof(NEW_PAGE));
        
        // Copy Header (Names, Speeds, Next Page, etc.)
        newPage.header = oldPage.header;
        
        // Copy Steps
        for (int s = 0; s < MAXNUM_STEP; s++) {
            newPage.step[s].pause = oldPage.step[s].pause;
            newPage.step[s].time = oldPage.step[s].time;
            
            // Copy old joints 1 to 22 exactly as they were
            for (int j = 1; j <= OLD_JOINTS; j++) {
                newPage.step[s].position[j] = oldPage.step[s].position[j];
            }
            
            // Initialize new Left Hand joints 23 and 24 as INVALID (0x4000).
            // This prevents the new hand from snapping unexpectedly during old actions.
            newPage.step[s].position[23] = 0x4000;
            newPage.step[s].position[24] = 0x4000;
        }
        
        outFile.write(reinterpret_cast<char*>(&newPage), sizeof(NEW_PAGE));
        count++;
    }
    
    std::cout << "Successfully migrated " << count << " pages to the new 24-joint format!" << std::endl;
    return 0;
}