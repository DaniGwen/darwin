// Map the 20 Dynamixel IDs to their physical robot groups
const jointConfiguration = {
    "Head": [
        { id: 19, name: "Pan (19)" }, 
        { id: 20, name: "Tilt (20)" }
    ],
    "Right Arm": [
        { id: 1, name: "Shoulder Pitch (1)" }, 
        { id: 3, name: "Shoulder Roll (3)" }, 
        { id: 5, name: "Elbow (5)" }
    ],
    "Left Arm": [
        { id: 2, name: "Shoulder Pitch (2)" }, 
        { id: 4, name: "Shoulder Roll (4)" }, 
        { id: 6, name: "Elbow (6)" }
    ],
    "Right Leg": [
        { id: 7, name: "Hip Yaw (7)" }, 
        { id: 9, name: "Hip Roll (9)" }, 
        { id: 11, name: "Hip Pitch (11)" }, 
        { id: 13, name: "Knee (13)" }, 
        { id: 15, name: "Ankle Pitch (15)" }, 
        { id: 17, name: "Ankle Roll (17)" }
    ],
    "Left Leg": [
        { id: 8, name: "Hip Yaw (8)" }, 
        { id: 10, name: "Hip Roll (10)" }, 
        { id: 12, name: "Hip Pitch (12)" }, 
        { id: 14, name: "Knee (14)" }, 
        { id: 16, name: "Ankle Pitch (16)" }, 
        { id: 18, name: "Ankle Roll (18)" }
    ]
};

// --- 1. UI GENERATION ---
function buildUI() {
    const container = document.getElementById("sliders-container");

    for (const [groupName, joints] of Object.entries(jointConfiguration)) {
        // Create the panel for the group (e.g., "Right Arm")
        const groupDiv = document.createElement("div");
        groupDiv.className = "joint-group";
        groupDiv.innerHTML = `<h3>${groupName}</h3>`;

        // Create a slider for each joint in this group
        joints.forEach(joint => {
            const row = document.createElement("div");
            row.className = "slider-row";
            row.innerHTML = `
                <label>${joint.name}</label>
                <input type="range" id="slider-${joint.id}" min="0" max="4095" value="2048" 
                       oninput="updateJointDisplay(${joint.id}, this.value)"
                       onchange="sendJointCommand(${joint.id}, this.value)">
                <div class="val-display" id="val-${joint.id}">2048</div>
            `;
            groupDiv.appendChild(row);
        });

        container.appendChild(groupDiv);
    }
}

// --- 2. INTERACTION & API CALLS ---

// Updates the number next to the slider immediately while dragging
function updateJointDisplay(id, value) {
    document.getElementById(`val-${id}`).innerText = value;
}

// Sends the POST request to the C++ server when you release the slider
async function sendJointCommand(id, value) {
    try {
        await fetch(`/api/joint/${id}/${value}`, { method: 'POST' });
        console.log(`Sent: ID ${id} -> ${value}`);
    } catch (error) {
        console.error("Failed to send joint command:", error);
    }
}

// Sends the command to play the current Action Page
async function playAction() {
    try {
        await fetch(`/api/play`, { method: 'POST' });
        console.log("Playing action...");
    } catch (error) {
        console.error("Failed to play action:", error);
    }
}

// Fetches the current Page, Step, and Joint values from the robot
async function fetchRobotState() {
    try {
        const response = await fetch('/api/state');
        const data = await response.json();
        
        // Update the top navigation bar
        document.getElementById("status-display").innerText = `Page: ${data.page} | Step: ${data.step}`;

        // Update the sliders to match the robot's actual physical state
        // (Only do this on load or periodically, to avoid fighting the user's dragging)
        for (let id = 1; id <= 20; id++) {
            let val = data.joints[id];
            
            // The Action Editor uses a specific bitmask (INVALID_BIT_MASK) for unused joints.
            // If a joint is invalid/disabled, it usually reads as a massive number. Let's filter that out.
            if (val > 4095) continue; 

            const slider = document.getElementById(`slider-${id}`);
            const display = document.getElementById(`val-${id}`);
            
            if (slider && display) {
                slider.value = val;
                display.innerText = val;
            }
        }
    } catch (error) {
        console.error("Failed to fetch robot state. Is the server running?", error);
        document.getElementById("status-display").innerText = "Status: OFFLINE";
    }
}

// --- 3. INITIALIZATION ---
window.onload = () => {
    buildUI();           // Draw the sliders
    fetchRobotState();   // Sync sliders with the robot's current position

    // Optional: Auto-refresh the state every 2 seconds so if you use the 
    // terminal UI on the Pi, the Web UI stays in sync!
    setInterval(fetchRobotState, 2000); 
};