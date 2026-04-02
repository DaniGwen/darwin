const jointConfiguration = {
    "Head & Body": [{ id: 19, name: "Pan (19)" }, { id: 20, name: "Tilt (20)" }],
    "Right Arm": [{ id: 1, name: "Sho Pitch (1)" }, { id: 3, name: "Sho Roll (3)" }, { id: 5, name: "Elbow (5)" }],
    "Left Arm": [{ id: 2, name: "Sho Pitch (2)" }, { id: 4, name: "Sho Roll (4)" }, { id: 6, name: "Elbow (6)" }],
    "Right Leg": [{ id: 7, name: "Hip Yaw (7)" }, { id: 9, name: "Hip Roll (9)" }, { id: 11, name: "Hip Pitch (11)" }, { id: 13, name: "Knee (13)" }, { id: 15, name: "Ank Pitch (15)" }, { id: 17, name: "Ank Roll (17)" }],
    "Left Leg": [{ id: 8, name: "Hip Yaw (8)" }, { id: 10, name: "Hip Roll (10)" }, { id: 12, name: "Hip Pitch (12)" }, { id: 14, name: "Knee (14)" }, { id: 16, name: "Ank Pitch (16)" }, { id: 18, name: "Ank Roll (18)" }],
    "Extras": [{ id: 21, name: "Wrist (21)" }, { id: 22, name: "Gripper (22)" }]
};

let currentStep = 7; // Start in Live mode

function buildUI() {
    const container = document.getElementById("sliders-container");
    container.innerHTML = ""; // clear

    for (const [groupName, joints] of Object.entries(jointConfiguration)) {
        const groupDiv = document.createElement("div");
        groupDiv.className = "joint-group";
        groupDiv.innerHTML = `<h3>${groupName}</h3>`;

        joints.forEach(joint => {
            const row = document.createElement("div");
            row.className = "slider-row";
            row.innerHTML = `
                <label>${joint.name}</label>
                <button class="btn-torque" id="tq-${joint.id}" onclick="toggleTorque(${joint.id})">ON</button>
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

function updateJointDisplay(id, value) {
    document.getElementById(`val-${id}`).innerText = value;
}

async function sendJointCommand(id, value) {
    await fetch(`/api/joint/${id}/${value}`, { method: 'POST' });
}

async function toggleTorque(id) {
    const btn = document.getElementById(`tq-${id}`);
    const isOff = btn.classList.contains("off");
    const newState = isOff ? 1 : 0; // Turn on if currently off
    
    await fetch(`/api/torque/${id}/${newState}`, { method: 'POST' });
    fetchRobotState(); // Refresh UI instantly
}

async function loadPage() {
    const pageNum = document.getElementById("inp-page").value;
    await fetch(`/api/page/${pageNum}`, { method: 'POST' });
    fetchRobotState();
}

async function setStep(stepNum) {
    // Update active tab visuals
    document.querySelectorAll('.step-btn').forEach(btn => btn.classList.remove('active'));
    document.getElementById(`btn-step-${stepNum}`).classList.add('active');
    currentStep = stepNum;

    await fetch(`/api/step/${stepNum}`, { method: 'POST' });
    fetchRobotState();
}

async function readRobot() {
    await fetch('/api/read_robot', { method: 'POST' });
    if(currentStep !== 7) setStep(7); // Jump to Live tab if not already there
    fetchRobotState();
}

async function playAction() {
    await fetch(`/api/play`, { method: 'POST' });
}

async function fetchRobotState() {
    try {
        const res = await fetch('/api/state');
        const data = await res.json();
        
        document.getElementById("inp-page").value = data.page;
        document.getElementById("inp-speed").value = data.speed;
        document.getElementById("inp-accel").value = data.accel;

        for (let id = 1; id <= 22; id++) {
            let val = data.joints[id];
            
            const slider = document.getElementById(`slider-${id}`);
            const display = document.getElementById(`val-${id}`);
            const tqBtn = document.getElementById(`tq-${id}`);
            
            if (!slider) continue;

            // Handle Bitmasks
            if (val & 0x4000) { // Invalid data (----)
                display.innerText = "----";
                display.className = "val-display val-invalid";
                slider.disabled = true;
                tqBtn.innerText = "OFF"; tqBtn.className = "btn-torque off";
            } else if (val & 0x2000) { // Torque Off (????)
                display.innerText = "????";
                display.className = "val-display val-invalid";
                slider.disabled = true;
                tqBtn.innerText = "OFF"; tqBtn.className = "btn-torque off";
            } else {
                display.innerText = val;
                display.className = "val-display";
                slider.disabled = false;
                slider.value = val;
                tqBtn.innerText = "ON"; tqBtn.className = "btn-torque";
            }
        }
    } catch (error) { console.error("Server offline."); }
}

window.onload = () => {
    buildUI();
    fetchRobotState();
    // Refresh every 1.5s to keep in sync with terminal usage
    setInterval(fetchRobotState, 1500);
};

async function toggleAllTorque(state) {
    try {
        await fetch(`/api/torque_all/${state}`, { method: 'POST' });
        fetchRobotState(); // Refresh UI instantly
    } catch (error) {
        console.error("Failed to toggle all torque:", error);
    }
}