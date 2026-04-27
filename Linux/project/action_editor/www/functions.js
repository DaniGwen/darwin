const jointConfiguration = {
    "Head & Body": [{ id: 19, name: "Pan (19)" }, { id: 20, name: "Tilt (20)" }],
    "Right Arm": [{ id: 1, name: "Sho Pitch (1)" }, { id: 3, name: "Sho Roll (3)" }, { id: 5, name: "Elbow (5)" }],
    "Left Arm": [{ id: 2, name: "Sho Pitch (2)" }, { id: 4, name: "Sho Roll (4)" }, { id: 6, name: "Elbow (6)" }],
    "Right Leg": [{ id: 7, name: "Hip Yaw (7)" }, { id: 9, name: "Hip Roll (9)" }, { id: 11, name: "Hip Pitch (11)" }, { id: 13, name: "Knee (13)" }, { id: 15, name: "Ank Pitch (15)" }, { id: 17, name: "Ank Roll (17)" }],
    "Left Leg": [{ id: 8, name: "Hip Yaw (8)" }, { id: 10, name: "Hip Roll (10)" }, { id: 12, name: "Hip Pitch (12)" }, { id: 14, name: "Knee (14)" }, { id: 16, name: "Ank Pitch (16)" }, { id: 18, name: "Ank Roll (18)" }],
    "Right Hand": [
        { id: 21, name: "R Wrist (21)", mirrorId: 23, invert: true },  
        { id: 22, name: "R Gripper (22)", mirrorId: 24, invert: false }  
    ],
    "Left Hand": [
        { id: 23, name: "L Wrist (23)", mirrorId: 21, invert: true },  
        { id: 24, name: "L Gripper (24)", mirrorId: 22, invert: false }  
    ]
};

window.onload = () => {
    buildUI();
    fetchPageList();
    fetchRobotState();
    // Refresh every 1.5s to keep in sync with terminal usage
    setInterval(fetchRobotState, 1500);
};

let pendingEditsNotPlayed = false;
let pendingSave = false; // NEW: Tracks when Play is clicked so Save can glow
let needsPlayToSync = false;
let actionStepNum = 1;
let lastPlayedStep = -1;

function updateSyncUI() {
    const btnSave = document.getElementById('btn-save-step');
    const btnPlay = document.getElementById('btn-play-step');

    if (!btnSave || !btnPlay) return;

    if ((pendingEditsNotPlayed || needsPlayToSync) && currentStep !== 7) {
        // STATE 1: Unsynced. User edited sliders OR switched to a new tab.
        btnSave.disabled = true;
        btnSave.style.opacity = "0.5";
        btnSave.style.cursor = "not-allowed";
        btnSave.style.background = "#333";
        btnSave.style.color = "#a0a0a0";

        // Dynamic text depending on the exact context
        btnSave.innerText = pendingEditsNotPlayed ? "⚠️ Play to Unlock" : "⚠️ Play to Sync";

        btnPlay.classList.add('needs-action');
        btnSave.classList.remove('needs-action');

    } else if (pendingSave && currentStep !== 7) {
        // STATE 2: Step played, waiting to be saved.
        btnSave.disabled = false;
        btnSave.style.opacity = "1";
        btnSave.style.cursor = "pointer";
        btnSave.style.background = "var(--success)"; // Green
        btnSave.style.color = "#000";
        btnSave.innerText = "📸 Save Step";

        btnPlay.classList.remove('needs-action');
        btnSave.classList.add('needs-action');

    } else {
        // STATE 3: Fully synced and safely saved.
        btnSave.disabled = false;
        btnSave.style.opacity = "1";
        btnSave.style.cursor = "pointer";
        btnSave.style.background = "#444"; // Gray
        btnSave.style.color = "#fff";
        btnSave.innerText = "📸 Save Step";

        btnPlay.classList.remove('needs-action');
        btnSave.classList.remove('needs-action');
    }
}

let currentStep = 7; // Start in Live mode

// --- UPDATED UI GENERATOR ---
// --- NEW: The Clipboard Memory ---
const groupClipboard = {};

// --- UPDATED UI GENERATOR ---
function buildUI() {
    const container = document.getElementById("sliders-container");
    container.innerHTML = "";

    for (const [groupName, joints] of Object.entries(jointConfiguration)) {
        const groupDiv = document.createElement("div");
        groupDiv.className = "joint-group";

        const isLeftLimb = groupName.includes("Left");
        const isRightLimb = groupName.includes("Right");
        let mirrorBtn = "";

        if (isLeftLimb) {
            mirrorBtn = `<button onclick="mirrorGroup('${groupName}')" style="font-size:0.75rem; background:var(--accent); color:#000; padding:2px 8px; border:none; border-radius:3px; cursor:pointer;">🪞 Mirror Right</button>`;
        } else if (isRightLimb) {
            mirrorBtn = `<button onclick="mirrorGroup('${groupName}')" style="font-size:0.75rem; background:var(--accent); color:#000; padding:2px 8px; border:none; border-radius:3px; cursor:pointer;">🪞 Mirror Left</button>`;
        }

        groupDiv.innerHTML = `
            <div style="display:flex; justify-content:space-between; align-items:center; border-bottom: 1px solid #333; padding-bottom: 4px; margin-bottom: 8px;">
                <h3 style="margin:0; border:none; padding:0; color:var(--accent); font-size: 1rem;">${groupName}</h3>
                <div style="display:flex; gap:3px; flex-wrap:nowrap; justify-content:flex-end;">
                    <button onclick="toggleGroupTorque('${groupName}', 1)" style="font-size:0.7rem; background:var(--success); color:#000; padding:2px 5px; border:none; border-radius:3px; cursor:pointer;">ON</button>
                    <button onclick="toggleGroupTorque('${groupName}', 0)" style="font-size:0.7rem; background:var(--danger); color:#fff; padding:2px 5px; border:none; border-radius:3px; cursor:pointer;">OFF</button>
                    <button onclick="copyGroup('${groupName}', this)" style="font-size:0.7rem; background:#444; color:#fff; padding:2px 5px; border:none; border-radius:3px; cursor:pointer;">📄 Copy</button>
                    <button onclick="pasteGroup('${groupName}', this)" style="font-size:0.7rem; background:#444; color:#fff; padding:2px 5px; border:none; border-radius:3px; cursor:pointer;">📋 Paste</button>
                    ${mirrorBtn}
                </div>
            </div>
        `;

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

async function toggleGroupTorque(groupName, state) {
    if (currentStep !== 7) await setStep(7);

    const joints = jointConfiguration[groupName];
    if (!joints) return;

    const promises = joints.map(j => fetch(`/api/torque/${j.id}/${state}`, { method: 'POST' }));
    await Promise.all(promises);

    pendingSave = true; // NEW: Flag the physical change!
    updateSyncUI();
    fetchRobotState();
}

// --- NEW: Mirroring Logic ---
// Maps the [Source ID, Target ID] pairs for each limb
const mirrorPairs = {
    "Right Arm": [[1, 2], [3, 4], [5, 6]],
    "Left Arm": [[2, 1], [4, 3], [6, 5]],
    "Right Leg": [[7, 8], [9, 10], [11, 12], [13, 14], [15, 16], [17, 18]],
    "Left Leg": [[8, 7], [10, 9], [12, 11], [14, 13], [16, 15], [18, 17]]
};

async function mirrorGroup(sourceName) {
    const pairs = mirrorPairs[sourceName];
    if (!pairs) return;

    try {
        // Send the mirror command for every joint in the limb simultaneously
        const promises = pairs.map(pair => fetch(`/api/mirror/${pair[0]}/${pair[1]}`, { method: 'POST' }));
        await Promise.all(promises);

        pendingEditsNotPlayed = true;
        updateSyncUI();

        // Wait a tiny bit for the hardware to finish moving, then refresh UI
        setTimeout(fetchRobotState, 200);
    } catch (error) {
        console.error("Mirroring failed:", error);
    }
}

async function fetchPageList() {
    try {
        const res = await fetch('/api/pages_list');
        const pages = await res.json();

        const select = document.getElementById("select-page");
        select.innerHTML = ""; // Clear existing options

        pages.forEach(p => {
            const opt = document.createElement("option");
            opt.value = p.id;
            const dispName = p.name.trim() === "" ? "(Empty/Uninitialized)" : p.name;
            opt.innerText = `Pg ${p.id}: ${dispName}`;
            select.appendChild(opt);
        });
    } catch (e) { console.error("Failed to load page list", e); }
}

function updateJointDisplay(id, value) {
    document.getElementById(`val-${id}`).innerText = value;
}

async function sendJointCommand(id, value) {
    await fetch(`/api/joint/${id}/${value}`, { method: 'POST' });

    pendingEditsNotPlayed = true;
    updateSyncUI();
}

async function toggleTorque(id) {
    if (currentStep !== 7) await setStep(7);

    const btn = document.getElementById(`tq-${id}`);
    const isOff = btn.classList.contains("off");
    const newState = isOff ? 1 : 0;

    await fetch(`/api/torque/${id}/${newState}`, { method: 'POST' });

    pendingSave = true; // NEW: Flag the physical change!
    updateSyncUI();
    fetchRobotState();
}

async function loadPage(pageNum) {
    await fetch(`/api/page/${pageNum}`, { method: 'POST' });
    fetchRobotState();
}

// --- NEW: Track the last offline step ---
let lastOfflineStep = -1;

async function setStep(stepNum) {
    // 1. Clean up button visuals
    document.querySelectorAll('.step-btn').forEach(btn => {
        btn.classList.remove('active');
        btn.style.border = "1px solid #333";
    });

    if (stepNum !== 7) lastOfflineStep = stepNum;

    const activeBtn = document.getElementById(`btn-step-${stepNum}`);
    if (activeBtn) activeBtn.classList.add('active');

    if (stepNum === 7 && lastOfflineStep !== -1) {
        const lastBtn = document.getElementById(`btn-step-${lastOfflineStep}`);
        if (lastBtn) {
            lastBtn.style.border = "2px dashed var(--accent)";
            lastBtn.style.boxSizing = "border-box";
        }
    }

    if (currentStep === 7 && stepNum !== 7) {
        // Returned from Live Mode
        if (pendingSave || pendingEditsNotPlayed) {
            pendingSave = true;
            pendingEditsNotPlayed = false;
        }
        lastPlayedStep = -1;
    } else if (currentStep !== 7 && stepNum !== 7 && currentStep !== stepNum) {
        // Switched between offline tabs (e.g., STP 1 -> STP 2).
        pendingSave = false;
        pendingEditsNotPlayed = false;
        needsPlayToSync = true; // THE GUARDIAN FIX: Require playback before saving!
    }

    currentStep = stepNum;

    // --- Update Labels and UI ---
    const label = document.getElementById('current-step-label');
    if (stepNum === 7) {
        let reminder = lastOfflineStep !== -1 ? ` (Came from STP ${lastOfflineStep})` : "";
        label.innerText = `MODE: LIVE${reminder}`;
        label.style.color = "var(--danger)";
    } else {
        label.innerText = `EDITING: STP ${stepNum}`;
        label.style.color = "var(--accent)";
    }

    const isEditMode = (stepNum !== 7);
    document.getElementById('btn-save-step').style.display = isEditMode ? 'block' : 'none';
    document.getElementById('btn-delete-step').style.display = isEditMode ? 'block' : 'none';
    document.getElementById('btn-play-step').style.display = isEditMode ? 'block' : 'none';
    document.getElementById('btn-copy-step').style.display = isEditMode ? 'block' : 'none';
    document.getElementById('btn-paste-step').style.display = isEditMode ? 'block' : 'none';

    updateSyncUI();
    await fetch(`/api/step/${stepNum}`, { method: 'POST' });
    fetchRobotState();
}

async function readRobot() {
    await fetch('/api/read_robot', { method: 'POST' });
    if (currentStep !== 7) setStep(7); // Jump to Live tab if not already there
    fetchRobotState();
}

// --- UPDATED: Smart Action Playback ---
async function playAction() {
    // The DARwIn-OP engine is 0-indexed. If there are 4 steps, the last step is STP 3.
    const lastStepOfAction = Math.max(0, actionStepNum - 1);
    
    try {
        // Did we already play the last step?
        if (lastPlayedStep !== lastStepOfAction) {
            
            // 1. Give the user visual feedback that we are prepping the robot
            const btn = document.querySelector('.btn-play');
            const oldText = btn.innerText;
            btn.innerText = "⏳ Syncing to Last Step...";
            btn.style.opacity = "0.7";
            
            // 2. Tell the native DARwIn-OP engine to smoothly play the last step
            await fetch(`/api/play_step/${lastStepOfAction}`, { method: 'POST' });
            
            // 3. Wait 1.2 seconds for the physical motors to smoothly glide into position.
            // (Because this is a JS Promise, your web browser stays perfectly responsive!)
            await new Promise(resolve => setTimeout(resolve, 1200));
            
            // 4. Update the tracker and restore the button
            lastPlayedStep = lastStepOfAction;
            btn.innerText = oldText;
            btn.style.opacity = "1";
        }
        
        // The robot is now perfectly positioned. Safely execute the sequence!
        await fetch('/api/play', { method: 'POST' });
        
    } catch (error) {
        console.error("Failed to play action:", error);
    }
}

async function fetchRobotState() {
    try {
        const res = await fetch('/api/state');
        const data = await res.json();
        actionStepNum = data.stepnum || 1;

        const selectPage = document.getElementById("select-page");
        if (selectPage.value != data.page && selectPage.options.length > 0) {
            selectPage.value = data.page;
        }

        // UPDATE INPUTS SAFELY: Only overwrite them if the user IS NOT currently clicking/typing inside them
        if (document.activeElement !== document.getElementById("inp-name")) {
            document.getElementById("inp-name").value = data.name;
        }
        if (document.activeElement !== document.getElementById("inp-speed")) {
            document.getElementById("inp-speed").value = data.speed;
        }
        if (document.activeElement !== document.getElementById("inp-accel")) {
            document.getElementById("inp-accel").value = data.accel;
        }
        if (document.activeElement !== document.getElementById("inp-step-time")) {
            document.getElementById("inp-step-time").value = data.step_time;
        }
        if (document.activeElement !== document.getElementById("inp-step-pause")) {
            document.getElementById("inp-step-pause").value = data.step_pause;
        }

        // --- JOINT SLIDERS LOOP ---
        for (let id = 1; id <= 24; id++) {
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

                // --- BUG FIX 2: Prevent sliders from snapping backward! ---
                if (!pendingSave || currentStep === 7) {
                    slider.value = val;
                }

                tqBtn.innerText = "ON"; tqBtn.className = "btn-torque";
            }
        } // <--- END OF THE FOR LOOP

        // --- BUG FIX 1: Main Save Button (Moved OUTSIDE the loop!) ---
        const mainSaveBtn = document.getElementById("btn-main-save");
        if (mainSaveBtn) {
            if (data.is_edited) {
                mainSaveBtn.classList.add('needs-file-save');
                mainSaveBtn.style.background = ""; // Clear inline color
                mainSaveBtn.style.color = "";      // Clear inline text color
                mainSaveBtn.innerText = "💾 Save to File*";
            } else {
                mainSaveBtn.classList.remove('needs-file-save');
                mainSaveBtn.style.background = "#444"; // Restore inline color
                mainSaveBtn.style.color = "#fff";      // Restore inline text color
                mainSaveBtn.innerText = "💾 Saved";
            }
        }

    } catch (error) {
        console.error("Server offline.");
    }
}

async function toggleAllTorque(state) {
    if (currentStep !== 7) await setStep(7);

    try {
        await fetch(`/api/torque_all/${state}`, { method: 'POST' });

        pendingSave = true; // NEW: Flag the physical change!
        updateSyncUI();
        fetchRobotState();
    } catch (error) {
        console.error("Failed to toggle all torque:", error);
    }
}

// --- Send Speed/Accel updates to the server ---
async function updatePageParam(param, value) {
    try {
        await fetch(`/api/page_param/${param}/${value}`, { method: 'POST' });
        console.log(`Updated ${param} to ${value}`);
    } catch (error) {
        console.error("Failed to update parameter:", error);
    }
}

// --- Save the live pose ---
async function saveLiveToCurrentStep() {
    if (currentStep === 7) return;

    try {
        // Instantly save to the backend without interrupting the user with a pop-up!
        await fetch(`/api/save_live_step/${currentStep}`, { method: 'POST' });
        fetchRobotState();

        // Reset tracking states
        pendingSave = false;

        // Provide a slick, 1.5-second visual confirmation directly on the button
        const btnSave = document.getElementById('btn-save-step');
        btnSave.innerText = "✅ Saved!";
        btnSave.style.background = "var(--success)";
        btnSave.style.color = "#000";
        btnSave.classList.remove('needs-action');

        setTimeout(() => {
            updateSyncUI(); // Reverts the button back to State 3 (Gray)
        }, 1500);

    } catch (error) {
        console.error("Failed to save step:", error);
    }
}

// --- Save to motion.bin file ---
async function saveToFile() {
    try {
        const res = await fetch('/api/save_file', { method: 'POST' });
        const data = await res.json();

        if (data.status === "saved") {
            const mainSaveBtn = document.getElementById("btn-main-save");

            // Instantly remove the orange glow and flash green!
            mainSaveBtn.classList.remove('needs-file-save');
            mainSaveBtn.style.background = "var(--success)";
            mainSaveBtn.style.color = "#000";
            mainSaveBtn.innerText = "✅ Saved!";

            // Wait 1.5 seconds, then return to dark gray
            setTimeout(() => {
                mainSaveBtn.style.background = "#444";
                mainSaveBtn.style.color = "#fff";
                mainSaveBtn.innerText = "💾 Saved";
            }, 1500);

            fetchRobotState(); // Sync up the rest of the UI
        }
    } catch (error) {
        console.error("Failed to save file:", error);
    }
}

// --- NEW: Delete Current Step ---
async function deleteCurrentStep() {
    if (currentStep === 7) return;
    if (confirm(`Are you sure you want to completely delete STP ${currentStep}? This will shift all following steps down.`)) {
        try {
            await fetch(`/api/delete_step/${currentStep}`, { method: 'POST' });
            fetchRobotState();
        } catch (error) {
            console.error("Failed to delete step:", error);
        }
    }
}

async function playSingleStep() {
    if (currentStep === 7) return;
    try {
        await fetch(`/api/play_step/${currentStep}`, { method: 'POST' });

        // Only trigger the Save glow if we actually moved edited sliders.
        // If we just clicked Play to "Sync" the robot to the tab, it doesn't need saving!
        if (pendingEditsNotPlayed) {
            pendingSave = true;
        }

        pendingEditsNotPlayed = false;
        lastPlayedStep = currentStep;
        needsPlayToSync = false; // The physical robot is now completely synced with the screen!

        updateSyncUI();

    } catch (error) {
        console.error("Failed to play step:", error);
    }
}

// --- NEW: Rename Page ---
async function renamePage() {
    const newName = document.getElementById("inp-name").value;
    await fetch('/api/rename_page', { method: 'POST', body: newName });
    fetchPageList(); // Refresh dropdown names
    fetchRobotState(); // Update UI
    alert(`Page renamed to: ${newName}. Don't forget to Save to File!`);
}

// --- NEW: Clear/Initialize Page ---
async function newPage() {
    if (confirm("⚠️ Are you sure? This will instantly delete the name and ALL steps of the current page!")) {
        await fetch('/api/new_page', { method: 'POST' });
        fetchPageList();
        fetchRobotState();
    }
}

// --- NEW: Copy Group Pose to Memory ---
function copyGroup(groupName, btnElement) {
    const joints = jointConfiguration[groupName];
    if (!joints) return;

    groupClipboard[groupName] = {};

    // Read the current slider values directly from the UI
    joints.forEach(j => {
        const slider = document.getElementById(`slider-${j.id}`);
        // Only copy the value if the joint is actually valid/enabled (not ---- or ????)
        if (slider && !slider.disabled) {
            groupClipboard[groupName][j.id] = slider.value;
        }
    });

    // Visual feedback
    const oldText = btnElement.innerText;
    btnElement.innerText = "✅ Copied!";
    btnElement.style.background = "var(--success)";
    btnElement.style.color = "#000";
    setTimeout(() => {
        btnElement.innerText = oldText;
        btnElement.style.background = "#444";
        btnElement.style.color = "#fff";
    }, 1000);
}

// --- NEW: Paste Group Pose from Memory ---
async function pasteGroup(groupName, btnElement) {
    const clipData = groupClipboard[groupName];

    // Safety check
    if (!clipData || Object.keys(clipData).length === 0) {
        alert(`⚠️ You haven't copied anything for the ${groupName} yet!`);
        return;
    }

    try {
        // Shoot off the individual joint updates to the server simultaneously
        const promises = Object.entries(clipData).map(([id, val]) => {
            return fetch(`/api/joint/${id}/${val}`, { method: 'POST' });
        });
        await Promise.all(promises);

        pendingEditsNotPlayed = true;
        updateSyncUI();

        // Visual feedback
        const oldText = btnElement.innerText;
        btnElement.innerText = "✅ Pasted!";
        btnElement.style.background = "var(--success)";
        btnElement.style.color = "#000";
        setTimeout(() => {
            btnElement.innerText = oldText;
            btnElement.style.background = "#444";
            btnElement.style.color = "#fff";
        }, 1000);

        // Give the backend a split second to process, then refresh the sliders
        setTimeout(fetchRobotState, 200);
    } catch (error) {
        console.error("Paste failed:", error);
    }
}

// --- NEW: Whole Step Clipboard ---
let stepClipboard = null;

async function copyWholeStep() {
    if (currentStep === 7) return;

    try {
        // Fetch the exact state of the step we are currently looking at
        const res = await fetch('/api/state');
        const data = await res.json();

        stepClipboard = data.joints; // Save all 22 joints to memory

        // Visual feedback
        const btn = document.getElementById('btn-copy-step');
        const oldText = btn.innerText;
        btn.innerText = "✅ Copied!";
        btn.style.background = "var(--success)";
        setTimeout(() => {
            btn.innerText = oldText;
            btn.style.background = "#444";
        }, 1000);

    } catch (error) {
        console.error("Failed to copy step:", error);
    }
}

async function pasteWholeStep() {
    if (currentStep === 7) return;
    if (!stepClipboard) {
        alert("⚠️ You haven't copied a step yet!");
        return;
    }

    try {
        // Send all 22 joint values to the server simultaneously
        const promises = Object.entries(stepClipboard).map(([id, val]) => {
            return fetch(`/api/joint/${id}/${val}`, { method: 'POST' });
        });
        await Promise.all(promises);

        pendingEditsNotPlayed = true;
        updateSyncUI();

        // Visual feedback
        const btn = document.getElementById('btn-paste-step');
        const oldText = btn.innerText;
        btn.innerText = "✅ Pasted!";
        btn.style.background = "var(--success)";
        setTimeout(() => {
            btn.innerText = oldText;
            btn.style.background = "#444";
        }, 1000);

        // Refresh UI
        setTimeout(fetchRobotState, 200);
    } catch (error) {
        console.error("Failed to paste step:", error);
    }
}

// --- NEW: Update Step Time/Pause ---
async function updateStepParam(param, value) {
    try {
        // Clamp the value between 0 and 255 (the max the DARwIn framework allows)
        let safeVal = Math.max(0, Math.min(255, parseInt(value)));
        await fetch(`/api/step_param/${param}/${safeVal}`, { method: 'POST' });
        console.log(`Updated Step ${param} to ${safeVal}`);
    } catch (error) {
        console.error("Failed to update step parameter:", error);
    }
}