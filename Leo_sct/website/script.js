const ROSBRIDGE_URL = `ws://${window.location.hostname}:9090`;

const robots = Array.from({ length: 4 }, (_, index) => ({
  id: index + 1,
  name: `Leo-${index + 1}`,
  ns: `robot_${index}`,
  isOn: false,
}));

const state = {
  mode: "startup",
  manualRobotId: 1,
  autonomousRobotIds: new Set(),
  autonomousActiveIds: new Set(),
};

const startupGrid = document.getElementById("startupGrid");
const autonomousGrid = document.getElementById("autonomousGrid");
const manualRobotSelect = document.getElementById("manualRobotSelect");
const cmdPreview = document.getElementById("cmdPreview");
const activeSummary = document.getElementById("activeSummary");
const connectionStatus = document.getElementById("connectionStatus");

let rosSocket = null;
let rosConnected = false;
const advertisedTopics = new Set();

function setConnectionStatus(connected) {
  rosConnected = connected;
  connectionStatus.textContent = connected ? "ROS2: Connected" : "ROS2: Disconnected";
}

function sendRosMessage(payload) {
  if (!rosSocket || !rosConnected) return;
  rosSocket.send(JSON.stringify(payload));
}

function advertiseTopic(topic, type) {
  if (advertisedTopics.has(topic)) return;
  sendRosMessage({ op: "advertise", topic, type });
  advertisedTopics.add(topic);
}

function connectRosBridge() {
  if (rosSocket) {
    rosSocket.close();
  }
  rosSocket = new WebSocket(ROSBRIDGE_URL);
  rosSocket.addEventListener("open", () => {
    setConnectionStatus(true);
    robots.forEach((robot) => {
      advertiseTopic(`/${robot.ns}/cmd_vel`, "geometry_msgs/msg/Twist");
    });
  });
  rosSocket.addEventListener("close", () => {
    setConnectionStatus(false);
    advertisedTopics.clear();
  });
  rosSocket.addEventListener("error", () => {
    setConnectionStatus(false);
  });
}

function renderStartupCards() {
  startupGrid.innerHTML = "";
  robots.forEach((robot) => {
    const card = document.createElement("div");
    card.className = "card robot-card";
    card.innerHTML = `
      <h3>${robot.name}</h3>
      <div class="status-row">
        <span>Status: ${robot.isOn ? "Online" : "Offline"}</span>
        <span class="status-dot ${robot.isOn ? "on" : ""}"></span>
      </div>
      <div class="button-row">
        <button class="primary-btn" data-action="start" data-id="${robot.id}">Start</button>
        <button class="ghost-btn" data-action="stop" data-id="${robot.id}">Stop</button>
      </div>
    `;
    startupGrid.appendChild(card);
  });
}

function renderAutonomousOptions() {
  autonomousGrid.innerHTML = "";
  robots.forEach((robot) => {
    const wrapper = document.createElement("label");
    wrapper.className = "status-row";
    wrapper.innerHTML = `
      <span>${robot.name}</span>
      <input type="checkbox" data-id="${robot.id}" ${
        state.autonomousRobotIds.has(robot.id) ? "checked" : ""
      } />
    `;
    autonomousGrid.appendChild(wrapper);
  });
}

function renderManualSelect() {
  manualRobotSelect.innerHTML = "";
  robots.forEach((robot) => {
    const option = document.createElement("option");
    option.value = robot.id;
    option.textContent = robot.name;
    if (robot.id === state.manualRobotId) {
      option.selected = true;
    }
    manualRobotSelect.appendChild(option);
  });
}

function updateSummary() {
  const autoList = Array.from(state.autonomousRobotIds)
    .map((id) => robots.find((robot) => robot.id === id)?.name)
    .filter(Boolean)
    .join(", ");
  const manualName = robots.find((robot) => robot.id === state.manualRobotId)?.name;
  activeSummary.textContent = `Manual: ${manualName || "-"} • Auto: ${autoList || "-"}`;
}

function appendConsoleLine(text) {
  const line = document.createElement("div");
  line.className = "console-line";
  line.textContent = text;
  cmdPreview.prepend(line);
}

function switchMode(nextMode) {
  state.mode = nextMode;
  document.querySelectorAll(".mode-btn").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === nextMode);
    button.setAttribute("aria-selected", button.dataset.mode === nextMode);
  });
  document.querySelectorAll("[data-mode-panel]").forEach((panel) => {
    panel.classList.toggle("is-hidden", panel.dataset.modePanel !== nextMode);
  });
}

function publishCmdVel(robot, linearX, angularZ) {
  if (!rosConnected) {
    appendConsoleLine("ROS bridge not connected. Start rosbridge_websocket.");
    return;
  }
  const topic = `/${robot.ns}/cmd_vel`;
  const msg = {
    linear: { x: linearX, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angularZ },
  };
  sendRosMessage({ op: "publish", topic, msg });
}

function callSupervisorService(enabled, robotIds) {
  if (!rosConnected) {
    appendConsoleLine("ROS bridge not connected. Start rosbridge_websocket.");
    return;
  }
  if (!robotIds.length) {
    appendConsoleLine("Select autonomous robots before enabling SCT.");
    return;
  }
  const blocked = robotIds
    .map((id) => robots.find((robot) => robot.id === id))
    .filter((robot) => robot && !robot.isOn);
  if (blocked.length) {
    appendConsoleLine(
      `[auto] Blocked (not started): ${blocked.map((robot) => robot.name).join(", ")}.`
    );
  }
  const allowedIds = robotIds.filter((id) => robots.find((robot) => robot.id === id)?.isOn);
  if (!allowedIds.length) {
    appendConsoleLine("[auto] No started robots selected.");
    return;
  }
  const names = allowedIds
    .map((id) => robots.find((robot) => robot.id === id))
    .filter(Boolean);
  names.forEach((robot) => {
    sendRosMessage({
      op: "call_service",
      service: `/${robot.ns}/enable_supervisor`,
      type: "std_srvs/SetBool",
      args: { data: enabled },
    });
  });
  names.forEach((robot) => {
    if (enabled) {
      state.autonomousActiveIds.add(robot.id);
    } else {
      state.autonomousActiveIds.delete(robot.id);
    }
  });
  appendConsoleLine(
    `[auto] Supervisor ${enabled ? "enabled" : "disabled"} for ${names
      .map((robot) => robot.name)
      .join(", ")}.`
  );
}

function stopRobotMotion(robot) {
  publishCmdVel(robot, 0.0, 0.0);
  appendConsoleLine(`[cmd_vel] ${robot.name} stop (zero velocity).`);
}

function sendCmdVel(command) {
  const linear = Number(document.getElementById("linearSpeed").value);
  const angular = Number(document.getElementById("angularSpeed").value);
  const robot = robots.find((entry) => entry.id === state.manualRobotId);
  if (!robot?.isOn) {
    if (robot) {
      stopRobotMotion(robot);
    }
    appendConsoleLine("[cmd_vel] Manual drive blocked: robot is stopped.");
    return;
  }
  if (state.autonomousActiveIds.has(robot.id)) {
    appendConsoleLine("[cmd_vel] Manual drive blocked: robot is in autonomous mode.");
    return;
  }
  const robotName = robot?.name;
  const payload = {
    robot: robotName,
    linear: command === "forward" ? linear : command === "backward" ? -linear : 0,
    angular: command === "left" ? angular : command === "right" ? -angular : 0,
    stamp: new Date().toISOString(),
  };
  appendConsoleLine(`[cmd_vel] ${JSON.stringify(payload)}`);
  if (robot) {
    publishCmdVel(robot, payload.linear, payload.angular);
  }
}

function updateRobotPower(id, power) {
  const robot = robots.find((entry) => entry.id === id);
  if (!robot) return;
  robot.isOn = power;
  renderStartupCards();
}

function initHandlers() {
  connectionStatus.addEventListener("click", () => {
    if (!rosConnected) {
      appendConsoleLine("Reconnecting to ROS bridge...");
      connectRosBridge();
    }
  });
  document.querySelectorAll(".mode-btn").forEach((button) => {
    button.addEventListener("click", () => switchMode(button.dataset.mode));
  });

  startupGrid.addEventListener("click", (event) => {
    const target = event.target;
    if (!(target instanceof HTMLButtonElement)) return;
    const id = Number(target.dataset.id);
    if (target.dataset.action === "start") {
      updateRobotPower(id, true);
      callSupervisorService(false, [id]);
    }
    if (target.dataset.action === "stop") {
      updateRobotPower(id, false);
      callSupervisorService(false, [id]);
      state.autonomousActiveIds.delete(id);
      const robot = robots.find((entry) => entry.id === id);
      if (robot) {
        stopRobotMotion(robot);
      }
    }
  });

  document.getElementById("allStopBtn").addEventListener("click", () => {
    robots.forEach((robot) => {
      robot.isOn = false;
      callSupervisorService(false, [robot.id]);
      state.autonomousActiveIds.delete(robot.id);
      stopRobotMotion(robot);
    });
    renderStartupCards();
    appendConsoleLine("Emergency stop: all robots powered down.");
  });

  autonomousGrid.addEventListener("change", (event) => {
    const target = event.target;
    if (!(target instanceof HTMLInputElement)) return;
    const id = Number(target.dataset.id);
    if (target.checked) {
      state.autonomousRobotIds.add(id);
    } else {
      state.autonomousRobotIds.delete(id);
    }
    updateSummary();
  });

  manualRobotSelect.addEventListener("change", (event) => {
    state.manualRobotId = Number(event.target.value);
    updateSummary();
  });

  document.querySelectorAll(".dpad-btn").forEach((button) => {
    button.addEventListener("click", () => {
      sendCmdVel(button.dataset.cmd);
    });
  });

  document.getElementById("startAutoBtn").addEventListener("click", () => {
    const preset = document.getElementById("missionPreset").value;
    callSupervisorService(true, Array.from(state.autonomousRobotIds));
    appendConsoleLine(
      `[auto] Start ${preset} (SCT) on ${state.autonomousRobotIds.size || 0} robots.`
    );
  });

  document.getElementById("stopAutoBtn").addEventListener("click", () => {
    callSupervisorService(false, Array.from(state.autonomousRobotIds));
    appendConsoleLine("[auto] Stop autonomous pipeline.");
  });
}

renderStartupCards();
renderAutonomousOptions();
renderManualSelect();
updateSummary();
initHandlers();
connectRosBridge();
