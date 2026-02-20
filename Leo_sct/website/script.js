const DEFAULT_ROSBRIDGE_URL = "ws://192.168.178.141:9090";

const ROBOT_CONFIG_PATH = "./robots.json";
const TWIST_MSG_TYPE = "geometry_msgs/msg/Twist";

let robots = [];

const state = {
  mode: "startup",
  manualRobotId: null,
  autonomousRobotIds: new Set(),
  autonomousActiveIds: new Set(),
};

const startupGrid = document.getElementById("startupGrid");
const autonomousGrid = document.getElementById("autonomousGrid");
const manualRobotSelect = document.getElementById("manualRobotSelect");
const cmdPreview = document.getElementById("cmdPreview");
const activeSummary = document.getElementById("activeSummary");
const connectionStatus = document.getElementById("connectionStatus");

const rosSockets = new Map();
const connectedRobotIds = new Set();
const advertisedTopicsByRobot = new Map();
let activeManualCommand = null;
let manualCommandTimer = null;

function buildFallbackRobots() {
  return Array.from({ length: 4 }, (_, index) => ({
    name: `Leo-${index + 1}`,
    ns: `robot_${index}`,
    cmd_topic: `/robot_${index}/cmd_vel`,
    ws_url: DEFAULT_ROSBRIDGE_URL,
  }));
}

function normalizeRobots(configRobots) {
  return configRobots.map((robot, index) => ({
    id: index + 1,
    name: String(robot.name ?? `Leo-${index + 1}`),
    ns: String(robot.ns ?? robot.namespace ?? `robot_${index}`).replace(/^\/+|\/+$/g, ""),
    cmdTopic: String(robot.cmd_topic ?? robot.cmdTopic ?? `/${robot.ns ?? robot.namespace ?? `robot_${index}`}/cmd_vel`),
    wsUrl: String(robot.ws_url ?? robot.wsUrl ?? robot.rosbridge_url ?? DEFAULT_ROSBRIDGE_URL),
    isOn: false,
  }));
}

function cmdVelTopic(robot) {
  return `/${String(robot.cmdTopic ?? `/${robot.ns}/cmd_vel`).replace(/^\/+|\/+$/g, "")}`;
}

async function loadRobotsConfig() {
  try {
    const response = await fetch(ROBOT_CONFIG_PATH, { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const config = await response.json();
    const rawRobots = Array.isArray(config?.robots) ? config.robots : [];
    if (!rawRobots.length) {
      throw new Error("robots list is empty");
    }
    robots = normalizeRobots(rawRobots);
    appendConsoleLine(`[cfg] Loaded ${robots.length} robots from robots.json`);
  } catch (error) {
    robots = normalizeRobots(buildFallbackRobots());
    appendConsoleLine(
      `[cfg] Using fallback robot config (${robots.length} robots). Reason: ${error.message}`
    );
  }
}

function initializeRobotState() {
  state.autonomousRobotIds.clear();
  state.autonomousActiveIds.clear();
  state.manualRobotId = robots[0]?.id ?? null;
}

function updateConnectionStatus() {
  const connectedCount = connectedRobotIds.size;
  const total = robots.length;
  if (!total) {
    connectionStatus.textContent = "ROS2: No robots configured";
    return;
  }
  connectionStatus.textContent = `ROS2: ${connectedCount}/${total} connected`;
}

function sendRosMessage(robotId, payload) {
  const socket = rosSockets.get(robotId);
  if (!socket) {
    return { ok: false, reason: "socket_not_created" };
  }
  if (socket.readyState !== WebSocket.OPEN) {
    return { ok: false, reason: `socket_not_open(state=${socket.readyState})` };
  }
  try {
    socket.send(JSON.stringify(payload));
    return { ok: true };
  } catch (error) {
    return { ok: false, reason: `send_failed(${error.message})` };
  }
}

function advertiseTopic(robotId, topic, type) {
  let advertisedTopics = advertisedTopicsByRobot.get(robotId);
  if (!advertisedTopics) {
    advertisedTopics = new Set();
    advertisedTopicsByRobot.set(robotId, advertisedTopics);
  }
  if (advertisedTopics.has(topic)) return;
  const result = sendRosMessage(robotId, { op: "advertise", topic, type });
  if (result.ok) {
    advertisedTopics.add(topic);
  } else {
    appendConsoleLine(`[drop] advertise ${topic}: ${result.reason}`);
  }
}

function connectRobotBridge(robot) {
  const existingSocket = rosSockets.get(robot.id);
  if (existingSocket) {
    existingSocket.close();
  }
  const socket = new WebSocket(robot.wsUrl);
  rosSockets.set(robot.id, socket);
  socket.addEventListener("open", () => {
    connectedRobotIds.add(robot.id);
    updateConnectionStatus();
    advertiseTopic(robot.id, cmdVelTopic(robot), TWIST_MSG_TYPE);
    appendConsoleLine(`[ros] Connected ${robot.name} via ${robot.wsUrl}`);
  });
  socket.addEventListener("message", (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data?.op === "status" && data?.level === "error") {
        appendConsoleLine(`[ros] ${robot.name} status error: ${data.msg ?? "unknown"}`);
      }
    } catch (error) {
      // ignore non-JSON payloads from bridge
    }
  });
  socket.addEventListener("close", () => {
    connectedRobotIds.delete(robot.id);
    advertisedTopicsByRobot.delete(robot.id);
    updateConnectionStatus();
    appendConsoleLine(`[ros] Disconnected ${robot.name}`);
  });
  socket.addEventListener("error", () => {
    connectedRobotIds.delete(robot.id);
    updateConnectionStatus();
    appendConsoleLine(`[ros] Error on ${robot.name} (${robot.wsUrl})`);
  });
}

function connectRosBridges() {
  robots.forEach((robot) => connectRobotBridge(robot));
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
  if (state.manualRobotId === null && robots.length) {
    state.manualRobotId = robots[0].id;
  }
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
  if (!connectedRobotIds.has(robot.id)) {
    appendConsoleLine(`[ros] ${robot.name} bridge not connected.`);
    return;
  }
  const topic = cmdVelTopic(robot);
  const msg = {
    linear: { x: linearX, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angularZ },
  };
  const result = sendRosMessage(robot.id, { op: "publish", topic, type: TWIST_MSG_TYPE, msg });
  if (result.ok) {
    appendConsoleLine(`[pub] ${topic} -> linear.x=${linearX.toFixed(2)}, angular.z=${angularZ.toFixed(2)}`);
  } else {
    appendConsoleLine(`[drop] ${topic}: ${result.reason}`);
  }
}

function callSupervisorService(enabled, robotIds) {
  if (!connectedRobotIds.size) {
    appendConsoleLine("No ROS bridges connected.");
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
    const result = sendRosMessage(robot.id, {
      op: "call_service",
      service: `/${robot.ns}/enable_supervisor`,
      type: "std_srvs/SetBool",
      args: { data: enabled },
    });
    if (!result.ok) {
      appendConsoleLine(`[drop] service /${robot.ns}/enable_supervisor: ${result.reason}`);
    }
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
  if (!robot) {
    appendConsoleLine("[cmd_vel] Manual drive blocked: no robot selected.");
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

function startManualCommand(command) {
  if (!command || command === "stop") {
    stopManualCommand();
    return;
  }
  if (activeManualCommand === command && manualCommandTimer) {
    return;
  }
  if (manualCommandTimer) {
    clearInterval(manualCommandTimer);
    manualCommandTimer = null;
  }
  activeManualCommand = command;
  sendCmdVel(command);
  manualCommandTimer = setInterval(() => {
    sendCmdVel(command);
  }, 100);
  appendConsoleLine(`[manual] Holding '${command}' (press Stop to release).`);
}

function stopManualCommand() {
  if (manualCommandTimer) {
    clearInterval(manualCommandTimer);
    manualCommandTimer = null;
  }
  if (activeManualCommand === null) return;
  sendCmdVel("stop");
  activeManualCommand = null;
  appendConsoleLine("[manual] Released.");
}

function updateRobotPower(id, power) {
  const robot = robots.find((entry) => entry.id === id);
  if (!robot) return;
  robot.isOn = power;
  renderStartupCards();
}

function initHandlers() {
  connectionStatus.addEventListener("click", () => {
    appendConsoleLine("Reconnecting all robot bridges...");
    connectRosBridges();
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
      appendConsoleLine("[ui] Robot marked active for manual control.");
    }
    if (target.dataset.action === "stop") {
      updateRobotPower(id, false);
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
    const command = button.dataset.cmd;

    if (command === "stop") {
      button.addEventListener("click", stopManualCommand);
      return;
    }

    button.addEventListener("mousedown", (event) => {
      event.preventDefault();
      startManualCommand(command);
    });

    button.addEventListener("mouseup", (event) => {
      event.preventDefault();
      stopManualCommand();
    });

    button.addEventListener("mouseleave", stopManualCommand);

    // optional for touchscreens
    button.addEventListener("touchstart", (event) => {
      event.preventDefault();
      startManualCommand(command);
    });

    button.addEventListener("touchend", (event) => {
      event.preventDefault();
      stopManualCommand();
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

async function initializeApp() {
  await loadRobotsConfig();
  initializeRobotState();
  updateConnectionStatus();
  renderStartupCards();
  renderAutonomousOptions();
  renderManualSelect();
  updateSummary();
  initHandlers();
  connectRosBridges();
}

initializeApp();

window.addEventListener("beforeunload", () => {
  robots.forEach(robot => publishCmdVel(robot, 0.0, 0.0));
});
