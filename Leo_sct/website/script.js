const DEFAULT_ROSBRIDGE_URL = "ws://192.168.178.141:9090";

const ROBOT_CONFIG_PATH = "./robots.json";
const SESSION_ENDPOINT = "/api/session";
const CONNECT_INFO_ENDPOINT = "/api/connect_info";
const QR_ENDPOINT = "/api/qr";
const CONTROL_CLAIM_ENDPOINT = "/api/control/claim";
const CONTROL_RELEASE_ENDPOINT = "/api/control/release";
const CONTROL_HEARTBEAT_ENDPOINT = "/api/control/heartbeat";
const EMERGENCY_ACTIVATE_ENDPOINT = "/api/emergency/activate";

const DEFAULT_REMOTE_SESSION_TIMEOUT_MS = 5 * 60 * 1000;
const DEFAULT_LEASE_TIMEOUT_MS = 12 * 1000;
const SESSION_POLL_MS = 2000;
const LEASE_HEARTBEAT_MS = 4000;

const TWIST_MSG_TYPE = "geometry_msgs/msg/Twist";
const SET_BOOL_SRV_TYPE = "std_srvs/srv/SetBool";
const MISSION_LABELS = {
  explore: "Explore",
  find_marker: "Find Marker",
};
const ROBOT_COLOR_FALLBACKS = ["#7e57c2", "#2e7d32", "#c62828", "#1565c0"];
const ROBOT_SQUARE_SYMBOLS = ["🟪", "🟩", "🟥", "🟦"];

let robots = [];

const state = {
  mode: "autonomous",
  manualRobotId: null,
  autonomousRobotIds: new Set(),
  autonomousActiveIds: new Set(),
  autonomousRunning: false,
  autonomousMissionPreset: "explore",
  session: {
    localEmergency: false,
    timeoutMs: DEFAULT_REMOTE_SESSION_TIMEOUT_MS,
    expiresAtMs: null,
    expired: false,
    source: "unknown",
  },
  control: {
    leaseTimeoutMs: DEFAULT_LEASE_TIMEOUT_MS,
    leasedRobotIds: new Set(),
    emergencyActive: false,
  },
};

const autonomousGrid = document.getElementById("autonomousGrid");
const manualRobotSelect = document.getElementById("manualRobotSelect");
const cmdPreview = document.getElementById("cmdPreview");
const activeSummary = document.getElementById("activeSummary");
const connectionStatus = document.getElementById("connectionStatus");
const sessionStatus = document.getElementById("sessionStatus");
const sessionOverlay = document.getElementById("sessionOverlay");
const connectOptionRow = document.getElementById("connectOptionRow");
const allStopBtn = document.getElementById("allStopBtn");
const phoneQrImage = document.getElementById("phoneQrImage");
const phoneUrlLink = document.getElementById("phoneUrlLink");
const qrStatus = document.getElementById("qrStatus");
const phoneConnectCard = document.getElementById("phoneConnectCard");

const rosSockets = new Map();
const connectedRobotIds = new Set();
const advertisedTopicsByRobot = new Map();
const pendingServiceCalls = new Map();
let activeManualCommand = null;
let manualCommandTimer = null;
let sessionTickerTimer = null;
let sessionExpiryTimer = null;
let sessionPollTimer = null;
let leaseHeartbeatTimer = null;

function robotKey(robotId) {
  return String(robotId);
}

function isLoopbackHostname() {
  const host = window.location.hostname;
  return host === "localhost" || host === "127.0.0.1" || host === "::1" || host === "[::1]";
}

function formatCountdown(ms) {
  const totalSeconds = Math.max(0, Math.floor(ms / 1000));
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return `${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}`;
}

function isMobileViewport() {
  return window.matchMedia("(max-width: 720px)").matches;
}

function clearSessionTimers() {
  if (sessionTickerTimer) {
    clearInterval(sessionTickerTimer);
    sessionTickerTimer = null;
  }
  if (sessionExpiryTimer) {
    clearTimeout(sessionExpiryTimer);
    sessionExpiryTimer = null;
  }
}

function clearControlTimers() {
  if (sessionPollTimer) {
    clearInterval(sessionPollTimer);
    sessionPollTimer = null;
  }
  if (leaseHeartbeatTimer) {
    clearInterval(leaseHeartbeatTimer);
    leaseHeartbeatTimer = null;
  }
}

function updateSessionStatus() {
  if (state.session.localEmergency) {
    sessionStatus.textContent = "Session: local operator";
    return;
  }
  if (state.session.expired) {
    sessionStatus.textContent = "Session: expired";
    return;
  }
  if (typeof state.session.expiresAtMs === "number") {
    const remaining = state.session.expiresAtMs - Date.now();
    const countdown = formatCountdown(remaining);
    sessionStatus.textContent = isMobileViewport()
      ? `Session ends in ${countdown}`
      : `Session: ${countdown} left`;
    return;
  }
  sessionStatus.textContent = "Session: active";
}

function applySessionPolicyUI() {
  if (state.session.localEmergency) {
    allStopBtn.hidden = false;
    allStopBtn.disabled = false;
  } else {
    allStopBtn.hidden = true;
    allStopBtn.disabled = true;
  }

  document.body.classList.toggle("session-expired", state.session.expired);
  document.body.classList.toggle("local-operator", state.session.localEmergency);
  if (sessionOverlay) {
    sessionOverlay.hidden = !state.session.expired;
  }
  if (phoneConnectCard) {
    phoneConnectCard.hidden = !state.session.localEmergency;
  }
  if (connectOptionRow) {
    connectOptionRow.hidden = !state.session.localEmergency;
  }
  updateSessionStatus();
}

function sessionAllowsControls() {
  return !state.session.expired;
}

function guardSession(actionLabel) {
  if (sessionAllowsControls()) return true;
  appendConsoleLine(`[session] ${actionLabel} blocked: session expired.`);
  return false;
}

function hasLease(robotId) {
  return state.control.leasedRobotIds.has(robotKey(robotId));
}

function canBypassLease() {
  return state.session.localEmergency && state.control.emergencyActive;
}

function disconnectRosBridges() {
  rosSockets.forEach((socket) => {
    try {
      socket.close();
    } catch (error) {
      // ignore
    }
  });
}

async function apiPost(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    cache: "no-store",
    body: JSON.stringify(body ?? {}),
  });
  let payload = {};
  try {
    payload = await response.json();
  } catch (error) {
    payload = {};
  }
  return { ok: response.ok, status: response.status, payload };
}

function parseSessionResponse(payload) {
  const timeoutSeconds = Number(payload?.session_timeout_seconds);
  const leaseSeconds = Number(payload?.lease_timeout_seconds);
  const timeoutMs = Number.isFinite(timeoutSeconds) && timeoutSeconds > 0
    ? timeoutSeconds * 1000
    : DEFAULT_REMOTE_SESSION_TIMEOUT_MS;

  state.session.localEmergency = Boolean(payload?.local_emergency);
  state.session.timeoutMs = timeoutMs;
  state.session.expiresAtMs = payload?.expires_at ? Date.parse(String(payload.expires_at)) : null;
  state.session.expired = Boolean(payload?.expired);
  state.session.source = "server";

  state.control.leaseTimeoutMs = Number.isFinite(leaseSeconds) && leaseSeconds > 0
    ? leaseSeconds * 1000
    : DEFAULT_LEASE_TIMEOUT_MS;
  state.control.emergencyActive = Boolean(payload?.emergency_active);
  state.control.leasedRobotIds = new Set(
    Array.isArray(payload?.owned_robot_ids) ? payload.owned_robot_ids.map((x) => String(x)) : []
  );

  if (typeof state.session.expiresAtMs === "number" && Number.isNaN(state.session.expiresAtMs)) {
    state.session.expiresAtMs = Date.now() + state.session.timeoutMs;
  }

  if (!state.session.localEmergency && !state.session.expiresAtMs && !state.session.expired) {
    state.session.expiresAtMs = Date.now() + state.session.timeoutMs;
  }

  if (!state.session.localEmergency && typeof state.session.expiresAtMs === "number") {
    state.session.expired = state.session.expired || Date.now() >= state.session.expiresAtMs;
  }

  if (!state.session.localEmergency && state.control.emergencyActive) {
    state.session.expired = true;
  }
}

async function refreshSessionPolicy(renew = false) {
  const sessionUrl = renew ? `${SESSION_ENDPOINT}?renew=1` : SESSION_ENDPOINT;
  const response = await fetch(sessionUrl, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`HTTP ${response.status}`);
  }
  const payload = await response.json();
  parseSessionResponse(payload);
  applySessionPolicyUI();
  scheduleSessionTimeout();
}

async function loadSessionPolicy() {
  try {
    const localClient = isLoopbackHostname();
    await refreshSessionPolicy(localClient);

    appendConsoleLine(
      state.session.localEmergency
        ? "[session] Local operator session active."
        : "[session] Remote session active (5-minute timeout)."
    );
  } catch (error) {
    const localEmergency = isLoopbackHostname();
    state.session.localEmergency = localEmergency;
    state.session.timeoutMs = DEFAULT_REMOTE_SESSION_TIMEOUT_MS;
    state.session.expiresAtMs = localEmergency ? null : Date.now() + state.session.timeoutMs;
    state.session.expired = false;
    state.session.source = "browser_fallback";
    appendConsoleLine(
      `[session] Fallback policy (${localEmergency ? "local" : "remote"}) due to session API error: ${error.message}`
    );
    applySessionPolicyUI();
    scheduleSessionTimeout();
  }

  if (state.session.expired) {
    expireSession(state.control.emergencyActive
      ? "Emergency stop activated by local operator."
      : "5-minute session window ended.");
  }
}

function startSessionPolling() {
  if (sessionPollTimer) clearInterval(sessionPollTimer);
  sessionPollTimer = setInterval(async () => {
    if (!sessionAllowsControls()) return;
    try {
      await refreshSessionPolicy(false);
      if (state.session.expired) {
        expireSession(state.control.emergencyActive
          ? "Emergency stop activated by local operator."
          : "5-minute session window ended.");
      }
    } catch (error) {
      appendConsoleLine(`[session] poll failed: ${error.message}`);
    }
  }, SESSION_POLL_MS);
}

function startLeaseHeartbeat() {
  if (leaseHeartbeatTimer) clearInterval(leaseHeartbeatTimer);
  leaseHeartbeatTimer = setInterval(async () => {
    if (!sessionAllowsControls()) return;
    const robotIds = Array.from(state.control.leasedRobotIds);
    if (!robotIds.length) return;
    const result = await apiPost(CONTROL_HEARTBEAT_ENDPOINT, { robot_ids: robotIds });
    if (!result.ok) {
      const err = result.payload?.error ?? `HTTP ${result.status}`;
      if (err === "session_expired" || err === "emergency_active") {
        const sessionPayload = result.payload?.session;
        if (sessionPayload) parseSessionResponse(sessionPayload);
        expireSession(err === "emergency_active"
          ? "Emergency stop activated by local operator."
          : "5-minute session window ended.");
      }
      return;
    }
    if (result.payload?.session) {
      parseSessionResponse(result.payload.session);
      applySessionPolicyUI();
    }
  }, LEASE_HEARTBEAT_MS);
}

async function claimRobotLease(robotId, contextLabel) {
  if (!guardSession(`Claim control for robot ${robotId}`)) return false;
  if (hasLease(robotId)) return true;

  const result = await apiPost(CONTROL_CLAIM_ENDPOINT, { robot_id: robotKey(robotId) });
  if (!result.ok) {
    const err = result.payload?.error ?? `HTTP ${result.status}`;
    if (result.payload?.session) {
      parseSessionResponse(result.payload.session);
      applySessionPolicyUI();
    }
    if (err === "robot_busy") {
      const robot = robots.find((entry) => entry.id === Number(robotId));
      appendConsoleLine(`[lock] ${robot?.name ?? `Robot ${robotId}`} is controlled by another client.`);
    } else if (err === "emergency_active") {
      expireSession("Emergency stop activated by local operator.");
    } else if (err === "session_expired") {
      expireSession("5-minute session window ended.");
    } else {
      appendConsoleLine(`[lock] Could not claim robot ${robotId} for ${contextLabel}: ${err}`);
    }
    return false;
  }

  if (result.payload?.session) {
    parseSessionResponse(result.payload.session);
    applySessionPolicyUI();
  }
  state.control.leasedRobotIds.add(robotKey(robotId));
  return true;
}

async function claimRobotLeases(robotIds, contextLabel) {
  const granted = [];
  for (const rid of robotIds) {
    const ok = await claimRobotLease(rid, contextLabel);
    if (ok) granted.push(rid);
  }
  return granted;
}

async function releaseRobotLeases(robotIds) {
  const ids = Array.isArray(robotIds) ? robotIds.map((x) => robotKey(x)) : [];
  if (!ids.length && !state.control.leasedRobotIds.size) return;
  const result = await apiPost(CONTROL_RELEASE_ENDPOINT, { robot_ids: ids });
  if (result.ok && result.payload?.session) {
    parseSessionResponse(result.payload.session);
    applySessionPolicyUI();
  }
  if (ids.length) {
    ids.forEach((id) => state.control.leasedRobotIds.delete(id));
  } else {
    state.control.leasedRobotIds.clear();
  }
}

async function activateEmergencyLatch() {
  const result = await apiPost(EMERGENCY_ACTIVATE_ENDPOINT, {});
  if (!result.ok) {
    const err = result.payload?.error ?? `HTTP ${result.status}`;
    appendConsoleLine(`[safety] Emergency activate failed: ${err}`);
    return false;
  }
  if (result.payload?.session) {
    parseSessionResponse(result.payload.session);
    applySessionPolicyUI();
  }
  return true;
}

function expireSession(reason) {
  if (state.session.expired) return;
  clearSessionTimers();
  clearControlTimers();
  stopManualCommand();
  disconnectRosBridges();
  state.session.expired = true;
  state.autonomousRunning = false;
  pendingServiceCalls.clear();
  state.control.leasedRobotIds.clear();
  applySessionPolicyUI();
  appendConsoleLine(`[session] ${reason}`);
}

function scheduleSessionTimeout() {
  clearSessionTimers();

  if (state.session.localEmergency || state.session.expired || typeof state.session.expiresAtMs !== "number") {
    updateSessionStatus();
    return;
  }

  const remainingMs = state.session.expiresAtMs - Date.now();
  if (remainingMs <= 0) {
    expireSession("5-minute session window ended.");
    return;
  }

  sessionTickerTimer = setInterval(updateSessionStatus, 1000);
  sessionExpiryTimer = setTimeout(() => {
    expireSession("5-minute session window ended.");
  }, remainingMs);
  updateSessionStatus();
}

function buildFallbackRobots() {
  return Array.from({ length: 4 }, (_, index) => ({
    name: ["Purple", "Green", "Red", "Blue"][index] ?? `Leo-${index + 1}`,
    color: ROBOT_COLOR_FALLBACKS[index % ROBOT_COLOR_FALLBACKS.length],
    ns: `robot_${index}`,
    cmd_topic: `/robot_${index}/cmd_vel`,
    ws_url: DEFAULT_ROSBRIDGE_URL,
  }));
}

function normalizeRobots(configRobots) {
  return configRobots.map((robot, index) => ({
    id: index + 1,
    name: String(robot.name ?? `Leo-${index + 1}`),
    color: String(robot.color ?? ROBOT_COLOR_FALLBACKS[index % ROBOT_COLOR_FALLBACKS.length]),
    ns: String(robot.ns ?? robot.namespace ?? `robot_${index}`).replace(/^\/+|\/+$/g, ""),
    cmdTopic: String(robot.cmd_topic ?? robot.cmdTopic ?? `/${robot.ns ?? robot.namespace ?? `robot_${index}`}/cmd_vel`),
    wsUrl: String(robot.ws_url ?? robot.wsUrl ?? robot.rosbridge_url ?? DEFAULT_ROSBRIDGE_URL),
  }));
}

function cmdVelTopic(robot) {
  return `/${String(robot.cmdTopic ?? `/${robot.ns}/cmd_vel`).replace(/^\/+|\/+$/g, "")}`;
}

function missionServiceNames(missionPreset, enabled) {
  if (missionPreset === "find_marker") return ["enable_supervisor_find_marker"];
  if (missionPreset === "explore") return ["enable_supervisor_explore"];
  if (!enabled) return ["enable_supervisor_explore", "enable_supervisor_find_marker"];
  return ["enable_supervisor_explore"];
}

function supervisorServiceCandidates(robot, missionPreset, enabled) {
  const candidates = [];
  const ns = String(robot.ns ?? "").replace(/^\/+|\/+$/g, "");
  const topic = cmdVelTopic(robot);
  const serviceNames = missionServiceNames(missionPreset, enabled);

  serviceNames.forEach((serviceName) => {
    if (ns) {
      candidates.push(`/${ns}/${serviceName}`);
    }

    if (topic.endsWith("/cmd_vel")) {
      const topicPrefix = topic.slice(0, -"/cmd_vel".length);
      if (topicPrefix && topicPrefix !== "/") {
        candidates.push(`${topicPrefix}/${serviceName}`);
      } else {
        candidates.push(`/${serviceName}`);
      }
    }
  });

  if (ns) {
    candidates.push(`/${ns}/enable_supervisor`);
  }

  if (topic.endsWith("/cmd_vel")) {
    const topicPrefix = topic.slice(0, -"/cmd_vel".length);
    if (topicPrefix && topicPrefix !== "/") {
      candidates.push(`${topicPrefix}/enable_supervisor`);
    } else {
      candidates.push("/enable_supervisor");
    }
  }

  candidates.push("/enable_supervisor");
  return [...new Set(candidates)];
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
  state.manualRobotId = null;
}

function robotSelectionMode(robotId) {
  const isManual = state.manualRobotId === robotId;
  const isAutonomous = state.autonomousRobotIds.has(robotId);
  if (isManual && isAutonomous) return "manual/autonomous";
  if (isManual) return "manual";
  if (isAutonomous) return "autonomous";
  return "idle";
}

function updateConnectionStatus() {
  if (!robots.length) {
    connectionStatus.innerHTML = "";
    const emptyCard = document.createElement("div");
    emptyCard.className = "robot-status-card is-offline";
    emptyCard.textContent = "No robots configured";
    connectionStatus.appendChild(emptyCard);
    return;
  }

  connectionStatus.innerHTML = "";
  robots.forEach((robot) => {
    const connected = connectedRobotIds.has(robot.id);
    const card = document.createElement("div");
    card.className = `robot-status-card ${connected ? "is-online" : "is-offline"}`;

    const swatch = document.createElement("span");
    swatch.className = "robot-status-swatch";
    swatch.setAttribute("aria-hidden", "true");
    swatch.style.background = robot.color;

    const name = document.createElement("span");
    name.className = "robot-status-name";
    name.textContent = connected ? `${robot.name}:` : robot.name;

    const meta = document.createElement("span");
    meta.className = "robot-status-meta";
    meta.textContent = connected ? robotSelectionMode(robot.id) : "";

    card.append(swatch, name, meta);
    connectionStatus.appendChild(card);
  });
}

function sendRosMessage(robotId, payload) {
  if (!sessionAllowsControls()) {
    return { ok: false, reason: "session_expired" };
  }
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
  if (!sessionAllowsControls()) return;

  const existingSocket = rosSockets.get(robot.id);
  if (existingSocket) {
    existingSocket.close();
  }

  const socket = new WebSocket(robot.wsUrl);
  rosSockets.set(robot.id, socket);

  socket.addEventListener("open", () => {
    connectedRobotIds.add(robot.id);
    updateConnectionStatus();
    renderAutonomousOptions();
    renderManualSelect();
    updateSummary();
    advertiseTopic(robot.id, cmdVelTopic(robot), TWIST_MSG_TYPE);
    appendConsoleLine(`[ros] Connected ${robot.name} via ${robot.wsUrl}`);
  });

  socket.addEventListener("message", (event) => {
    try {
      const data = JSON.parse(event.data);
      if (data?.op === "status" && data?.level === "error") {
        appendConsoleLine(`[ros] ${robot.name} status error: ${data.msg ?? "unknown"}`);
      }
      if (data?.op === "service_response" && typeof data?.id === "string" && data.id.startsWith("svc:")) {
        const meta = pendingServiceCalls.get(data.id);
        if (!meta) return;
        pendingServiceCalls.delete(data.id);
        const ok = data.result === true;
        const msg = data.values?.message ? ` (${data.values.message})` : "";
        appendConsoleLine(
          ok
            ? `[svc] ${meta.robotName} ${meta.service} -> success${msg}`
            : `[svc] ${meta.robotName} ${meta.service} -> failed${msg}`
        );
      }
    } catch (error) {
      // ignore non-JSON payloads from bridge
    }
  });

  socket.addEventListener("close", () => {
    connectedRobotIds.delete(robot.id);
    advertisedTopicsByRobot.delete(robot.id);
    updateConnectionStatus();
    renderAutonomousOptions();
    renderManualSelect();
    updateSummary();
    appendConsoleLine(`[ros] Disconnected ${robot.name}`);
  });

  socket.addEventListener("error", () => {
    connectedRobotIds.delete(robot.id);
    updateConnectionStatus();
    renderAutonomousOptions();
    renderManualSelect();
    updateSummary();
    appendConsoleLine(`[ros] Error on ${robot.name} (${robot.wsUrl})`);
  });
}

function connectRosBridges() {
  if (!guardSession("Reconnect ROS bridges")) return;
  robots.forEach((robot) => connectRobotBridge(robot));
}

function renderAutonomousOptions() {
  if (!autonomousGrid) return;
  const connectedRobots = robots.filter((robot) => connectedRobotIds.has(robot.id));
  const connectedIds = new Set(connectedRobots.map((robot) => robot.id));
  state.autonomousRobotIds.forEach((id) => {
    if (!connectedIds.has(id)) {
      state.autonomousRobotIds.delete(id);
    }
  });

  autonomousGrid.innerHTML = "";
  connectedRobots.forEach((robot) => {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "auto-robot-btn";
    if (state.autonomousRobotIds.has(robot.id)) {
      button.classList.add("is-selected");
    }
    button.dataset.id = String(robot.id);
    button.style.setProperty("--robot-color", robot.color || ROBOT_COLOR_FALLBACKS[index % ROBOT_COLOR_FALLBACKS.length]);

    const swatch = document.createElement("span");
    swatch.className = "robot-color-swatch";
    swatch.setAttribute("aria-hidden", "true");

    const label = document.createElement("span");
    label.textContent = robot.name;

    button.append(swatch, label);
    autonomousGrid.appendChild(button);
  });
}

function renderManualSelect() {
  if (!manualRobotSelect) return;
  const connectedRobots = robots.filter((robot) => connectedRobotIds.has(robot.id));
  if (state.manualRobotId !== null && !connectedRobots.some((robot) => robot.id === state.manualRobotId)) {
    state.manualRobotId = null;
  }

  manualRobotSelect.innerHTML = "";

  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = "Select Robot";
  placeholder.disabled = true;
  placeholder.selected = state.manualRobotId === null;
  manualRobotSelect.appendChild(placeholder);

  connectedRobots.forEach((robot, index) => {
    const originalIndex = Math.max(0, robots.findIndex((entry) => entry.id === robot.id));
    const option = document.createElement("option");
    option.value = robot.id;
    option.textContent = `${ROBOT_SQUARE_SYMBOLS[originalIndex % ROBOT_SQUARE_SYMBOLS.length]} ${robot.name}`;
    if (state.manualRobotId !== null && robot.id === state.manualRobotId) {
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
  updateConnectionStatus();
}

function appendConsoleLine(text) {
  cmdPreview.innerHTML = "";
  const line = document.createElement("div");
  line.className = "console-line";
  line.textContent = text;
  cmdPreview.appendChild(line);
}

async function initializePhoneQr() {
  if (!phoneQrImage || !phoneUrlLink || !qrStatus) return;
  try {
    const response = await fetch(CONNECT_INFO_ENDPOINT, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const info = await response.json();
    const phoneUrl = String(info.phone_url || window.location.href);
    phoneUrlLink.href = phoneUrl;
    phoneUrlLink.textContent = phoneUrl;
    qrStatus.textContent = "Scan this QR code from your phone.";

    const localQrUrl = `${QR_ENDPOINT}?url=${encodeURIComponent(phoneUrl)}`;
    phoneQrImage.src = localQrUrl;
    phoneQrImage.alt = `QR code for ${phoneUrl}`;
    phoneQrImage.onerror = () => {
      const fallback = `https://api.qrserver.com/v1/create-qr-code/?size=220x220&data=${encodeURIComponent(phoneUrl)}`;
      phoneQrImage.src = fallback;
      qrStatus.textContent = "Using online QR fallback.";
    };
  } catch (error) {
    qrStatus.textContent = `QR unavailable: ${error.message}`;
  }
}

function switchMode(nextMode) {
  if (!guardSession("Switch mode")) return;
  state.mode = nextMode;
  document.querySelectorAll(".mode-btn").forEach((button) => {
    button.classList.toggle("is-active", button.dataset.mode === nextMode);
    button.setAttribute("aria-selected", button.dataset.mode === nextMode);
  });
  document.querySelectorAll("[data-mode-panel]").forEach((panel) => {
    panel.classList.toggle("is-hidden", panel.dataset.modePanel !== nextMode);
  });
}

function getInitialModeFromQuery() {
  const mode = new URLSearchParams(window.location.search).get("mode");
  return mode === "manual" || mode === "autonomous" ? mode : null;
}

function publishCmdVel(robot, linearX, angularZ, bypassLease = false, logOutput = true) {
  if (!guardSession("Publish cmd_vel")) return;
  if (!bypassLease && !hasLease(robot.id) && !canBypassLease()) {
    appendConsoleLine(`[lock] ${robot.name} blocked: this client does not hold control.`);
    return;
  }
  if (!connectedRobotIds.has(robot.id)) {
    if (logOutput) appendConsoleLine(`[ros] ${robot.name} bridge not connected.`);
    return;
  }
  const topic = cmdVelTopic(robot);
  const msg = {
    linear: { x: linearX, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: angularZ },
  };
  const result = sendRosMessage(robot.id, { op: "publish", topic, type: TWIST_MSG_TYPE, msg });
  if (result.ok) {
    if (logOutput) {
      appendConsoleLine(`[pub] ${topic} -> linear.x=${linearX.toFixed(2)}, angular.z=${angularZ.toFixed(2)}`);
    }
  } else {
    if (logOutput) appendConsoleLine(`[drop] ${topic}: ${result.reason}`);
  }
}

function callSupervisorService(enabled, robotIds, missionPreset, bypassLease = false) {
  if (!guardSession("Supervisor service call")) return false;
  if (!connectedRobotIds.size) {
    appendConsoleLine("No ROS bridges connected.");
    return false;
  }
  if (!robotIds.length) {
    appendConsoleLine("Select autonomous robots before enabling SCT.");
    return false;
  }
  const names = robotIds
    .map((id) => robots.find((robot) => robot.id === id))
    .filter(Boolean)
    .filter((robot) => {
      if (bypassLease || hasLease(robot.id) || canBypassLease()) return true;
      appendConsoleLine(`[lock] ${robot.name} blocked: no control lease.`);
      return false;
    });

  if (!names.length) return false;

  names.forEach((robot) => {
    const services = supervisorServiceCandidates(robot, missionPreset, enabled);
    let sent = false;
    services.forEach((service, idx) => {
      const callId = `svc:${robot.id}:${Date.now()}:${idx}`;
      const result = sendRosMessage(robot.id, {
        op: "call_service",
        id: callId,
        service,
        type: SET_BOOL_SRV_TYPE,
        args: { data: enabled },
      });
      if (!result.ok) {
        appendConsoleLine(`[drop] service ${service}: ${result.reason}`);
        return;
      }
      sent = true;
      pendingServiceCalls.set(callId, { robotName: robot.name, service });
    });
    if (!sent) {
      appendConsoleLine(`[auto] ${robot.name}: no service call sent.`);
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
  return names.length > 0;
}

function stopRobotMotion(robot, bypassLease = false) {
  publishCmdVel(robot, 0.0, 0.0, bypassLease);
  appendConsoleLine(`[cmd_vel] ${robot.name} stop (zero velocity).`);
}

function sendCmdVel(command, logOutput = true) {
  if (!guardSession("Manual drive")) return;
  const linear = Number(document.getElementById("linearSpeed").value);
  const angular = Number(document.getElementById("angularSpeed").value);
  const robot = robots.find((entry) => entry.id === state.manualRobotId);
  if (!robot) {
    if (logOutput) appendConsoleLine("[cmd_vel] Manual drive blocked: no robot selected.");
    return;
  }
  if (!hasLease(robot.id) && !canBypassLease()) {
    if (logOutput) appendConsoleLine(`[lock] Manual drive blocked: ${robot.name} lease not held.`);
    return;
  }
  if (state.autonomousActiveIds.has(robot.id)) {
    if (logOutput) appendConsoleLine("[cmd_vel] Manual drive blocked: robot is in autonomous mode.");
    return;
  }
  const payload = {
    robot: robot.name,
    linear: command === "forward" ? linear : command === "backward" ? -linear : 0,
    angular: command === "left" ? angular : command === "right" ? -angular : 0,
    stamp: new Date().toISOString(),
  };
  if (logOutput) appendConsoleLine(`[cmd_vel] ${JSON.stringify(payload)}`);
  publishCmdVel(robot, payload.linear, payload.angular, false, logOutput);
}

async function startManualCommand(command) {
  if (!guardSession("Start manual command")) return;
  if (!command || command === "stop") {
    await stopManualCommand();
    return;
  }

  const robot = robots.find((entry) => entry.id === state.manualRobotId);
  if (!robot) return;

  const claimed = await claimRobotLease(robot.id, "manual drive");
  if (!claimed) return;

  if (activeManualCommand === command && manualCommandTimer) {
    return;
  }
  if (manualCommandTimer) {
    clearInterval(manualCommandTimer);
    manualCommandTimer = null;
  }

  activeManualCommand = command;
  sendCmdVel(command, true);
  manualCommandTimer = setInterval(() => {
    sendCmdVel(command, false);
  }, 100);
  appendConsoleLine(`[manual] Holding '${command}' (press Stop to release).`);
}

async function stopManualCommand() {
  if (manualCommandTimer) {
    clearInterval(manualCommandTimer);
    manualCommandTimer = null;
  }
  if (activeManualCommand === null) return;

  sendCmdVel("stop");
  activeManualCommand = null;

  const robot = robots.find((entry) => entry.id === state.manualRobotId);
  if (robot) {
    await releaseRobotLeases([robot.id]);
  }

  appendConsoleLine("[manual] Released.");
}

async function runEmergencyStop(updateAutoButton) {
  if (!state.session.localEmergency) {
    appendConsoleLine("[safety] Emergency stop is only available from local operator access.");
    return;
  }
  if (!guardSession("Emergency stop")) return;

  const activated = await activateEmergencyLatch();
  if (!activated) return;

  await stopManualCommand();
  callSupervisorService(false, robots.map((robot) => robot.id), null, true);
  robots.forEach((robot) => {
    state.autonomousActiveIds.delete(robot.id);
    stopRobotMotion(robot, true);
  });
  state.autonomousRunning = false;
  if (typeof updateAutoButton === "function") updateAutoButton();
  appendConsoleLine("Emergency stop: autonomous disabled and all robots halted. Remote clients are blocked.");
}

function initHandlers() {
  const toggleAutoBtn = document.getElementById("toggleAutoBtn");
  const updateAutoButton = () => {
    toggleAutoBtn.textContent = state.autonomousRunning ? "Stop Autonomous" : "Start Autonomous";
  };

  connectionStatus.addEventListener("click", () => {
    if (!guardSession("Reconnect robot bridges")) return;
    appendConsoleLine("Reconnecting all robot bridges...");
    connectRosBridges();
  });

  document.querySelectorAll(".mode-btn").forEach((button) => {
    button.addEventListener("click", () => switchMode(button.dataset.mode));
  });

  allStopBtn.addEventListener("click", async () => {
    await runEmergencyStop(updateAutoButton);
  });

  autonomousGrid.addEventListener("click", async (event) => {
    if (!guardSession("Select autonomous robots")) return;
    const target = event.target;
    if (!(target instanceof HTMLButtonElement)) return;
    const id = Number(target.dataset.id);

    if (state.autonomousRobotIds.has(id)) {
      state.autonomousRobotIds.delete(id);
      target.classList.remove("is-selected");
      if (!state.autonomousRunning) {
        await releaseRobotLeases([id]);
      }
    } else {
      // Last selection wins: moving robot from manual to autonomous is allowed.
      if (state.manualRobotId === id) {
        await stopManualCommand();
        state.manualRobotId = null;
      }
      const claimed = await claimRobotLease(id, "autonomous selection");
      if (!claimed) return;
      state.autonomousRobotIds.add(id);
      target.classList.add("is-selected");
    }
    renderManualSelect();
    updateSummary();
  });

  manualRobotSelect.addEventListener("change", async (event) => {
    if (!guardSession("Change manual robot")) return;
    const nextValue = String(event.target.value ?? "");
    if (!nextValue) {
      await stopManualCommand();
      state.manualRobotId = null;
      updateSummary();
      return;
    }

    const nextManualId = Number(nextValue);
    // Last selection wins: moving robot from autonomous to manual is allowed.
    state.autonomousRobotIds.delete(nextManualId);

    await stopManualCommand();
    state.manualRobotId = nextManualId;
    await claimRobotLease(state.manualRobotId, "manual selection");
    renderAutonomousOptions();
    updateSummary();
  });

  document.querySelectorAll(".dpad-btn").forEach((button) => {
    const command = button.dataset.cmd;

    if (command === "stop") {
      button.addEventListener("click", async () => {
        if (!guardSession("Stop manual command")) return;
        await stopManualCommand();
      });
      return;
    }

    button.addEventListener("mousedown", async (event) => {
      event.preventDefault();
      await startManualCommand(command);
    });

    button.addEventListener("mouseup", async (event) => {
      event.preventDefault();
      await stopManualCommand();
    });

    button.addEventListener("mouseleave", async () => {
      await stopManualCommand();
    });

    button.addEventListener("touchstart", async (event) => {
      event.preventDefault();
      await startManualCommand(command);
    });

    button.addEventListener("touchend", async (event) => {
      event.preventDefault();
      await stopManualCommand();
    });
  });

  toggleAutoBtn.addEventListener("click", async () => {
    if (!guardSession("Toggle autonomous mode")) return;
    const preset = document.getElementById("missionPreset").value;
    const presetLabel = MISSION_LABELS[preset] ?? preset;
    const selectedIds = Array.from(state.autonomousRobotIds);

    if (!state.autonomousRunning) {
      const granted = await claimRobotLeases(selectedIds, "autonomous start");
      if (!granted.length) {
        appendConsoleLine("[auto] No robot lease available to start autonomous.");
        return;
      }
      const started = callSupervisorService(true, granted, preset);
      if (!started) return;
      state.autonomousRunning = true;
      state.autonomousMissionPreset = preset;
      appendConsoleLine(`[auto] Start ${presetLabel} on ${granted.length} robots.`);
    } else {
      const runningIds = Array.from(state.autonomousActiveIds);
      const stopped = callSupervisorService(
        false,
        runningIds,
        state.autonomousMissionPreset
      );
      if (!stopped) return;
      state.autonomousRunning = false;
      await releaseRobotLeases(runningIds);
      appendConsoleLine("[auto] Stop autonomous pipeline.");
    }
    updateAutoButton();
  });

  updateAutoButton();
}

async function initializeApp() {
  await loadSessionPolicy();
  if (state.session.localEmergency) {
    await initializePhoneQr();
  }
  await loadRobotsConfig();
  initializeRobotState();
  updateConnectionStatus();
  renderAutonomousOptions();
  renderManualSelect();
  updateSummary();
  initHandlers();
  const initialMode = getInitialModeFromQuery();
  if (initialMode) {
    switchMode(initialMode);
  }
  if (sessionAllowsControls()) {
    connectRosBridges();
    startSessionPolling();
    startLeaseHeartbeat();
    if (state.manualRobotId !== null) {
      await claimRobotLease(state.manualRobotId, "manual default");
    }
  }
}

initializeApp();

window.addEventListener("beforeunload", () => {
  if (!sessionAllowsControls()) return;
  robots.forEach((robot) => publishCmdVel(robot, 0.0, 0.0, true));
  releaseRobotLeases([]);
});
