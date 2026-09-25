"use strict";

const draft = {
  1: { table_id: null, loaded_confirmed: false },
  2: { table_id: null, loaded_confirmed: false },
};

const elements = {
  connectionWarning: document.getElementById("connection-warning"),
  setupView: document.getElementById("setup-view"),
  robotTrigger: document.getElementById("robot-trigger"),
  orderSidebar: document.getElementById("order-sidebar"),
  sidebarBackdrop: document.getElementById("sidebar-backdrop"),
  sidebarClose: document.getElementById("sidebar-close"),
  deliveryView: document.getElementById("delivery-view"),
  obstacleScreen: document.getElementById("obstacle-screen"),
  setupMessage: document.getElementById("setup-message"),
  cancelMissionButton: document.getElementById("cancel-mission-button"),
  completionBanner: document.getElementById("completion-banner"),
  completionMessage: document.getElementById("completion-message"),
  startButton: document.getElementById("start-button"),
  deliveryTitle: document.getElementById("delivery-title"),
  deliveryMessage: document.getElementById("delivery-message"),
  deliveryDestination: document.getElementById("delivery-destination"),
  deliveryStops: document.getElementById("delivery-stops"),
  missionSummary: document.getElementById("mission-summary"),
  pickupButton: document.getElementById("pickup-button"),
  resetButton: document.getElementById("reset-button"),
  pickupNext: document.getElementById("pickup-next"),
  errorPanel: document.getElementById("error-panel"),
  errorMessage: document.getElementById("error-message"),
  robotTable1: document.getElementById("robot-table-1"),
  robotTable2: document.getElementById("robot-table-2"),
};

let controllerState = null;
let connected = false;
let startPending = false;
let pickupPendingFor = null;
let lastError = "";
let refreshInFlight = false;
let sidebarOpen = false;
let activeShelf = 1;

function physicalShelfForUi(shelf) {
  return 3 - shelf;
}

function uiShelfMessage(message) {
  return message.replace(/ชั้น ([12])/g, (_match, shelf) =>
    "ชั้น " + physicalShelfForUi(Number(shelf)));
}

function showSetupMessage(message, kind = "") {
  elements.setupMessage.textContent = message;
  elements.setupMessage.dataset.kind = kind;
}

function renderDraft() {
  let selectedCount = 0;
  const setup = isSetupState();

  for (const shelf of [1, 2]) {
    const selection = draft[shelf];
    const card = document.querySelector('[data-order-card][data-shelf="' + shelf + '"]');
    card.dataset.active = String(shelf === activeShelf);
    const isSelected = selection.table_id !== null;
    if (isSelected) {
      selectedCount += 1;
    }

    card.querySelectorAll('[data-action="table"]').forEach((button) => {
      const selected = Number(button.dataset.table) === selection.table_id;
      button.setAttribute("aria-pressed", String(selected));
      button.disabled = !isSetupState();
    });

    const clearButton = card.querySelector('[data-action="clear"]');
    clearButton.disabled = !isSetupState() || !isSelected;

  }

  elements.robotTable1.textContent = draft[1].table_id === null ? "ว่าง" : "โต๊ะ " + draft[1].table_id;
  elements.robotTable2.textContent = draft[2].table_id === null ? "ว่าง" : "โต๊ะ " + draft[2].table_id;

  const canStart = selectedCount > 0;
  elements.startButton.hidden = !setup;
  elements.startButton.disabled = !setup || !connected || !canStart || startPending;
  elements.robotTrigger.disabled = !setup;
  if (!setup) setSidebarOpen(false);
}

function setSidebarOpen(open) {
  sidebarOpen = Boolean(open) && isSetupState();
  elements.setupView.classList.toggle("sidebar-open", sidebarOpen);
  elements.orderSidebar.classList.toggle("is-open", sidebarOpen);
  elements.orderSidebar.setAttribute("aria-hidden", String(!sidebarOpen));
  elements.orderSidebar.toggleAttribute("inert", !sidebarOpen);
  elements.sidebarBackdrop.hidden = !sidebarOpen;
  elements.robotTrigger.setAttribute("aria-expanded", String(sidebarOpen));
}

function isSetupState() {
  return !controllerState || controllerState.state === "IDLE" || controllerState.state === "COMPLETED";
}

function renderStops(snapshot) {
  const orders = Array.isArray(snapshot.orders) ? snapshot.orders : [];
  const index = snapshot.current_order_index;
  elements.deliveryStops.replaceChildren();
  elements.missionSummary.replaceChildren();

  orders.forEach((order, orderIndex) => {
    const done = snapshot.state === "COMPLETED" ||
      snapshot.state === "RETURNING" ||
      (Number.isInteger(index) && orderIndex < index);
    const current = Number.isInteger(index) && orderIndex === index &&
      !["RETURNING", "COMPLETED"].includes(snapshot.state);
    const text = "โต๊ะ " + order.table_id;

    const stop = document.createElement("span");
    stop.className = "delivery-stop";
    stop.dataset.kind = done ? "done" : current ? "current" : "pending";
    stop.textContent = done ? "✓ " + text : text;
    elements.deliveryStops.append(stop);

    const summary = document.createElement("span");
    summary.className = "summary-tag";
    summary.dataset.current = String(current);
    summary.textContent = "ชั้น " + physicalShelfForUi(order.shelf) + " → " + text + (done ? " · ส่งแล้ว" : "");
    elements.missionSummary.append(summary);
  });
}

function renderState(snapshot) {
  controllerState = snapshot;
  const state = snapshot.state || "IDLE";
  const setup = isSetupState();
  const cancellable = Boolean(snapshot.mission_id) &&
    ["PREPARING", "NAVIGATING", "WAITING_PICKUP", "PICKUP_DELAY", "RETURNING", "CANCELLING"].includes(state);

  const sharedDraft = snapshot.draft || {};
  for (const shelf of [1, 2]) {
    const physicalShelf = physicalShelfForUi(shelf);
    const selection = sharedDraft[String(physicalShelf)] || sharedDraft[physicalShelf] || {};
    draft[shelf] = {
      table_id: Number.isInteger(selection.table_id) ? selection.table_id : null,
      loaded_confirmed: selection.loaded_confirmed === true,
    };
  }
  activeShelf = [1, 2].includes(snapshot.active_shelf)
    ? physicalShelfForUi(snapshot.active_shelf)
    : 1;
  if (setup && snapshot.setup_message) {
    showSetupMessage(uiShelfMessage(snapshot.setup_message));
  }

  elements.connectionWarning.hidden = connected;
  elements.setupView.hidden = !setup;
  elements.deliveryView.hidden = setup;
  elements.deliveryView.dataset.state = state;
  elements.obstacleScreen.hidden = snapshot.obstacle_detected !== true;
  elements.missionSummary.hidden = !setup;
  elements.cancelMissionButton.hidden = !cancellable;
  elements.cancelMissionButton.disabled = !connected || state === "CANCELLING";
  elements.cancelMissionButton.textContent = state === "CANCELLING"
    ? "กำลังหยุดหุ่นยนต์..."
    : "ยกเลิกภารกิจ · หยุดหุ่นยนต์";

  elements.completionBanner.hidden = state !== "COMPLETED";
  if (state === "COMPLETED") {
    elements.completionMessage.textContent = snapshot.message || "กลับถึงครัวแล้ว พร้อมรับงานรอบใหม่";
  }

  elements.deliveryTitle.textContent = {
    PREPARING: "กำลังเตรียมภารกิจ",
    NAVIGATING: "กำลังเดินทาง",
    CANCELLING: "กำลังหยุดหุ่นยนต์",
    WAITING_PICKUP: "ถึงจุดหมายแล้ว",
    PICKUP_DELAY: "กำลังรอก่อนเคลื่อนที่",
    RETURNING: "กำลังกลับครัว",
    ERROR: "หุ่นยนต์หยุดทำงาน",
  }[state] || "กำลังทำงาน";
  elements.deliveryMessage.textContent = snapshot.message || "กำลังทำงาน";
  elements.errorPanel.hidden = state !== "ERROR";
  elements.errorMessage.textContent = snapshot.error || snapshot.message || "";
  elements.resetButton.hidden = state !== "ERROR";
  elements.resetButton.disabled = !connected;

  const index = snapshot.current_order_index;
  const currentOrder = Array.isArray(snapshot.orders) && Number.isInteger(index)
    ? snapshot.orders[index]
    : null;
  if (state === "RETURNING") {
    elements.deliveryDestination.textContent = "ส่งอาหารครบแล้ว กำลังกลับครัว";
  } else if (currentOrder) {
    elements.deliveryDestination.textContent =
      "โต๊ะ " + currentOrder.table_id + " · อาหารจากชั้น " + physicalShelfForUi(currentOrder.shelf);
  } else {
    elements.deliveryDestination.textContent = "";
  }

  const waitingForPickup = state === "WAITING_PICKUP" && currentOrder;
  const pickupKey = waitingForPickup
    ? snapshot.mission_id + ":" + snapshot.current_order_index
    : null;
  if (pickupPendingFor && pickupKey !== pickupPendingFor) {
    pickupPendingFor = null;
  }
  elements.pickupButton.hidden = !waitingForPickup;
  elements.pickupButton.disabled = Boolean(pickupPendingFor && pickupKey === pickupPendingFor);
  if (waitingForPickup) {
    let nextIndex = index + 1;
    while (
      nextIndex < snapshot.orders.length &&
      snapshot.orders[nextIndex].table_id === currentOrder.table_id
    ) {
      nextIndex += 1;
    }
    const nextOrder = snapshot.orders[nextIndex];
    elements.pickupNext.textContent = nextOrder
      ? "ต่อไปโต๊ะ " + nextOrder.table_id
      : "ยืนยันเพื่อกลับครัว";
  }

  renderStops(snapshot);
  renderDraft();
}

async function requestJson(path, options = {}) {
  const response = await fetch(path, {
    cache: "no-store",
    ...options,
    headers: {
      Accept: "application/json",
      ...(options.body ? { "Content-Type": "application/json" } : {}),
      ...options.headers,
    },
  });
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || "คำขอไม่สำเร็จ (" + response.status + ")");
  }
  return payload;
}

async function loadTheme() {
  try {
    const config = await requestJson("/api/config");
    document.documentElement.dataset.theme = config.theme === "dark" ? "dark" : "light";
  } catch (_error) {
    // Keep the theme declared in index.html when config is unavailable.
  }
}

async function refreshState() {
  if (refreshInFlight) return;
  refreshInFlight = true;
  try {
    const snapshot = await requestJson("/api/state");
    connected = true;
    lastError = "";
    renderState(snapshot);
  } catch (error) {
    connected = false;
    if (lastError !== error.message) {
      lastError = error.message;
      showSetupMessage("ติดต่อ controller ไม่ได้ กำลังลองเชื่อมต่อใหม่", "error");
    }
    elements.connectionWarning.hidden = false;
    elements.startButton.disabled = true;
    renderDraft();
  } finally {
    refreshInFlight = false;
  }
}

async function startMission() {
  const selected = [1, 2].filter((shelf) => draft[shelf].table_id !== null);
  if (selected.length === 0) {
    showSetupMessage("เลือกโต๊ะอย่างน้อยหนึ่งโต๊ะก่อนเริ่ม", "error");
    return;
  }

  startPending = true;
  renderDraft();
  try {
    await requestJson("/api/pos/action", {
      method: "POST",
      body: JSON.stringify({ action: "start" }),
    });
    setSidebarOpen(false);
    showSetupMessage("รับรายการแล้ว กำลังเริ่มภารกิจ");
    await refreshState();
  } catch (error) {
    showSetupMessage(error.message, "error");
  } finally {
    startPending = false;
    renderDraft();
  }
}

async function cancelMission() {
  elements.cancelMissionButton.disabled = true;
  try {
    await requestJson("/api/mission/cancel", {
      method: "POST",
      body: JSON.stringify({}),
    });
    await refreshState();
  } catch (error) {
    elements.deliveryMessage.textContent = error.message;
    elements.cancelMissionButton.disabled = false;
  }
}

async function assignTable(shelf, tableId) {
  try {
    const physicalShelf = physicalShelfForUi(shelf);
    await requestJson("/api/pos/action", {
      method: "POST",
      body: JSON.stringify({ action: "set_table", shelf: physicalShelf, table_id: tableId }),
    });
    const snapshot = await requestJson("/api/state");
    const selection = snapshot.draft && (snapshot.draft[String(physicalShelf)] || snapshot.draft[physicalShelf]);
    // Older running controllers still require the placement flag; newer ones set it with the table.
    if (selection && selection.table_id === tableId && selection.loaded_confirmed !== true) {
      await requestJson("/api/pos/action", {
        method: "POST",
        body: JSON.stringify({ action: "toggle_loaded", shelf: physicalShelf }),
      });
    }
    await refreshState();
  } catch (error) {
    showSetupMessage(error.message, "error");
  }
}

async function applySetupAction(action, shelf = null, tableId = null) {
  const payload = { action };
  if (shelf !== null) payload.shelf = physicalShelfForUi(shelf);
  if (tableId !== null) payload.table_id = tableId;
  try {
    await requestJson("/api/pos/action", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    await refreshState();
  } catch (error) {
    showSetupMessage(error.message, "error");
  }
}

async function confirmPickup() {
  if (!controllerState || !controllerState.mission_id ||
      !Number.isInteger(controllerState.current_order_index)) {
    return;
  }

  const pickupKey = controllerState.mission_id + ":" + controllerState.current_order_index;
  if (pickupPendingFor === pickupKey) return;
  pickupPendingFor = pickupKey;
  elements.pickupButton.disabled = true;
  try {
    await requestJson("/api/mission/pickup-confirmed", {
      method: "POST",
      body: JSON.stringify({
        mission_id: controllerState.mission_id,
        order_index: controllerState.current_order_index,
      }),
    });
    await refreshState();
  } catch (error) {
    pickupPendingFor = null;
    elements.deliveryMessage.textContent = error.message;
  }
}

async function resetMission() {
  if (!connected || !controllerState || controllerState.state !== "ERROR") return;
  elements.resetButton.disabled = true;
  try {
    await requestJson("/api/mission/reset", { method: "POST", body: JSON.stringify({}) });
    for (const shelf of [1, 2]) {
      draft[shelf] = { table_id: null, loaded_confirmed: false };
    }
    await refreshState();
  } catch (error) {
    elements.deliveryMessage.textContent = error.message;
    elements.resetButton.disabled = false;
  }
}

document.addEventListener("click", (event) => {
  const button = event.target.closest("button");
  if (!button) return;

  if (button.id === "start-button") {
    startMission();
    return;
  }
  if (button.id === "cancel-mission-button") {
    cancelMission();
    return;
  }
  if (button.id === "pickup-button") {
    confirmPickup();
    return;
  }
  if (button.id === "reset-button") {
    resetMission();
    return;
  }
  if (button.id === "robot-trigger") {
    const robotShelf = event.target.closest("[data-robot-shelf]");
    if (robotShelf) {
      activeShelf = physicalShelfForUi(Number(robotShelf.dataset.robotShelf));
      renderDraft();
      applySetupAction("select_shelf", activeShelf);
      setSidebarOpen(true);
    } else {
      setSidebarOpen(!sidebarOpen);
    }
    return;
  }
  if (button.id === "sidebar-close") {
    setSidebarOpen(false);
    return;
  }
  if (!isSetupState()) return;

  const shelf = Number(button.dataset.shelf);
  if (![1, 2].includes(shelf)) return;
  const selection = draft[shelf];

  if (button.dataset.action === "table") {
    const table = Number(button.dataset.table);
    assignTable(shelf, table);
  } else if (button.dataset.action === "clear") {
    applySetupAction("clear_shelf", shelf);
  }
});

elements.sidebarBackdrop.addEventListener("click", () => setSidebarOpen(false));
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && sidebarOpen) setSidebarOpen(false);
});

loadTheme();
refreshState();
window.setInterval(refreshState, 500);
