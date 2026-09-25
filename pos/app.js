"use strict";

const draft = {
  1: { table_id: null, loaded_confirmed: false },
  2: { table_id: null, loaded_confirmed: false },
};

const elements = {
  connectionWarning: document.getElementById("connection-warning"),
  setupView: document.getElementById("setup-view"),
  deliveryView: document.getElementById("delivery-view"),
  setupMessage: document.getElementById("setup-message"),
  completionBanner: document.getElementById("completion-banner"),
  completionMessage: document.getElementById("completion-message"),
  startButton: document.getElementById("start-button"),
  footerStatus: document.getElementById("footer-status"),
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

function showSetupMessage(message, kind = "") {
  elements.setupMessage.textContent = message;
  elements.setupMessage.dataset.kind = kind;
}

function renderDraft() {
  let selectedCount = 0;
  let allLoaded = true;
  const setup = isSetupState();

  for (const shelf of [1, 2]) {
    const selection = draft[shelf];
    const card = document.querySelector('[data-order-card][data-shelf="' + shelf + '"]');
    const isSelected = selection.table_id !== null;
    if (isSelected) {
      selectedCount += 1;
      allLoaded = allLoaded && selection.loaded_confirmed;
    }

    card.querySelectorAll('[data-action="table"]').forEach((button) => {
      const selected = Number(button.dataset.table) === selection.table_id;
      button.setAttribute("aria-pressed", String(selected));
      button.disabled = !isSetupState();
    });

    const clearButton = card.querySelector('[data-action="clear"]');
    clearButton.disabled = !isSetupState() || !isSelected;

    const loadedButton = card.querySelector('[data-action="loaded"]');
    loadedButton.disabled = !isSetupState() || !isSelected;
    loadedButton.setAttribute("aria-pressed", String(selection.loaded_confirmed));
    loadedButton.lastElementChild.textContent = selection.loaded_confirmed
      ? "ยืนยันแล้ว · วางอาหารบนชั้นนี้"
      : "ยืนยันว่าใส่อาหารแล้ว";
  }

  elements.robotTable1.textContent = draft[1].table_id === null ? "ว่าง" : "โต๊ะ " + draft[1].table_id;
  elements.robotTable2.textContent = draft[2].table_id === null ? "ว่าง" : "โต๊ะ " + draft[2].table_id;

  const canStart = selectedCount > 0 && allLoaded;
  elements.startButton.hidden = !setup;
  elements.startButton.disabled = !setup || !connected || !canStart || startPending;
  if (!setup) {
    const status = controllerState?.state;
    elements.footerStatus.textContent = {
      WAITING_PICKUP: "รอผู้ใช้ยืนยันว่ารับอาหารแล้ว",
      RETURNING: "ส่งครบแล้ว กำลังกลับครัว",
      ERROR: "หุ่นยนต์หยุดแล้ว กรุณาตรวจสอบ",
    }[status] || "หุ่นยนต์กำลังดำเนินภารกิจ";
    return;
  }
  elements.footerStatus.textContent = !connected
    ? "กำลังรอการเชื่อมต่อ controller"
    : selectedCount === 0
      ? "เลือกโต๊ะอย่างน้อยหนึ่งชั้น"
      : !allLoaded
        ? "ยืนยันการวางอาหารให้ครบทุกชั้นที่เลือก"
        : "ตรวจสอบรายการแล้ว พร้อมเริ่มจัดส่ง";
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
    summary.textContent = "ชั้น " + order.shelf + " → " + text + (done ? " · ส่งแล้ว" : "");
    elements.missionSummary.append(summary);
  });
}

function renderState(snapshot) {
  controllerState = snapshot;
  const state = snapshot.state || "IDLE";
  const setup = isSetupState();

  elements.connectionWarning.hidden = connected;
  elements.setupView.hidden = !setup;
  elements.deliveryView.hidden = setup;
  elements.deliveryView.dataset.state = state;

  elements.completionBanner.hidden = state !== "COMPLETED";
  if (state === "COMPLETED") {
    elements.completionMessage.textContent = snapshot.message || "กลับถึงครัวแล้ว พร้อมรับงานรอบใหม่";
  }

  elements.deliveryTitle.textContent = {
    PREPARING: "กำลังเตรียมภารกิจ",
    NAVIGATING: "กำลังเดินทาง",
    WAITING_PICKUP: "ถึงจุดหมายแล้ว",
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
      "โต๊ะ " + currentOrder.table_id + " · อาหารจากชั้น " + currentOrder.shelf;
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
    const nextOrder = snapshot.orders[index + 1];
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
  const orders = [1, 2]
    .filter((shelf) => draft[shelf].table_id !== null)
    .map((shelf) => ({
      shelf,
      table_id: draft[shelf].table_id,
      loaded_confirmed: draft[shelf].loaded_confirmed,
    }));

  if (orders.length === 0 || orders.some((order) => !order.loaded_confirmed)) {
    showSetupMessage("เลือกโต๊ะและยืนยันการวางอาหารให้ครบก่อนเริ่ม", "error");
    return;
  }

  startPending = true;
  renderDraft();
  try {
    await requestJson("/api/mission/start", {
      method: "POST",
      body: JSON.stringify({ orders }),
    });
    for (const shelf of [1, 2]) {
      draft[shelf] = { table_id: null, loaded_confirmed: false };
    }
    showSetupMessage("รับรายการแล้ว กำลังเริ่มภารกิจ");
    await refreshState();
  } catch (error) {
    showSetupMessage(error.message, "error");
  } finally {
    startPending = false;
    renderDraft();
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
  if (button.id === "pickup-button") {
    confirmPickup();
    return;
  }
  if (button.id === "reset-button") {
    resetMission();
    return;
  }
  if (!isSetupState()) return;

  const shelf = Number(button.dataset.shelf);
  if (![1, 2].includes(shelf)) return;
  const selection = draft[shelf];

  if (button.dataset.action === "table") {
    const table = Number(button.dataset.table);
    selection.table_id = selection.table_id === table ? null : table;
    selection.loaded_confirmed = false;
    showSetupMessage(selection.table_id === null
      ? "เลือกอย่างน้อยหนึ่งชั้นเพื่อเริ่มงาน"
      : "เลือกโต๊ะ " + table + " สำหรับชั้น " + shelf + " แล้ว");
  } else if (button.dataset.action === "loaded" && selection.table_id !== null) {
    selection.loaded_confirmed = !selection.loaded_confirmed;
    showSetupMessage(selection.loaded_confirmed
      ? "ยืนยันแล้วว่าวางอาหารบนชั้น " + shelf
      : "ยกเลิกการยืนยันชั้น " + shelf);
  } else if (button.dataset.action === "clear") {
    draft[shelf] = { table_id: null, loaded_confirmed: false };
    showSetupMessage("ล้างรายการชั้น " + shelf + " แล้ว");
  }

  renderDraft();
});

refreshState();
window.setInterval(refreshState, 500);
