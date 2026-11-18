"use strict";

const SERVICE = "9e238080-4e25-4a8a-b0b9-845e2841a6f0";
const RAW_NOTIFY = "9e238081-4e25-4a8a-b0b9-845e2841a6f0";
const CONTROL = "9e238082-4e25-4a8a-b0b9-845e2841a6f0";
const SEQ_MASK = 0x3fffffff;
const RATE_HZ = 100;
const PLOT_SPAN = 2000;
const RAW_MAX = 0x0fffffff;
const WRENCH_BASE_RANGES = [20, 20, 20, 5, 5, 5];
const byId = id => document.getElementById(id);
const connect = byId("connect");
const connectUsb = byId("connect-usb");
const disconnect = byId("disconnect");
const tare = byId("tare");
const dataButton = byId("data-mode");
const downloadRaw = byId("download-raw");
const deviceFilter = byId("device-filter");
const canvas = byId("plot");
const channel = byId("channel");
const names = Array.from({ length: 8 }, (_, i) => "L" + (i + 1));
const wrenchNames = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
const forceColors = ["#74e8d9", "#5fbfff", "#f4c86a", "#d8a1f6", "#f49bb2", "#a7d989"];
const forceCanvases = wrenchNames.map((axis, i) => {
  const panel = document.createElement("section");
  panel.className = "plot";
  const title = document.createElement("h3");
  title.textContent = axis + (i < 3 ? "（N）" : "（N·m）");
  title.style.color = forceColors[i];
  const plot = document.createElement("canvas");
  plot.setAttribute("role", "img");
  plot.setAttribute("aria-label", title.textContent + "，最近 2,000 筆");
  panel.append(title, plot);
  byId("force-plots").append(panel);
  return plot;
});
const tempFields = tiles("temperatures", ["TMP117_1", "TMP117_2", "TMP117_3", "TMP117_4", "TMP275"]);
const canConnect = window.isSecureContext && Boolean(navigator.bluetooth);
const canUseUsb = window.isSecureContext && Boolean(navigator.serial);
const number = new Intl.NumberFormat("en-US");
const decimal = new Intl.NumberFormat("en-US", { minimumFractionDigits: 3, maximumFractionDigits: 3 });
let device, source, control, pending, latest, diagnostic, temperature;
let usb;
let mode = "";
let dataMode = "wrench";
let changing = false;
let tareTimer;
let fields;
let connected = false;
let connecting = false;
let history = [];
let missing = 0;
let received = 0;
let lastRateCount = 0;
let previousDiagnostic;
let rateTime = performance.now();
let dataDirty = true;
let lastDataTime = -Infinity;
let lastPaint = 0;

function tiles(id, labels) {
  return labels.map(label => {
    const tile = document.createElement("div");
    tile.className = "tile";
    const term = document.createElement("dt");
    term.textContent = label;
    const value = document.createElement("dd");
    value.textContent = "—";
    tile.append(term, value);
    byId(id).append(tile);
    return value;
  });
}

function setView(next) {
  dataMode = next;
  const force = dataMode === "wrench";
  const labels = force ? wrenchNames : names;
  channel.replaceChildren(...labels.map((name, i) => new Option(name, i)));
  byId("channels").replaceChildren();
  fields = tiles("channels", labels);
  byId("raw-title").textContent = force ? "力量與力矩" : "線圈 RAW 數值";
  byId("units").textContent = force ? "Fx、Fy、Fz（N） · Tx、Ty、Tz（N·m）" : "LDC161x · 28-bit counts";
  document.querySelectorAll("[data-ble]").forEach(element => { element.hidden = mode !== "ble"; });
  byId("temp-panel").hidden = mode === "usb" && force;
  byId("raw-channel").hidden = force;
  byId("raw-plot").hidden = force;
  downloadRaw.hidden = force;
  byId("force-plots").hidden = !force;
  dataButton.textContent = force ? "切換到 RAW" : "切換到力量";
  byId("force-range").hidden = !force;
  dataDirty = true;
}

function setButtons() {
  connect.disabled = !canConnect || connecting || connected;
  connectUsb.disabled = !canUseUsb || connecting || connected;
  disconnect.disabled = connecting || !connected;
  const canCommand = mode === "usb" || Boolean(control);
  tare.disabled = connecting || !connected || changing || !canCommand || tareTimer !== undefined;
  dataButton.disabled = connecting || !connected || changing || !canCommand || tareTimer !== undefined;
  deviceFilter.disabled = connecting || !connected || changing || !canCommand || tareTimer !== undefined;
}

setView("wrench");
setButtons();

function status(message, level = "") {
  byId("status").textContent = message;
  byId("status").dataset.level = level;
}

function clearTrace() {
  pending = latest = undefined;
  history = [];
  downloadRaw.disabled = true;
  missing = received = lastRateCount = 0;
  rateTime = performance.now();
  lastDataTime = -Infinity;
  dataDirty = true;
  byId("rate").textContent = "—";
  byId("health").textContent = "";
}

function reset() {
  deviceFilter.value = "";
  byId("tare-status").textContent = "無載靜止時可手動歸零。";
  diagnostic = temperature = previousDiagnostic = undefined;
  clearTrace();
}

function closeConnection(message, level) {
  if (tareTimer !== undefined) finishTare(false, "連線中斷，未收到歸零結果。");
  connected = false;
  if (source) source.removeEventListener("characteristicvaluechanged", onNotification);
  if (device) {
    device.removeEventListener("gattserverdisconnected", onDisconnect);
    if (device.gatt.connected) device.gatt.disconnect();
  }
  source = control = device = pending = undefined;
  setButtons();
  byId("rate").textContent = "—";
  byId("health").textContent = latest ? "畫面保留最後的資料" : "";
  byId("health").dataset.level = "";
  status(message, level);
}

function onDisconnect() {
  if (!connecting) closeConnection("BLE 連線已中斷。按「藍牙連線」重新連線。", "warn");
}

connect.addEventListener("click", async () => {
  connecting = true;
  setButtons();
  reset();
  mode = "ble";
  setView("wrench");
  status("正在搜尋 UTL80A…");
  try {
    device = await navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE] }] });
    device.addEventListener("gattserverdisconnected", onDisconnect);
    status("正在連線並啟用通知…");
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE);
    source = await service.getCharacteristic(RAW_NOTIFY);
    try {
      control = await service.getCharacteristic(CONTROL);
    } catch (error) {
      if (error.name !== "NotFoundError") throw error;
      setView("raw");
    }
    source.addEventListener("characteristicvaluechanged", onNotification);
    await source.startNotifications();
    if (control) await sendBle(1);
    if (!device.gatt.connected) throw new Error("裝置已中斷連線");
    connected = true;
    rateTime = performance.now();
    status(control ? "已連線 · " + (device.name || "UTL80A") : "已連線 · 舊韌體僅提供 RAW", "ok");
  } catch (error) {
    const message = error.name === "NotFoundError"
      ? "未選取 UTL80A。確認板子已開機，再按「藍牙連線」。"
      : "連線失敗 · " + error.message;
    closeConnection(message, "warn");
  } finally {
    connecting = false;
    setButtons();
  }
});

function sendBle(command) {
  return control.writeValueWithResponse(Uint8Array.of(command));
}

async function sendUsb(session, command) {
  const writer = session.port.writable.getWriter();
  try {
    await writer.write(new TextEncoder().encode(command + "\n"));
  } finally {
    writer.releaseLock();
  }
}

// A port may disconnect while a read or command is pending.
async function closeUsb(session, message, level = "") {
  if (session !== usb || session.closing) return;
  session.closing = true;
  connecting = true;
  setButtons();
  try {
    if (session.reader) await session.reader.cancel().catch(() => {});
    await session.reading?.catch(() => {});
    await session.port.close().catch(() => {});
  } finally {
    usb = undefined;
    connecting = false;
    closeConnection(message, level);
  }
}

function readWrench(line) {
  const parts = line.trim().split(/\s+/);
  if (parts.length !== wrenchNames.length) return null;
  const values = [];
  for (let i = 0; i < parts.length; i++) {
    const prefix = wrenchNames[i] + ":";
    if (!parts[i].startsWith(prefix)) return null;
    const value = parts[i].slice(prefix.length);
    if (value !== "nan" && !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$/.test(value)) return null;
    const number = value === "nan" ? null : Number(value);
    if (number !== null && !Number.isFinite(number)) return null;
    values.push(number);
  }
  return values;
}

function readUsbRaw(line) {
  const parts = line.trim().split(/\s+/);
  if (parts.length < names.length || !parts[0].startsWith("L1:")) return null;
  const values = [];
  for (const [i, name] of names.entries()) {
    const match = parts[i].match(/^L(\d+):(-?\d+)$/);
    if (!match || "L" + match[1] !== name) return null;
    const value = Number(match[2]);
    values.push(value > 0 && value < RAW_MAX ? value : null);
  }
  const tempLabels = ["TMP117_1", "TMP117_2", "TMP117_3", "TMP117_4", "TMP275"];
  const temperatures = tempLabels.map((name, i) => {
    const prefix = name + ":";
    const part = parts[names.length+i];
    if (!part?.startsWith(prefix)) return null;
    const value = Number(part.slice(prefix.length));
    return Number.isFinite(value) ? value : null;
  });
  temperature = temperatures;
  return values;
}

async function readUsb(session) {
  const reader = session.port.readable.getReader();
  const decoder = new TextDecoder();
  session.reader = reader;
  let buffer = "";
  try {
    while (!session.closing) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split("\n");
      buffer = lines.pop();
      if (buffer.length > 4096) buffer = "";
      for (const line of lines) {
        const values = dataMode === "wrench" ? readWrench(line) : readUsbRaw(line);
        if (values) {
          if (session.waitingForFirstFrame) {
            session.waitingForFirstFrame = false;
            status("USB 已連線。變更裝夾後請重新歸零。", "ok");
          }
          addFrame(received & SEQ_MASK, values);
        } else if (line.includes("[TARE] OK")) {
          if (tareTimer !== undefined) finishTare(true);
        } else if (line.includes("[TARE] Failed")) {
          if (tareTimer !== undefined) finishTare(false, line.includes("unstable") ?
            "讀值未穩定，已保留上一個零點。請在無載靜止後再試。" : undefined);
        } else if (dataMode === "wrench" && session.waitingForFirstFrame && !session.retried && line.startsWith("L1:")) {
          session.retried = true;
          // If opening the port restarted the board, send again once RAW startup is visible.
          session.initialCommand.then(() => {
            if (!session.closing && session.waitingForFirstFrame) return sendUsb(session, "w");
          }).catch(() => {});
        }
      }
    }
  } finally {
    session.reader = undefined;
    reader.releaseLock();
  }
}

connectUsb.addEventListener("click", async () => {
  connecting = true;
  setButtons();
  reset();
  mode = "usb";
  setView("wrench");
  status("請選擇 UTL80A 的 COM 埠。");
  let session;
  try {
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    session = { port, waitingForFirstFrame: true };
    usb = session;
    connected = true;
    rateTime = performance.now();
    status("USB 已開啟，正在等待力量資料。");
    session.initialCommand = sendUsb(session, "w");
    session.reading = readUsb(session);
    session.reading.then(
      () => closeUsb(session, "USB 連線已中斷。", "warn"),
      error => closeUsb(session, "USB 讀取失敗 · " + error.message, "warn"),
    );
    await session.initialCommand;
  } catch (error) {
    const message = error.name === "NotFoundError" ? "未選取序列埠。" : "USB 連線失敗 · " + error.message;
    if (session) await closeUsb(session, message, "warn");
    else closeConnection(message, "warn");
  } finally {
    if (!usb?.closing) connecting = false;
    setButtons();
  }
});

dataButton.addEventListener("click", async () => {
  if (!connected || changing) return;
  const previous = dataMode;
  const next = previous === "wrench" ? "raw" : "wrench";
  changing = true;
  setButtons();
  clearTrace();
  setView(next);
  try {
    if (mode === "ble") await sendBle(next === "raw" ? 0 : 1);
    else await sendUsb(usb, next === "raw" ? "p" : "w");
  } catch (error) {
    clearTrace();
    setView(previous);
    status("切換失敗 · " + error.message, "warn");
  } finally {
    changing = false;
    setButtons();
  }
});

function finishTare(ok, message) {
  clearTimeout(tareTimer);
  tareTimer = undefined;
  if (ok && dataMode === "wrench") clearTrace();
  byId("tare-status").textContent = message ?? (ok ? "歸零完成。" : "歸零失敗。請確認八路讀值後重試。");
  setButtons();
}

tare.addEventListener("click", async () => {
  if (!connected || changing || tareTimer !== undefined) return;
  byId("tare-status").textContent = "正在歸零，請保持無載及靜止。";
  tareTimer = setTimeout(() => finishTare(false, "未收到歸零回覆。請確認已燒錄新版韌體，再重試。"), 10500);
  setButtons();
  try {
    if (mode === "ble") await sendBle(2);
    else await sendUsb(usb, "tare");
  } catch (error) {
    finishTare(false, "無法送出歸零指令 · " + error.message);
  }
});

deviceFilter.addEventListener("change", async () => {
  if (!connected || changing || !deviceFilter.value) return;
  const profiles = {
    tcn: [7, "filter tcn", "TCN"],
    fzDirect: [9, "filter fz", "Fz 砝碼測試"],
    tcnLinear: [8, "filter tcn-linear", "本版線性基準"],
    impact: [5, "filter impact", "舊版線性直通"],
    fast: [6, "filter 4", "快速平均 4 筆"],
    mean: [4, "filter 24", "穩定平均 24 筆"],
    kalman: [3, "filter kalman", "卡爾曼"],
  };
  const [ble, serial, label] = profiles[deviceFilter.value];
  changing = true;
  setButtons();
  try {
    if (mode === "ble") await sendBle(ble);
    else await sendUsb(usb, serial);
    byId("tare-status").textContent = "已送出換算設定 · " + label;
  } catch (error) {
    deviceFilter.value = "";
    byId("tare-status").textContent = "換算設定失敗 · " + error.message;
  } finally {
    changing = false;
    setButtons();
  }
});

disconnect.addEventListener("click", () => {
  if (usb) closeUsb(usb, "已中斷 USB 連線。");
  else closeConnection("已中斷連線。");
});
if (!canConnect && !canUseUsb) {
  status("請用 Chrome 或 Edge 開啟 localhost 或 HTTPS 網址。", "warn");
}

function readRaw(view, count) {
  return Array.from({ length: count }, (_, i) => {
    const value = view.getUint32(4 + i * 4, true);
    return value > 0 && value < RAW_MAX ? value : null;
  });
}

function onNotification(event) {
  const view = event.target.value;
  if (view.byteLength < 4) return;
  const word = view.getUint32(0, true);
  const seq = word & SEQ_MASK;
  const kind = word >>> 30;
  if (view.byteLength === 14 && word <= 31) {
    temperature = Array.from({ length: 5 }, (_, i) =>
      word & (1 << i) ? view.getInt16(4 + i * 2, true) / 256 : null);
    dataDirty = true;
  } else if (view.byteLength === 5 && kind === 2) {
    const result = view.getUint8(4);
    if (tareTimer !== undefined && result >= 1 && result <= 3) {
      finishTare(result === 1, result === 3 ? "讀值未穩定，已保留上一個零點。請在無載靜止後再試。" : undefined);
    }
  } else if ((view.byteLength === 16 || view.byteLength === 20) && kind === 2) {
    diagnostic = {
      skipped: view.getUint32(4, true),
      rejected: view.getUint32(8, true),
      mtu: view.getUint16(12, true),
      interval: view.getUint16(14, true) * 1.25,
      rawHz: view.byteLength === 20 ? view.getUint16(16, true) : null,
      wrenchHz: view.byteLength === 20 ? view.getUint16(18, true) : null,
    };
    dataDirty = true;
  } else if (view.byteLength === 16 && kind === 3 && dataMode === "wrench") {
    const values = wrenchNames.map((_, i) => {
      const counts = view.getInt16(4 + i * 2, true);
      return counts === -32768 ? null : counts / (i < 3 ? 20 : 1000);
    });
    addFrame(seq, values);
  } else if (view.byteLength === 36 && kind === 0 && dataMode === "raw") {
    pending = undefined;
    addFrame(seq, readRaw(view, 8));
  } else if (view.byteLength === 20 && kind === 0 && dataMode === "raw") {
    pending = { seq, values: readRaw(view, 4) };
  } else if (view.byteLength === 20 && kind === 1 && dataMode === "raw" && pending?.seq === seq) {
    addFrame(seq, pending.values.concat(readRaw(view, 4)));
    pending = undefined;
  }
}

function addFrame(seq, values) {
  const step = latest ? (seq - latest.seq) & SEQ_MASK : 1;
  if (step === 0 || step > SEQ_MASK / 2) return;
  missing += step - 1;
  const receivedAt = performance.now();
  latest = { seq, values, gap: step > 1, receivedAt };
  history.push(latest);
  if (history.length > PLOT_SPAN) history.shift();
  received++;
  lastDataTime = receivedAt;
  if (dataMode === "raw") downloadRaw.disabled = false;
  dataDirty = true;
}

downloadRaw.addEventListener("click", () => {
  if (dataMode !== "raw" || history.length === 0) return;
  const start = history[0].receivedAt;
  const csv = [
    ["Elapsed_Time", "Sequence", "Gap", ...names].join(","),
    ...history.map(frame => [
      ((frame.receivedAt - start) / 1000).toFixed(6),
      frame.seq,
      Number(frame.gap),
      ...frame.values,
    ].join(",")),
  ].join("\r\n");
  const url = URL.createObjectURL(new Blob([csv + "\r\n"], { type: "text/csv;charset=utf-8" }));
  const link = document.createElement("a");
  link.href = url;
  link.download = "UTL80A_RAW_" + new Date().toISOString().replace(/[:.]/g, "-") + ".csv";
  document.body.append(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

function drawPlot(plot, frames, column, color, force) {
  const ctx = plot.getContext("2d");
  const { width, height } = plot.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const pixelsWide = Math.round(width * ratio);
  const pixelsHigh = Math.round(height * ratio);
  if (plot.width !== pixelsWide || plot.height !== pixelsHigh) {
    plot.width = pixelsWide;
    plot.height = pixelsHigh;
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  if (width < 120 || (!force && !latest)) return;
  const points = frames.map(frame => frame.values[column]);
  const values = points.filter(Number.isFinite);
  ctx.font = "12px monospace";
  ctx.fillStyle = "#a7bacb";
  if (!force && values.length === 0) {
    ctx.fillText("此通道目前沒有有效讀值", 14, height / 2);
    return;
  }
  let bottom, top;
  if (force) {
    top = WRENCH_BASE_RANGES[column];
    const peak = Math.max(0, ...values.map(Math.abs));
    // Use steps and keep the range until the peak leaves the visible history.
    while (top < peak) top *= 2;
    bottom = -top;
  } else {
    const low = Math.min(...values);
    const high = Math.max(...values);
    bottom = low === high ? low - 1 : low;
    top = low === high ? high + 1 : high;
  }
  const left = force ? 58 : 92;
  const right = width - 12;
  for (let i = 0; i < 3; i++) {
    const y = 18 + i * (height - 36) / 2;
    const mark = top - i * (top - bottom) / 2;
    ctx.fillText(force ? decimal.format(mark) : number.format(Math.round(mark)), 3, y + 4);
    ctx.strokeStyle = force && i === 1 ? "#587286" : "#2b4053";
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();
  }
  if (values.length === 0) {
    ctx.fillText("等待有效讀值", left + 8, height / 2 - 8);
    return;
  }
  ctx.save();
  ctx.beginPath();
  ctx.rect(left, 18, right - left, height - 36);
  ctx.clip();
  ctx.beginPath();
  ctx.strokeStyle = ctx.fillStyle = color;
  ctx.lineWidth = 1.6;
  let contiguous = false;
  for (const [i, value] of points.entries()) {
    if (!Number.isFinite(value)) {
      contiguous = false;
      continue;
    }
    const age = points.length - 1 - i;
    const x = right - age / (PLOT_SPAN - 1) * (right - left);
    const y = 18 + (top - value) / (top - bottom) * (height - 36);
    if (contiguous && !frames[i].gap) ctx.lineTo(x, y);
    else {
      ctx.moveTo(x, y);
      ctx.fillRect(x - 1, y - 1, 2, 2);
    }
    contiguous = true;
  }
  ctx.stroke();
  ctx.restore();
}

function paintPlot() {
  const frames = history;
  if (dataMode === "wrench") {
    forceCanvases.forEach((plot, i) => drawPlot(plot, frames, i, forceColors[i], true));
  } else {
    drawPlot(canvas, frames, Number(channel.value), forceColors[0], false);
  }
}

function paint() {
  fields.forEach((field, i) => {
    const value = latest?.values[i];
    field.textContent = value == null ? "—" : (dataMode === "wrench" ? decimal : number).format(value);
  });
  tempFields.forEach((field, i) => {
    const value = temperature?.[i];
    field.textContent = value == null ? "—" : value.toFixed(3) + " °C";
  });
  byId("sequence").textContent = latest ? number.format(latest.seq) : "—";
  byId("gaps").textContent = latest ? number.format(missing) : "—";
  byId("skipped").textContent = diagnostic ? number.format(diagnostic.skipped) : "—";
  byId("rejected").textContent = diagnostic ? number.format(diagnostic.rejected) : "—";
  byId("mtu").textContent = diagnostic ? diagnostic.mtu + " B" : "—";
  byId("interval").textContent = diagnostic ? diagnostic.interval + " ms" : "—";
  byId("raw-hz").textContent = diagnostic?.rawHz == null ? "—" : diagnostic.rawHz + " Hz";
  byId("wrench-hz").textContent = diagnostic?.wrenchHz == null ? "—" : diagnostic.wrenchHz + " Hz";
  byId("sample-count").textContent = number.format(Math.min(history.length, PLOT_SPAN));
  paintPlot();
}

function tick(now) {
  if (connected && now - rateTime >= 2000) {
    const rate = (received - lastRateCount) * 1000 / (now - rateTime);
    byId("rate").textContent = rate.toFixed(1);
    const health = byId("health");
    const bleBackpressure = diagnostic && diagnostic.rejected > (previousDiagnostic?.rejected ?? 0);
    const mcuLate = diagnostic && diagnostic.skipped > (previousDiagnostic?.skipped ?? 0);
    let hint = "";
    if (bleBackpressure) hint = "BLE 傳送失敗增加，目前鏈路或堆疊未能送完";
    else if (mcuLate) hint = "MCU 排程漏拍增加";
    else if (now - lastDataTime >= 2000) hint = "尚未收到所選模式的資料。請確認韌體版本與連線。";
    else if (dataMode === "wrench" && latest?.values.every(value => value === null)) hint = "力量目前無有效讀值。請在無載時歸零。";
    else if (mode === "ble" && rate < RATE_HZ - 5) hint = "實收速率低於 100 Hz，請查看新鮮八路及序號缺口";
    health.textContent = hint;
    health.dataset.level = hint ? "warn" : "";
    previousDiagnostic = diagnostic;
    lastRateCount = received;
    rateTime = now;
  }
  if (dataDirty && now - lastPaint >= 16) {
    paint();
    dataDirty = false;
    lastPaint = now;
  }
  requestAnimationFrame(tick);
}

channel.addEventListener("change", paintPlot);
window.addEventListener("resize", paintPlot);
requestAnimationFrame(tick);
