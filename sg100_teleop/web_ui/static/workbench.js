"use strict";
const $ = (id) => document.getElementById(id);
const token = document.querySelector('meta[name="workbench-token"]').content;
const modes = ["free", "hand", "claw", "script", "pinch", "two_finger"];
let state = null, busy = false, configured = false, replayFrame = 0, playing = false, previousQuality = "", previousHistory = "";

function toast(message) {
  $("toast").textContent = message;
  $("toast").hidden = false;
  clearTimeout(window.toastTimer);
  window.toastTimer = setTimeout(() => { $("toast").hidden = true; }, 8000);
}

async function post(action, arguments_ = {}) {
  const response = await fetch("/api/action", {method:"POST", headers:{"Content-Type":"application/json", "X-Workbench-Token":token}, body:JSON.stringify({action,...arguments_})});
  const result = await response.json();
  if (!response.ok) {
    if (result.status) render(result.status);
    throw new Error(result.error || "操作失败，请查看诊断记录");
  }
  render(result);
  return result;
}

async function run(action, arguments_ = {}) {
  if (busy && action !== "disarm") return;
  const ordinary = action !== "disarm";
  if (ordinary) busy = true;
  if (state) render(state);
  try { await post(action, arguments_); }
  catch (error) { toast(error.message); }
  finally { if (ordinary) busy = false; if (state) render(state); }
}

function timeText(seconds) {
  return String(Math.floor(seconds / 60)).padStart(2,"0") + ":" + String(Math.floor(seconds % 60)).padStart(2,"0");
}

function setEnabled(id, enabled) { $(id).disabled = !enabled; }

function replay(index) {
  if (!state || state.recording_state !== "PENDING_CONFIRMATION") return;
  replayFrame = Math.min(Math.max(0,index), Math.max(0,state.frames - 1));
  $("scrub").value = replayFrame;
  $("head-replay").src = `/api/media/pending/head/${replayFrame}.jpg`;
  $("wrist-replay").src = `/api/media/pending/right_wrist/${replayFrame}.jpg`;
  $("play-time").textContent = `${replayFrame + 1} / ${state.frames}`;
}

function qualityView(s) {
  const signature = JSON.stringify(s.quality);
  if (signature === previousQuality) return;
  previousQuality = signature;
  const report = s.quality;
  if (!report) { playing = false; replayFrame = 0; $("play").textContent = "播放"; $("dagger").checked = false; return; }
  $("quality-badge").textContent = ({PASS:"检查通过",WARN:"需要检查",FAIL:"检查失败"})[report.quality];
  $("quality-badge").className = "badge" + (report.quality === "FAIL" ? " fail" : "");
  $("quality-metrics").replaceChildren();
  for (const [label,key] of [["头部有效帧","head_valid_ratio"],["腕部有效帧","wrist_valid_ratio"],["动作有效率","robot_qcmd_17d_valid_ratio"]]) {
    const item = document.createElement("span"), value = document.createElement("b");
    item.textContent = label;
    value.textContent = report.metrics[key] == null ? "—" : (report.metrics[key] * 100).toFixed(1) + "%";
    item.append(value); $("quality-metrics").append(item);
  }
  $("quality-details").replaceChildren();
  for (const message of [...report.errors,...report.warnings]) {
    const li = document.createElement("li"); li.textContent = message; $("quality-details").append(li);
  }
  $("scrub").max = Math.max(0,s.frames - 1);
  replay(0);
}

function render(s) {
  state = s;
  const c = s.control || {}, active = s.recording_state !== "STOPPED";
  const controlReady = active && c.phase === "READY" && c.fresh && s.control_exit_code == null;
  const ready = s.recording_state === "READY", recording = s.recording_state === "RECORDING", pending = s.recording_state === "PENDING_CONFIRMATION";
  const armed = controlReady && c.arm_enabled && c.hand_enabled;
  if (!configured) {
    for (const [key,value] of Object.entries(s.config)) {
      const element = $("config-form").elements.namedItem(key);
      if (element) { if (element.type === "checkbox") element.checked = value; else element.value = value; }
    }
    $("instruction").value = s.instruction || s.config.instruction;
    configured = true;
  }
  $("environment").textContent = s.demo ? "离线演示 · 未连接真机" : "本机工作站";
  $("saved-count").textContent = String(s.saved_count ?? s.saved.length).padStart(2,"0");
  $("recording-state").textContent = ({STOPPED:"未开始",READY:"等待录制",RECORDING:"● 正在录制",PENDING_CONFIRMATION:"待确认"})[s.recording_state];
  $("recording-state").className = "badge " + (recording ? "recording" : "neutral");
  $("message").textContent = s.message;
  $("device-problems").replaceChildren();
  $("device-problems").hidden = !(s.problems || []).length;
  for(const message of s.problems || []){const li=document.createElement("li");li.textContent=message;$("device-problems").append(li);}
  $("episode").textContent = s.episode_index == null ? "—" : "EP " + String(s.episode_index).padStart(6,"0");
  $("duration").textContent = timeText(s.duration_s);
  $("frames").textContent = s.frames.toLocaleString();
  $("writer").textContent = s.writer_errors || s.writer_drops ? "有异常" : recording ? "正常" : "等待";
  $("writer").title = `队列 ${s.writer_backlog} / 错误 ${s.writer_errors} / 丢帧 ${s.writer_drops}`;
  $("instruction").disabled = recording || pending || busy;
  $("task-template").disabled = recording || pending || busy;
  if (s.instruction) $("instruction").value = s.instruction;
  for (const input of $("config-form").elements) input.disabled = active || busy;
  setEnabled("start", !active && !busy);
  setEnabled("close", active && ready && !busy);
  setEnabled("calibrate", controlReady && ready && !busy);
  setEnabled("arm", controlReady && c.avp_anchored && c.tracking_valid && !armed && !pending && !busy);
  setEnabled("disarm", active && controlReady); // available during finish/flush
  setEnabled("begin", ready && armed && s.telemetry_alive && Object.values(s.cameras).length === 2 && Object.values(s.cameras).every(v=>v.ready) && !busy);
  setEnabled("finish", recording && !busy);
  setEnabled("restart", recording && controlReady && !c.arm_enabled && !c.hand_enabled && !busy);
  setEnabled("off", recording && !busy);
  setEnabled("reference", ready && s.cameras.head?.ready && !busy);
  $("reference-toggle").disabled = !s.reference_updated_ns;
  $("head-reference").hidden = !s.reference_updated_ns || !$("reference-toggle").checked;
  if(s.reference_updated_ns && $("head-reference").dataset.version !== String(s.reference_updated_ns)){ $("head-reference").dataset.version=String(s.reference_updated_ns); $("head-reference").src="/api/reference.jpg?v="+s.reference_updated_ns; }
  setEnabled("save", pending && s.quality?.quality !== "FAIL" && !busy);
  for (const button of document.querySelectorAll("[data-decision]")) if (button.id !== "save") button.disabled = !pending || busy;
  for (const button of document.querySelectorAll("[data-mode]")) {
    button.classList.toggle("selected", c.mode === button.dataset.mode);
    button.setAttribute("aria-pressed", c.mode === button.dataset.mode ? "true" : "false");
    button.disabled = !controlReady || busy;
  }
  $("script-controls").hidden = c.mode !== "script";
  for (const button of document.querySelectorAll("[data-motion]")) button.disabled = !controlReady || busy;
  $("control-dot").classList.toggle("ok", controlReady);
  $("control-state").textContent = !active ? "设备未准备" : !controlReady ? (s.control_exit_code != null ? "控制进程已退出" : (c.error || "正在连接 / 初始化")) : armed ? "遥操已启用" : c.avp_anchored ? "锚点已建立 · 遥操已停止" : "等待建立锚点";
  $("review").hidden = !pending;
  $("recovery-tag").hidden = !s.failure_recovery;
  $("live-indicator").textContent = Object.values(s.cameras).length && Object.values(s.cameras).every(v=>v.ready) ? "● LIVE RGB" : "等待有效相机画面";
  for (const role of ["head","right_wrist"]) {
    const camera = s.cameras[role], image = $(role + "-live");
    const status = $(role + "-status");
    status.textContent = camera?.ready ? "在线" : camera ? "画面过期" : "未连接";
    status.className = camera?.ready ? "ready" : "";
    $(role + "-info").textContent = camera ? `640 × 480 RGB · ${camera.fps == null ? "—" : camera.fps.toFixed(1)} fps · ${camera.age_ms == null ? "—" : camera.age_ms.toFixed(0)} ms` : "—";
    image.hidden = !camera;
    if (camera && !image.getAttribute("src")) image.src = "/stream/" + role;
    if (!camera) image.removeAttribute("src");
    image.dataset.frame = camera?.frame ?? "";
  }
  $("devices").replaceChildren();
  for (const [name,online] of [["AVP",controlReady && c.tracking_valid],["机械臂",controlReady],["灵巧手",controlReady],["遥测",s.telemetry_alive]]) {
    const span = document.createElement("span"), indicator = document.createElement("i");
    span.textContent = name; indicator.textContent = online ? "在线" : "等待"; indicator.className = online ? "ok" : "";
    span.append(indicator); $("devices").append(span);
  }
  const disk = document.createElement("span"); disk.id = "disk"; disk.textContent = `可用 ${s.disk_free_gb} GB`; $("devices").append(disk);
  for (const [id,highlight] of [["prepare",!controlReady],["anchor",controlReady && !c.avp_anchored],["record",controlReady && c.avp_anchored && !pending],["review",pending]]) $("step-" + id).classList.toggle("active",!!highlight);
  qualityView(s);
  const history = JSON.stringify(s.saved);
  if (history !== previousHistory) {
    previousHistory = history; $("history").replaceChildren();
    if (!s.saved.length) { $("history").textContent = "完成第一条示教后，记录会显示在这里。"; $("history").className = "empty-state"; }
    else {
      $("history").className = "";
      for (const item of s.saved) {
        const row = document.createElement("div"), label = document.createElement("code"), tag = document.createElement("span"), path = document.createElement("p");
        row.className = "history-row"; label.textContent = item.id; tag.textContent = (item.demo ? "演示 · " : "") + (item.tags?.join(" / ") || "已保存 RAW"); row.append(label,tag);
        path.className = "history-path"; path.textContent = item.path;
        $("history").append(row,path);
      }
    }
  }
  $("events").replaceChildren();
  for (const event of s.events.slice(-12).reverse()) { const p = document.createElement("p"); p.textContent = event.time.slice(11,19) + " · " + event.message; $("events").append(p); }
  $("log-path").textContent = s.log_path || "原生 MuJoCo 窗口保留诊断与数字键操作。";
}

document.querySelectorAll("[data-action]").forEach(button=>button.addEventListener("click",()=>run(button.dataset.action, button.dataset.action === "begin" ? {instruction:$("instruction").value} : {})));
document.querySelectorAll("[data-mode]").forEach(button=>button.addEventListener("click",()=>run("mode",{mode:button.dataset.mode})));
document.querySelectorAll("[data-motion]").forEach(button=>button.addEventListener("click",()=>run("script",{motion:button.dataset.motion})));
document.querySelectorAll("[data-decision]").forEach(button=>button.addEventListener("click",()=>run("confirm",{decision:button.dataset.decision,dagger:$("dagger").checked})));
$("task-template").addEventListener("change",()=>{$("instruction").value = $("task-template").value; $("instruction").focus();});
$("start").addEventListener("click", async()=>{
  if (busy) return;
  busy = true; if (state) render(state);
  try {
    const config = Object.fromEntries(new FormData($("config-form")));
    // Disabled controls are omitted by FormData; read the fixed set explicitly.
    for (const input of $("config-form").elements) if (input.name) config[input.name] = input.type === "checkbox" ? input.checked : input.value;
    config.instruction = $("instruction").value;
    await post("configure",{config}); await post("start");
  } catch(error) { toast(error.message); }
  finally { busy = false; if (state) render(state); }
});
$("reference-toggle").addEventListener("change",()=>{if(state)render(state);});
$("reference-alpha").addEventListener("input",()=>{$("head-reference").style.opacity=$("reference-alpha").value;});
$("scrub").addEventListener("input",()=>{playing=false; $("play").textContent="播放"; replay(Number($("scrub").value));});
$("play").addEventListener("click",()=>{playing=!playing; $("play").textContent=playing?"暂停":"播放"; if(replayFrame>=state.frames-1)replay(0);});
setInterval(()=>{if(playing && state?.recording_state==="PENDING_CONFIRMATION"){if(replayFrame+2>=state.frames){playing=false;$("play").textContent="播放";}else replay(replayFrame+2);}},1000/15);
document.addEventListener("keydown",event=>{
  if (event.repeat || event.ctrlKey || event.metaKey || event.altKey || event.target.closest("input,textarea,select,[contenteditable=true]") ) return;
  const key = event.key.toLowerCase();
  let button = null;
  if (/^[1-6]$/.test(key)) button = document.querySelector(`[data-mode="${modes[Number(key)-1]}"]`);
  else if ({c:1,a:1,d:1,i:1}[key]) button = $(({c:"calibrate",a:"arm",d:"disarm",i:"restart"})[key]);
  else if ({j:1,k:1,l:1}[key]) button = document.querySelector(`[data-motion="${({j:"close",k:"hold",l:"open"})[key]}"]`);
  if (button && !button.disabled && !button.closest("[hidden]")) {event.preventDefault();button.click();}
});
for (const role of ["head","right_wrist"]) $(role+"-live").addEventListener("error",()=>{setTimeout(()=>{if(state?.cameras[role]) $(role+"-live").src="/stream/"+role+"?reconnect="+Date.now();},2500);});
async function poll() {
  try { const response=await fetch("/api/status"); if(!response.ok)throw new Error(); render(await response.json()); }
  catch(error) { $("message").textContent="工作台连接中断，设备状态未确认；请查看原生窗口。"; if(state) {state.control.fresh=false;render(state);$("message").textContent="工作台连接中断，设备状态未确认；请查看原生窗口。";} }
  finally { setTimeout(poll,500); }
}
setInterval(()=>{$("clock").textContent=new Date().toLocaleTimeString("zh-CN",{hour12:false,timeZone:"Asia/Shanghai"});},1000);
poll();
