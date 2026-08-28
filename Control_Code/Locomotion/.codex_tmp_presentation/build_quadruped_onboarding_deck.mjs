import fs from "node:fs/promises";
import path from "node:path";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const W = 1600;
const H = 900;
const SNAPSHOT = "f3c5199";
const OUT_DIR = path.resolve(".codex_tmp_presentation", "output");
const OUT_PPTX = path.join(OUT_DIR, "quadruped_project_onboarding_2026-08-29.pptx");

async function writeBlob(filePath, blob) {
  await fs.writeFile(filePath, new Uint8Array(await blob.arrayBuffer()));
}

function addGrid(slide, mode = "light") {
  const line = mode === "dark" ? "slate-800" : "slate-200";
  const width = mode === "dark" ? 1.2 : 0.8;
  for (let x = 80; x < W; x += 160) {
    slide.shapes.add({
      geometry: "line",
      position: { left: x, top: 0, width: 0, height: H },
      fill: "none",
      line: { style: "solid", fill: line, width },
    });
  }
  for (let y = 90; y < H; y += 135) {
    slide.shapes.add({
      geometry: "line",
      position: { left: 0, top: y, width: W, height: 0 },
      fill: "none",
      line: { style: "solid", fill: line, width },
    });
  }
}

function textbox(slide, text, position, style = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    position,
    fill: "none",
    line: { style: "solid", fill: "none", width: 0 },
  });
  shape.text = text;
  shape.text.style = {
    fontSize: style.fontSize ?? 22,
    bold: style.bold ?? false,
    color: style.color ?? "slate-800",
    alignment: style.alignment ?? "left",
  };
  return shape;
}

function box(slide, text, position, options = {}) {
  const geometry = options.geometry ?? "roundRect";
  const config = {
    geometry,
    position,
    fill: options.fill ?? "white",
    line: { style: "solid", fill: options.line ?? "slate-200", width: options.lineWidth ?? 1.2 },
    shadow: options.shadow ?? "shadow-none",
  };
  if (["rect", "textbox", "roundRect"].includes(geometry)) {
    config.borderRadius = options.radius ?? 14;
  }
  const shape = slide.shapes.add(config);
  shape.text = text;
  shape.text.style = {
    fontSize: options.fontSize ?? 20,
    bold: options.bold ?? false,
    color: options.color ?? "slate-800",
    alignment: options.alignment ?? "center",
  };
  return shape;
}

function pill(slide, text, left, top, width, fill, color = "slate-900") {
  return box(slide, text, { left, top, width, height: 42 }, {
    fill,
    line: fill,
    radius: 21,
    fontSize: 16,
    bold: true,
    color,
  });
}

function sectionHeader(slide, eyebrow, title, subtitle = "") {
  textbox(slide, eyebrow.toUpperCase(), { left: 80, top: 54, width: 520, height: 30 }, {
    fontSize: 14,
    bold: true,
    color: "cyan-700",
  });
  textbox(slide, title, { left: 80, top: 92, width: 1100, height: 76 }, {
    fontSize: 42,
    bold: true,
    color: "slate-950",
  });
  if (subtitle) {
    textbox(slide, subtitle, { left: 82, top: 160, width: 1180, height: 48 }, {
      fontSize: 20,
      color: "slate-600",
    });
  }
}

function footer(slide) {
  textbox(slide, `C++ repo snapshot ${SNAPSHOT} | teammate onboarding | Aug 29, 2026`, {
    left: 80,
    top: 848,
    width: 760,
    height: 24,
  }, { fontSize: 12, color: "slate-500" });
}

function notes(slide, lines) {
  slide.speakerNotes.textFrame.setText(lines);
  slide.speakerNotes.setVisible(true);
}

function arrow(slide, from, to, color = "cyan-600") {
  slide.shapes.connect(from, to, {
    kind: "elbow",
    fromSide: "right",
    toSide: "left",
    line: { style: "solid", fill: color, width: 2 },
    tail: { type: "triangle", width: "sm", length: "sm" },
  });
}

function line(slide, left, top, width, height, color = "slate-400", lineWidth = 2) {
  slide.shapes.add({
    geometry: "line",
    position: { left, top, width, height },
    fill: "none",
    line: { style: "solid", fill: color, width: lineWidth },
  });
}

function quadrupedIcon(slide, left, top, scale = 1, mode = "light") {
  const stroke = mode === "dark" ? "cyan-300" : "cyan-700";
  const fill = mode === "dark" ? "slate-900" : "white";
  box(slide, "", { left: left + 72 * scale, top: top + 32 * scale, width: 260 * scale, height: 72 * scale }, {
    fill,
    line: stroke,
    lineWidth: 2,
    radius: 20,
  });
  for (const dx of [92, 142, 248, 298]) {
    line(slide, left + dx * scale, top + 98 * scale, 0, 118 * scale, stroke, 4);
    line(slide, left + (dx - 22) * scale, top + 216 * scale, 44 * scale, 0, stroke, 4);
  }
  for (const dx of [108, 268]) {
    box(slide, "", { left: left + dx * scale, top: top + 14 * scale, width: 34 * scale, height: 34 * scale }, {
      geometry: "ellipse",
      fill: "amber-300",
      line: "amber-500",
      lineWidth: 1,
    });
  }
}

function formula(slide, label, body, position, accent = "cyan") {
  box(slide, "", position, {
    fill: accent === "amber" ? "amber-50" : accent === "rose" ? "rose-50" : "cyan-50",
    line: accent === "amber" ? "amber-300" : accent === "rose" ? "rose-300" : "cyan-300",
    radius: 12,
  });
  textbox(slide, label, { left: position.left + 22, top: position.top + 18, width: position.width - 44, height: 26 }, {
    fontSize: 16,
    bold: true,
    color: accent === "amber" ? "amber-700" : accent === "rose" ? "rose-700" : "cyan-800",
  });
  textbox(slide, body, { left: position.left + 22, top: position.top + 54, width: position.width - 44, height: position.height - 70 }, {
    fontSize: 18,
    color: "slate-900",
  });
}

function slide1(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-950";
  addGrid(slide, "dark");
  pill(slide, "NEW TEAMMATE PROJECT BRIEF", 86, 70, 320, "cyan-900", "cyan-100");
  textbox(slide, "Quadruped Robot\nProject Onboarding", { left: 86, top: 170, width: 760, height: 210 }, {
    fontSize: 58,
    bold: true,
    color: "white",
  });
  textbox(slide, "C++ whole-robot control, mechanical-dynamical architecture, and Stage 2 roadmap", {
    left: 90,
    top: 412,
    width: 700,
    height: 70,
  }, { fontSize: 24, color: "slate-300" });
  textbox(slide, "Presentation date: Aug 29, 2026\nSource snapshot: f3c5199", {
    left: 92,
    top: 560,
    width: 520,
    height: 70,
  }, { fontSize: 18, color: "slate-400" });
  quadrupedIcon(slide, 900, 250, 1.45, "dark");
  box(slide, "12 motors\n4 legs x 3 joints\nSocketCAN control", { left: 950, top: 615, width: 420, height: 116 }, {
    fill: "slate-900",
    line: "cyan-700",
    color: "cyan-100",
    fontSize: 24,
    bold: true,
  });
  notes(slide, [
    "Open by framing this as an engineering onboarding session, not just a demo recap.",
    "Use Aug 29, 2026 because the user said the presentation is tomorrow from the Aug 28, 2026 working date.",
    "[Sources]",
    "- README.md:3-8 states this is the C++ quadruped control repo and motor-control targets use Raspberry Pi/Linux SocketCAN.",
    "- CMakeLists.txt:10-11,25,58,119,168-172 show C++23, low-level actuator, robot model, IMU reader, and quadruped_robot_locomotion_v3 target linkage.",
  ]);
}

function slide2(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-50";
  addGrid(slide, "light");
  sectionHeader(slide, "Introduction", "What we are building", "A physical quadruped platform where mechanical design, embedded sensing, CAN control, and locomotion software meet.");
  const items = [
    ["Mechanical body", "CAD / URDF / MuJoCo model, four legs, custom actuator assemblies."],
    ["Whole-robot C++", "Linux process centered on quadruped_robot_locomotion_v3."],
    ["Motor network", "SocketCAN routes commands and feedback for twelve motor drivers."],
    ["Learning platform", "Each teammate can own a sub-system and produce measurable evidence."],
  ];
  items.forEach(([title, body], idx) => {
    const x = 110 + (idx % 2) * 700;
    const y = 270 + Math.floor(idx / 2) * 220;
    box(slide, "", { left: x, top: y, width: 610, height: 152 }, {
      fill: "white",
      line: idx % 2 === 0 ? "cyan-300" : "amber-300",
      radius: 16,
      shadow: "shadow-sm",
    });
    textbox(slide, title, { left: x + 28, top: y + 24, width: 540, height: 30 }, {
      fontSize: 24,
      bold: true,
      color: "slate-950",
    });
    textbox(slide, body, { left: x + 28, top: y + 68, width: 540, height: 58 }, {
      fontSize: 19,
      color: "slate-600",
    });
  });
  footer(slide);
  notes(slide, [
    "Main talk track: this project is both a robot and a training ground. New teammates should see where their work can plug in.",
    "Say explicitly that the deck is source-grounded where possible, but hardware status comes from the presenter's current report.",
    "[Sources]",
    "- README.md:3-8 identifies the C++ quadruped control programs and Raspberry Pi/Linux SocketCAN boundary.",
    "- Simulation/mujoco/README.md:3-23 describes preparing a SolidWorks URDF as a MuJoCo-ready model.",
    "- robot_hardware_config.h:15-24 lists four legs, four CAN channels, twelve motors, and 25:1 gear ratios.",
  ]);
}

function slide3(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "white";
  addGrid(slide, "light");
  sectionHeader(slide, "Objective", "Build a reliable quadruped learning platform", "The engineering goal is controlled locomotion; the team goal is ownership, evidence, and practical robotics experience.");
  box(slide, "Project objective\n\nProgress from deliberate walking to reliable dynamic locomotion while making every subsystem inspectable, testable, and teachable.", {
    left: 125,
    top: 258,
    width: 610,
    height: 360,
  }, { fill: "slate-950", line: "slate-900", color: "white", fontSize: 28, bold: true, radius: 18 });
  const goals = [
    ["Locomotion", "Stable gait states, foot targets, IK/FK, and validated hardware trials."],
    ["Hardware", "Lower mass, better actuator behavior, cleaner wiring, and repeatable calibration."],
    ["Safety", "Fresh feedback, loop timing checks, safe stop, and telemetry during every run."],
    ["Simulation", "Use MuJoCo to de-risk gait changes before hardware testing."],
  ];
  goals.forEach(([t, b], i) => {
    const y = 252 + i * 105;
    box(slide, "", { left: 800, top: y, width: 640, height: 84 }, {
      fill: i % 2 ? "cyan-50" : "amber-50",
      line: i % 2 ? "cyan-200" : "amber-200",
      radius: 12,
    });
    textbox(slide, t, { left: 826, top: y + 18, width: 170, height: 28 }, {
      fontSize: 22,
      bold: true,
      color: "slate-950",
    });
    textbox(slide, b, { left: 1010, top: y + 15, width: 390, height: 48 }, {
      fontSize: 18,
      color: "slate-700",
    });
  });
  footer(slide);
  notes(slide, [
    "Explain the dual objective: make the robot walk better and make the project a place where teammates gain real skill.",
    "Use the four objective lanes to invite teammates into a concrete sub-system.",
    "[Sources]",
    "- CMakeLists.txt:25-33,58-80,132-173 show the source split across low-level actuator, robot model, leg controllers, Xbox input, IMU reader, and the active executable.",
    "- Simulation/mujoco/README.md:41-51 says simulation samples the current C++ trajectory/gait/FK/IK sources and drives all 12 joints in MuJoCo.",
  ]);
}

function slide4(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-50";
  addGrid(slide, "light");
  sectionHeader(slide, "Current Status", "Stage 1 proved movement; Stage 2 is reliability", "Keep the source capabilities and hardware-verified behavior separate.");
  const stage1 = box(slide, "", { left: 100, top: 250, width: 650, height: 400 }, {
    fill: "white",
    line: "emerald-300",
    radius: 18,
    shadow: "shadow-sm",
  });
  const stage2 = box(slide, "", { left: 850, top: 250, width: 650, height: 400 }, {
    fill: "white",
    line: "cyan-300",
    radius: 18,
    shadow: "shadow-sm",
  });
  textbox(slide, "Stage 1: current demo baseline", { left: 135, top: 292, width: 570, height: 38 }, {
    fontSize: 28,
    bold: true,
    color: "emerald-700",
  });
  textbox(slide, "- Hardware report: able to walk reversely\n- Not yet reliable for lateral walking or yawing\n- Source already contains lateral/yaw profile logic, so code capability is ahead of demonstrated behavior\n- Baseline still needs repeatability evidence", {
    left: 145,
    top: 358,
    width: 540,
    height: 210,
  }, { fontSize: 23, color: "slate-750" });
  textbox(slide, "Stage 2: next engineering target", { left: 885, top: 292, width: 570, height: 38 }, {
    fontSize: 28,
    bold: true,
    color: "cyan-800",
  });
  textbox(slide, "- More dynamic walking\n- More reliable locomotion\n- Better actuators and a lighter robot\n- Advance toward MIT mode and MPC after feedback and model quality improve", {
    left: 895,
    top: 358,
    width: 540,
    height: 210,
  }, { fontSize: 23, color: "slate-750" });
  line(slide, 760, 450, 80, 0, "slate-400", 3);
  box(slide, "evidence gate", { left: 710, top: 414, width: 165, height: 70 }, {
    fill: "amber-100",
    line: "amber-400",
    fontSize: 20,
    bold: true,
    radius: 12,
    color: "amber-900",
  });
  footer(slide);
  notes(slide, [
    "Presenter status claim: Stage 1 reverse walking, not lateral/yawing yet, comes from the user's request in this session, not from a repo test log.",
    "Important phrasing: the code has lateral and yaw paths, but that is not the same as hardware-verified lateral/yaw locomotion.",
    "[Sources]",
    "- User-provided status in Aug 28, 2026 request: Stage 1 reverse walking; not lateral or yawing; Stage 2 aims at dynamic and reliable locomotion.",
    "- xbox_controller.h:41-46 and xbox_controller.cpp:254-267 show forward, lateral, yaw, vertical step input fields and the Right Bumper enable gate.",
    "- robotGaitControl.cpp:253-349 and quadruped_trajectory.h:7-10 show profile selection for forward/cycloidal, lateral, yaw, and vertical step modes.",
  ]);
}

function slide5(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "white";
  addGrid(slide, "light");
  sectionHeader(slide, "Mechanical Dynamics", "CAD model and physical architecture", "Stage 2 needs a model that matches the real robot closely enough for control decisions.");
  quadrupedIcon(slide, 110, 302, 1.12, "light");
  const cards = [
    ["Body + four legs", "Each leg has hip, thigh, and shank joints, for 12 commanded joints total."],
    ["Actuator assemblies", "Cycloidal actuator visuals exist in the MuJoCo prep path; real torque linkage modeling remains a caveat."],
    ["Mass properties", "CAD/URDF/MJCF mass, COM, inertia, and collision geometry need numerical validation before model-based control."],
    ["Contact model", "Ground contact and friction exist in simulation; hardware contact detection is not proved by this C++ source."],
  ];
  cards.forEach(([t, b], i) => {
    const x = i % 2 === 0 ? 650 : 1080;
    const y = 260 + Math.floor(i / 2) * 185;
    box(slide, "", { left: x, top: y, width: 360, height: 140 }, {
      fill: i === 2 ? "rose-50" : "slate-50",
      line: i === 2 ? "rose-300" : "slate-200",
      radius: 12,
    });
    textbox(slide, t, { left: x + 24, top: y + 22, width: 310, height: 30 }, {
      fontSize: 22,
      bold: true,
      color: "slate-950",
    });
    textbox(slide, b, { left: x + 24, top: y + 62, width: 308, height: 62 }, {
      fontSize: 16,
      color: "slate-650",
    });
  });
  formula(slide, "Stage 2 dynamics model target", "M(q) qddot + C(q,qdot) qdot + g(q) = S^T tau + J_c(q)^T lambda", {
    left: 170,
    top: 640,
    width: 1260,
    height: 110,
  }, "amber");
  footer(slide);
  notes(slide, [
    "Use this slide to explain why mechanical accuracy matters: model-based control is only as useful as the mass, inertia, actuator, and contact assumptions.",
    "The dynamics equation is a Stage 2 control-model target, not evidence that the current C++ loop solves full-body dynamics.",
    "[Sources]",
    "- robot_hardware_config.h:15-24 gives four leg configs, twelve device IDs, four CAN channels, and 25:1 reductions.",
    "- Simulation/mujoco/README.md:3-23 describes preparing the SolidWorks URDF as a MuJoCo-ready visual copy.",
    "- Simulation/mujoco/README.md:146-163 describes COM markers, 11.84 kg model mass, 25:1 reduction, idealized joint-side gains, and provisional collision/friction/servo assumptions.",
    "- Simulation/mujoco/README.md:165-192 says cycloidal actuator visuals do not change mass, COM, inertia, collision, or RL dynamics and physics-ready geometry must be added separately.",
    "- Memory-derived older CAD report note: 17.18 kg and assembly-level mass/COM override concerns may be stale and should be rechecked before presenting as current CAD truth.",
  ]);
}

function slide6(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-950";
  addGrid(slide, "dark");
  textbox(slide, "RUNTIME COMMAND PATH", { left: 80, top: 54, width: 520, height: 30 }, {
    fontSize: 14,
    bold: true,
    color: "cyan-300",
  });
  textbox(slide, "From joystick intent to twelve actuators", { left: 80, top: 95, width: 1000, height: 60 }, {
    fontSize: 42,
    bold: true,
    color: "white",
  });
  textbox(slide, "Centered on quadruped_robot_locomotion_v3", { left: 82, top: 165, width: 760, height: 34 }, {
    fontSize: 20,
    color: "slate-300",
  });
  const labels = [
    ["Human / operator", "Xbox controller\n/dev/input/js0"],
    ["Xbox_Input", "XboxController\nreadInputCommand()"],
    ["Main loop", "timing + state\nv3 executable"],
    ["Gait control", "RobotGaitControl\nprofile + gait phase"],
    ["Trajectory", "quadruped_trajectory.cpp\nfoot targets"],
    ["Robot", "all-leg trajectory\ncommand"],
    ["Leg objects", "4 leg values\nIK/FK/Jacobian"],
    ["Joint objects", "12 joint values\nfresh feedback + targets"],
    ["CAN layer", "MotorController\nCanBusManager"],
    ["Hardware", "Linux SocketCAN\n12 motor drivers"],
  ];
  const shapes = [];
  labels.forEach(([topLabel, body], i) => {
    const row = i < 5 ? 0 : 1;
    const col = i % 5;
    const x = 86 + col * 298;
    const y = row === 0 ? 275 : 565;
    const s = box(slide, `${topLabel}\n${body}`, { left: x, top: y, width: 238, height: 128 }, {
      fill: i < 2 ? "slate-900" : i < 8 ? "cyan-950" : "amber-950",
      line: i < 2 ? "slate-600" : i < 8 ? "cyan-600" : "amber-500",
      color: "white",
      fontSize: 17,
      bold: true,
      radius: 14,
    });
    shapes.push(s);
  });
  for (let i = 0; i < 4; i += 1) arrow(slide, shapes[i], shapes[i + 1], "cyan-400");
  line(slide, 1470, 404, 0, 94, "cyan-400", 2);
  line(slide, 270, 501, 1200, 0, "cyan-400", 2);
  line(slide, 270, 501, 0, 64, "cyan-400", 2);
  for (let i = 5; i < 9; i += 1) arrow(slide, shapes[i], shapes[i + 1], "cyan-400");
  pill(slide, "Boundary: Linux process", 368, 742, 330, "cyan-900", "cyan-100");
  pill(slide, "Boundary: SocketCAN network", 1010, 742, 390, "amber-900", "amber-100");
  textbox(slide, `Source snapshot ${SNAPSHOT}`, { left: 80, top: 846, width: 260, height: 24 }, {
    fontSize: 12,
    color: "slate-500",
  });
  notes(slide, [
    "Talk through this in one clear path: input command, gait state, foot trajectory, robot/leg/joint command generation, motor PDO command, SocketCAN, drivers.",
    "Do not imply closed-loop balance or contact detection here. The feedback path is on the safety slide.",
    "[Sources]",
    "- quadruped_robot_locomotion_v3.cpp:30-36 creates RobotGaitControl, Robot, RobotShutdownGuard, and XboxController('/dev/input/js0').",
    "- quadruped_robot_locomotion_v3.cpp:243-244 reads xbox::readInputCommand(controller).",
    "- quadruped_robot_locomotion_v3.cpp:278-316 updates walking/landing and calls Robot::CalculateAllTrajectoryCommand().",
    "- quadruped_robot_locomotion_v3.cpp:336-345 commands all legs by PDO and prints telemetry.",
    "- joint.cpp:507-515 writes position and velocity by PDO2; can_bus_manager.cpp:53-85 routes received frames to MotorController::handleFrame.",
    "- socketcan.cpp:70-88 opens PF_CAN/CAN_RAW; socketcan.cpp:112-122 and 165-187 write/read CAN frames.",
  ]);
}

function slide7(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "white";
  addGrid(slide, "light");
  sectionHeader(slide, "C++ Ownership", "Robot owns Legs; Legs own Joints; Joints own Motors", "This relationship is the cleanest mental model for new teammates reading the control code.");
  const robot = box(slide, "Robot\nowns hardware_config_ and 4 Leg values", { left: 575, top: 235, width: 450, height: 96 }, {
    fill: "slate-950",
    line: "slate-900",
    color: "white",
    fontSize: 22,
    bold: true,
    radius: 16,
  });
  const legNames = ["front_left", "front_right", "rear_left", "rear_right"];
  const legShapes = legNames.map((name, i) => box(slide, `Leg\n${name}`, {
    left: 105 + i * 370,
    top: 420,
    width: 250,
    height: 92,
  }, { fill: "cyan-50", line: "cyan-300", color: "cyan-900", fontSize: 21, bold: true, radius: 14 }));
  legShapes.forEach((s) => arrow(slide, robot, s, "slate-500"));
  box(slide, "Inside each Leg\n\nCanBusManager bus\nJoint hip\nJoint thigh\nJoint shank", { left: 150, top: 600, width: 410, height: 165 }, {
    fill: "slate-50",
    line: "slate-300",
    color: "slate-900",
    fontSize: 22,
    bold: true,
    radius: 14,
  });
  const jointBox = box(slide, "Inside each Joint\n\nCanBusManager & bus\nMotorController motor\nname, gear ratio, direction", { left: 620, top: 600, width: 410, height: 165 }, {
    fill: "amber-50",
    line: "amber-300",
    color: "amber-950",
    fontSize: 22,
    bold: true,
    radius: 14,
  });
  const busBox = box(slide, "CAN routing\n\nMotorController registers with CanBusManager;\nreceiveLoop dispatches frames by device id.", { left: 1088, top: 600, width: 390, height: 165 }, {
    fill: "rose-50",
    line: "rose-300",
    color: "rose-950",
    fontSize: 19,
    bold: true,
    radius: 14,
  });
  arrow(slide, legShapes[0], jointBox, "slate-500");
  arrow(slide, jointBox, busBox, "slate-500");
  pill(slide, "hardware config maps leg -> can channel -> device ids -> motor signs", 256, 792, 1035, "slate-900", "white");
  footer(slide);
  notes(slide, [
    "This is the add-on schematic relationship the user asked for earlier, now as a slide.",
    "Key explanation: Robot contains Leg values directly. Leg contains one bus and three Joint values. Joint owns its MotorController and keeps a reference to the Leg's bus.",
    "[Sources]",
    "- robot.h:199-204 and robot.cpp:9-23 show Robot's hardware_config_ and four owned Leg members returned by getLegs().",
    "- leg.h:191-196 shows each Leg contains LegHardwareConfig, CanBusManager bus, Joint hip, Joint thigh, and Joint shank.",
    "- joint.h:196-197 shows Joint stores CanBusManager& bus and MotorController motor; joint.cpp:37-45 constructs motor(&bus, device_id).",
    "- can_bus_manager.h:41-47 and can_bus_manager.cpp:14-37,53-85 show motor registration and receiveLoop frame dispatch.",
    "- robot_hardware_config.h:15-24 maps each leg to can1-can4, motor device IDs 1-12, 25:1 gear ratio, and direction signs.",
  ]);
}

function slide8(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-50";
  addGrid(slide, "light");
  sectionHeader(slide, "Current Gait Path", "Gait state drives foot targets, then joint targets", "The current controller is kinematic trajectory tracking with safety checks, not full-body closed-loop balance.");
  const flow = [
    "InputCommand\nRB enable gate",
    "Motion profile\nforward / lateral / yaw / vertical",
    "Run state\nStanding / Walking / Landing",
    "Gait phase\nmaster phase + leg offsets",
    "Foot target\np_d and v_d",
    "Joint target\nIK + damped Jacobian",
  ];
  const shapes = [];
  flow.forEach((label, i) => {
    const s = box(slide, label, { left: 92 + i * 246, top: 255, width: 205, height: 98 }, {
      fill: i % 2 ? "white" : "cyan-50",
      line: i % 2 ? "slate-300" : "cyan-300",
      color: "slate-950",
      fontSize: 18,
      bold: true,
      radius: 12,
    });
    shapes.push(s);
  });
  for (let i = 0; i < shapes.length - 1; i += 1) arrow(slide, shapes[i], shapes[i + 1], "cyan-700");
  formula(slide, "Current command math in the leg loop", [
    "p_d[k] = p_0 + r_profile(phi_k)",
    "q_ik[k] = IK(p_d[k])",
    "v_d[k] = (p_d[k] - p_d[k-1]) / dt",
    "qdot_cmd = sat(J_lambda^{-1} v_d + K_p(q_ik - q), qdot_max)",
    "q_next = q + dt qdot_cmd",
  ].join("\n"), { left: 135, top: 455, width: 620, height: 260 }, "cyan");
  formula(slide, "What is not proved by this source", [
    "No inferred joystick signs",
    "No source-proved closed-loop balance",
    "No source-proved contact detection",
    "No hardware-verified lateral/yaw claim",
  ].join("\n"), { left: 845, top: 455, width: 620, height: 260 }, "rose");
  footer(slide);
  notes(slide, [
    "This slide is for explaining the live walking code at a systems level.",
    "Say that forward/lateral/yaw/vertical profiles exist in code, but current hardware demonstration status is narrower.",
    "[Sources]",
    "- xbox_controller.h:41-46 and xbox_controller.cpp:254-267 define InputCommand fields and the Right Bumper enable gate.",
    "- robotGaitControl.h:65-70 defines Standing, Walking, and Landing; robotGaitControl.cpp:364-447 performs walking/landing state updates.",
    "- robotGaitControl.cpp:253-349 selects forward, lateral, turning, and vertical stepping profiles.",
    "- quadruped_trajectory.cpp:181-190 dispatches CycloidalTrot, VerticalStep, LeftToRight, and CircularYaw foot trajectories.",
    "- leg.cpp:462-519 uses analytical IK, the forward Jacobian, damped inverse Jacobian, velocity command limiting, and integrated next joint target.",
  ]);
}

function slide9(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "white";
  addGrid(slide, "light");
  sectionHeader(slide, "Kinematics", "The math teammates should recognize", "These formulas describe how foot motion becomes joint motion in the current control path.");
  formula(slide, "Forward kinematics", [
    "p_foot = f(q)",
    "q = [q_hip, q_thigh, q_shank]^T",
    "Use this to estimate measured foot position.",
  ].join("\n"), { left: 110, top: 250, width: 650, height: 165 }, "cyan");
  formula(slide, "Analytical inverse kinematics", [
    "q_ik = IK(p_d)",
    "cos(theta) arguments are clamped before acos()",
    "Use this to get desired joint position.",
  ].join("\n"), { left: 840, top: 250, width: 650, height: 165 }, "amber");
  formula(slide, "Jacobian velocity mapping", [
    "v_foot = J(q) qdot",
    "J_lambda^{-1} = (J^T J + lambda^2 I)^{-1} J^T",
    "qdot_des = J_lambda^{-1} v_d",
  ].join("\n"), { left: 110, top: 500, width: 650, height: 185 }, "cyan");
  formula(slide, "Force to torque map for future MIT/force work", [
    "tau_joint = J(q)^T F_foot",
    "tau_motor = tau_joint / gear_ratio",
    "Simulation uses this in a torque-only experiment, not as the active hardware controller.",
  ].join("\n"), { left: 840, top: 500, width: 650, height: 185 }, "rose");
  footer(slide);
  notes(slide, [
    "Keep this concise: FK tells where the foot is, IK tells what joint angles reach a desired foot point, Jacobian maps velocity, and J^T maps force to torque.",
    "The torque map belongs to future force/MIT-style control discussions; the active C++ command path sends position/velocity targets.",
    "[Sources]",
    "- leg.cpp:601-747 contains the FK, analytical IK, forward Jacobian, and damped inverse Jacobian implementations.",
    "- leg.cpp:462-519 shows the current trajectory command uses IK and damped Jacobian-based velocity before command limiting.",
    "- Simulation/mujoco/README.md:69-96 documents a torque-only Cartesian-force experiment using tau_joint = J.T @ foot_force and tau_motor = tau_joint / gear_ratio, and warns it is not a stable hardware controller.",
  ]);
}

function slide10(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-50";
  addGrid(slide, "light");
  sectionHeader(slide, "Control Architecture", "Current method, plus PID, LQR, MPC, and MIT mode", "Use this slide to orient teammates from today's code toward Stage 2 control.");
  const entries = [
    ["Current C++ path", "PDO2 position + velocity target\nq_next = q + dt * qdot_cmd\nSafety gates timing, freshness, tracking, velocity, homing"],
    ["PID", "e = r - y\nu(t)=K_p e(t)+K_i int_0^t e(tau)dtau+K_d de/dt"],
    ["LQR", "x_{k+1}=A x_k + B u_k\nu_k=-K x_k\nK=(R+B^T P B)^{-1}B^T P A"],
    ["MPC", "min sum ||x_k-x_ref||_Q^2 + ||u_k||_R^2\ns.t. dynamics, friction, torque, joint limits"],
    ["MIT mode", "tau = K_p(q_d-q)+K_d(qdot_d-qdot)+tau_ff\nNeeds validated actuator behavior and feedback freshness"],
  ];
  entries.forEach(([title, body], i) => {
    const x = i === 0 ? 110 : 110 + ((i - 1) % 2) * 720;
    const y = i === 0 ? 240 : 430 + Math.floor((i - 1) / 2) * 195;
    const w = i === 0 ? 1380 : 650;
    const h = i === 0 ? 130 : 150;
    box(slide, "", { left: x, top: y, width: w, height: h }, {
      fill: i === 0 ? "slate-950" : "white",
      line: i === 0 ? "slate-900" : i % 2 ? "cyan-300" : "amber-300",
      radius: 14,
      shadow: "shadow-sm",
    });
    textbox(slide, title, { left: x + 24, top: y + 20, width: 230, height: 32 }, {
      fontSize: 22,
      bold: true,
      color: i === 0 ? "white" : "slate-950",
    });
    textbox(slide, body, { left: x + 270, top: y + 20, width: w - 310, height: h - 34 }, {
      fontSize: i === 0 ? 20 : 17,
      color: i === 0 ? "slate-200" : "slate-700",
    });
  });
  footer(slide);
  notes(slide, [
    "Make a clear distinction: PID/LQR/MPC/MIT formulas are teaching and roadmap material. The current C++ path is kinematic target generation plus PDO2 position/velocity commands.",
    "For Stage 2, MIT mode and MPC are reasonable goals only after actuator behavior, model parameters, feedback timing, and safety boundaries are validated.",
    "[Sources]",
    "- joint.cpp:507-515 writes position/velocity target PDO2 frames.",
    "- robot_safety.h:32-45 and robot_safety.cpp:8-88 define checks for control dt, feedback age, tracking, measured velocity, and homing motion.",
    "- leg.cpp:462-519 shows the current kinematic command method.",
    "- Memory-derived Recoil review note: PDO3 target plus feedforward torque resembled fixed-gain position impedance, not complete validated MIT control; recheck firmware before assigning Stage 2 work.",
  ]);
}

function slide11(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "white";
  addGrid(slide, "light");
  sectionHeader(slide, "Safety And Sensing", "Safety, telemetry, and partial IMU", "This is the slide that prevents accidental overclaiming.");
  const blocks = [
    ["Joint feedback path", "Motor position/velocity PDO feedback -> CanBusManager / MotorController -> Joint fresh-feedback checks -> Leg and Robot arrays -> RobotSafety -> main loop"],
    ["Safety path", "RobotSafety validates loop timing, feedback freshness, joint tracking, measured velocity, and homing. RobotShutdownGuard performs safe stop on exit."],
    ["Telemetry path", "robot_telemetry.cpp receives control state, gait state, feedback, targets, timing, and speed scale. Keep it outside the command path."],
    ["IMU path", "CMake links imu_uart_reader and Imu defaults to /dev/ttyAMA2, but quadruped_robot_locomotion_v3.cpp does not instantiate Imu. Current ESP32 source uses BNO055 + USB Serial, so label this in progress."],
  ];
  blocks.forEach(([t, b], i) => {
    const x = i % 2 === 0 ? 105 : 830;
    const y = 250 + Math.floor(i / 2) * 245;
    box(slide, "", { left: x, top: y, width: 665, height: 180 }, {
      fill: i === 3 ? "rose-50" : i === 1 ? "amber-50" : "slate-50",
      line: i === 3 ? "rose-300" : i === 1 ? "amber-300" : "slate-300",
      radius: 14,
      shadow: "shadow-sm",
    });
    textbox(slide, t, { left: x + 26, top: y + 22, width: 590, height: 32 }, {
      fontSize: 24,
      bold: true,
      color: i === 3 ? "rose-800" : "slate-950",
    });
    textbox(slide, b, { left: x + 26, top: y + 67, width: 590, height: 88 }, {
      fontSize: 18,
      color: "slate-700",
    });
  });
  footer(slide);
  notes(slide, [
    "Use this to explain how the robot avoids blind command streaming: timing, feedback, tracking, velocity, and homing checks are explicit.",
    "IMU phrasing must be careful: reader library is linked, but not active in the main locomotion loop. The ESP32 source currently shows BNO055 and USB Serial, not a confirmed hardware-stabilizing UART link.",
    "[Sources]",
    "- motor_controller.cpp:490-543 stores PDO2 position/velocity feedback and exposes latest measurements.",
    "- joint.cpp:505-642 reads fresh feedback and waits for fresh feedback.",
    "- quadruped_robot_locomotion_v3.cpp:138-217 validates control dt and robot feedback before command generation.",
    "- robot_safety.h:32-45 and robot_safety.cpp:8-88 define timing, feedback, tracking, velocity, and homing checks.",
    "- robot.cpp:460-475 implements RobotShutdownGuard, whose destructor calls safeStopRobot unless dismissed.",
    "- robot_telemetry.cpp:79-139 prints gait state, master phase, feedback, target position/velocity, timing, and speed scale.",
    "- CMakeLists.txt:119-126 and 168-172 link imu_uart_reader; imu.h:29-37 describes the Pi UART reader and default /dev/ttyAMA2; quadruped_robot_locomotion_v3.cpp has no Imu instantiation.",
    "- QUADRUPED-ESP32-IMU/src/main.cpp:10,174-183,239-263 show BNO055 setup and USB Serial output path; no Serial2/HardwareSerial packet sender was found in the inspected source.",
  ]);
}

function slide12(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-950";
  addGrid(slide, "dark");
  textbox(slide, "TEAM OWNERSHIP", { left: 80, top: 54, width: 520, height: 30 }, {
    fontSize: 14,
    bold: true,
    color: "cyan-300",
  });
  textbox(slide, "What do you want to obtain from this project?", { left: 80, top: 98, width: 1220, height: 64 }, {
    fontSize: 42,
    bold: true,
    color: "white",
  });
  textbox(slide, "As leader, use their answers to assign subtasks that create real ownership and visible learning.", {
    left: 82,
    top: 170,
    width: 1050,
    height: 36,
  }, { fontSize: 20, color: "slate-300" });
  const qs = [
    "What skill do you want to gain?",
    "Which subsystem do you want to own?",
    "What deliverable would make you proud?",
    "How much hardware risk do you want?",
  ];
  qs.forEach((q, i) => {
    box(slide, q, { left: 115 + i * 365, top: 265, width: 300, height: 98 }, {
      fill: "slate-900",
      line: i % 2 ? "amber-500" : "cyan-600",
      color: "white",
      fontSize: 22,
      bold: true,
      radius: 14,
    });
  });
  const tracks = [
    "Mechanical / lightweight redesign",
    "Actuator testing and MIT-mode bring-up",
    "CAN reliability and safety instrumentation",
    "Gait, IK/FK, MPC, and controls",
    "MuJoCo simulation and validation",
    "Docs, tests, telemetry, and onboarding",
  ];
  tracks.forEach((track, i) => {
    const x = 140 + (i % 3) * 455;
    const y = 455 + Math.floor(i / 3) * 120;
    box(slide, track, { left: x, top: y, width: 365, height: 78 }, {
      fill: "cyan-950",
      line: "cyan-700",
      color: "cyan-50",
      fontSize: 19,
      bold: true,
      radius: 12,
    });
  });
  textbox(slide, `Source snapshot ${SNAPSHOT}`, { left: 80, top: 846, width: 260, height: 24 }, {
    fontSize: 12,
    color: "slate-500",
  });
  notes(slide, [
    "This slide is intentionally open-ended. It helps teammates tell you whether they want controls, mechanical design, firmware, simulation, safety, or documentation experience.",
    "Suggested leader language: I want each person to own a small subsystem with a testable deliverable, so you can point to something real by the end of the stage.",
    "[Sources]",
    "- User explicitly requested an open-ended question so teammate tasks can be designed around ownership and experience.",
    "- The ownership tracks come from the inspected code and repo structure: Control_Code/Locomotion, Xbox_Input, lowlevel_actuator, Simulation/mujoco, QUADRUPED-ESP32-IMU, and docs/archify.",
  ]);
}

function slide13(presentation) {
  const slide = presentation.slides.add();
  slide.background.fill = "slate-50";
  addGrid(slide, "light");
  sectionHeader(slide, "Future Plan", "Stage 2 roadmap", "Make the robot lighter, safer, and more dynamic. Advance only when evidence gates pass: source review, simulation, bench test, then hardware walking.");
  const steps = [
    ["1. Baseline", "Repeat reverse walking; log telemetry; define pass/fail criteria."],
    ["2. Hardware", "Reduce mass/inertia; characterize actuators; validate gear ratios, signs, limits, and thermal behavior."],
    ["3. Low-level control", "Bring up validated MIT-style torque/impedance mode with watchdogs and stale-target clearing."],
    ["4. Model control", "Add state estimation, body dynamics, contact assumptions, and MPC preview planning."],
    ["5. Expand gaits", "Reverse -> forward -> lateral -> yaw -> transitions -> disturbance tests."],
  ];
  steps.forEach(([t, b], i) => {
    const x = 120 + i * 286;
    box(slide, "", { left: x, top: 270, width: 230, height: 310 }, {
      fill: i < 2 ? "white" : i < 4 ? "cyan-50" : "amber-50",
      line: i < 2 ? "slate-300" : i < 4 ? "cyan-300" : "amber-300",
      radius: 14,
      shadow: "shadow-sm",
    });
    textbox(slide, t, { left: x + 20, top: 300, width: 190, height: 34 }, {
      fontSize: 22,
      bold: true,
      color: "slate-950",
    });
    textbox(slide, b, { left: x + 20, top: 360, width: 190, height: 160 }, {
      fontSize: 17,
      color: "slate-700",
    });
  });
  formula(slide, "Stage 2 decision rule", "Do not upgrade the controller faster than the evidence: compile -> simulate -> bench -> tethered hardware -> untethered demo.", {
    left: 175,
    top: 665,
    width: 1250,
    height: 108,
  }, "amber");
  footer(slide);
  notes(slide, [
    "Close by assigning next steps around evidence. The roadmap is ambitious, but it should not skip validation.",
    "If teammates ask why MPC cannot be first: it depends on model quality, actuator trust, contact assumptions, and state estimation.",
    "[Sources]",
    "- robot_safety.h:32-45 and robot_safety.cpp:8-88 show existing safety checks that Stage 2 should preserve and extend.",
    "- Simulation/mujoco/README.md:41-51 and 69-96 show current simulation paths, including a source-sampled C++ trajectory runner and a torque experiment explicitly labeled not hardware-stable.",
    "- Simulation/mujoco/README.md:146-163 and 189-192 describe provisional mass, COM, inertia, collision, and actuator modeling caveats.",
    "- Memory-derived Recoil review note says MIT/PDO3-related firmware conclusions are unvalidated until source, build, flash, and hardware validation are repeated.",
  ]);
}

async function main() {
  await fs.mkdir(OUT_DIR, { recursive: true });
  const presentation = Presentation.create({ slideSize: { width: W, height: H } });

  slide1(presentation);
  slide2(presentation);
  slide3(presentation);
  slide4(presentation);
  slide5(presentation);
  slide6(presentation);
  slide7(presentation);
  slide8(presentation);
  slide9(presentation);
  slide10(presentation);
  slide11(presentation);
  slide12(presentation);
  slide13(presentation);

  for (const [index, slide] of presentation.slides.items.entries()) {
    const stem = `slide-${String(index + 1).padStart(2, "0")}`;
    await writeBlob(path.join(OUT_DIR, `${stem}.png`), await presentation.export({ slide, format: "png", scale: 1 }));
    const layout = await slide.export({ format: "layout" });
    await fs.writeFile(path.join(OUT_DIR, `${stem}.layout.json`), await layout.text());
  }
  await writeBlob(path.join(OUT_DIR, "deck-montage.webp"), await presentation.export({ format: "webp", montage: true, scale: 1 }));
  const pptx = await PresentationFile.exportPptx(presentation);
  await pptx.save(OUT_PPTX);
  console.log(OUT_PPTX);
}

main().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
