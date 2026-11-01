const motorNames = [
  "MOTOR_L1", "MOTOR_L2", "MOTOR_R1", "MOTOR_R2", "MOTOR_INTAKE_11W",
  "MOTOR_CASCADE_11WR", "MOTOR_CASCADE_11WL", "MOTOR_SCORE_OPEN", "MOTOR_SCORE_ROLL"
];

const motorHeaders = [
  ["l1", "motor_l1"], ["l2", "motor_l2"], ["r1", "motor_r1"], ["r2", "motor_r2"],
  ["intake", "motor_intake_11w"], ["cascade_right", "motor_cascade_11wr"],
  ["cascade_left", "motor_cascade_11wl"], ["score_open", "motor_score_open"],
  ["score_roll", "motor_score_roll"]
];

const fileInput = document.querySelector("#recording");
const convertButton = document.querySelector("#convert");
const downloadButton = document.querySelector("#download");
const output = document.querySelector("#output");
const status = document.querySelector("#status");
let selectedFile = null;
let generatedCode = "";

function parseLine(line) {
  return line.split(",").map((field) => field.trim());
}

function parseRecording(text) {
  const lines = text.split(/\r?\n/).filter((line) => line.trim() !== "");
  if (lines.length < 2) throw new Error("The CSV does not contain any samples.");

  const header = parseLine(lines[0]);
  const normalizedHeader = header.map((field) => field.toLowerCase());
  const timestampIndex = normalizedHeader.indexOf("timestamp_ms");
  const actualMotorIndexes = motorHeaders.map((aliases) =>
    aliases.map((alias) => normalizedHeader.indexOf(alias))
      .find((index) => index >= 0)
  );

  if (timestampIndex < 0 || actualMotorIndexes.some((index) => index < 0)) {
    throw new Error("The CSV header is missing required recorder columns.");
  }

  const samples = [];
  for (const line of lines.slice(1)) {
    const fields = parseLine(line);
    const timestamp = Number(fields[timestampIndex]);
    const motors = actualMotorIndexes.map((index) =>
      Math.max(-127, Math.min(127, Number(fields[index])))
    );
    if (Number.isInteger(timestamp) && motors.every(Number.isFinite)) {
      samples.push({ timestamp, motors });
    }
  }

  if (samples.length === 0) throw new Error("The CSV contains no valid samples.");
  return samples;
}

function motorCommands(sample, previous) {
  return sample.motors
    .map((value, index) => value !== previous[index]
      ? `        devices.Motors.${motorNames[index]}.set(${value});`
      : "")
    .filter(Boolean)
    .join("\n");
}

function convertRecording(text) {
  const samples = parseRecording(text);
  const lines = [
    '#include "main.h"',
    '#include "hardware.hpp"',
    "",
    "namespace auton {",
    "",
    "void recorded_routine() {",
    "    isAuton = true;"
  ];
  let previous = Array(motorNames.length).fill(0);
  let previousTimestamp = samples[0].timestamp;

  for (const sample of samples) {
    const delay = Math.max(0, sample.timestamp - previousTimestamp);
    if (delay > 0) lines.push(`        pros::delay(${delay});`);
    const commands = motorCommands(sample, previous);
    if (commands) lines.push(commands);
    previous = sample.motors;
    previousTimestamp = sample.timestamp;
  }

  lines.push("        pros::delay(20);");
  for (const motor of motorNames) {
    lines.push(`        devices.Motors.${motor}.set(0);`);
  }
  lines.push("    isAuton = false;", "}", "", "} // namespace auton", "");
  return { code: lines.join("\n"), count: samples.length };
}

fileInput.addEventListener("change", () => {
  selectedFile = fileInput.files[0] || null;
  convertButton.disabled = !selectedFile;
  downloadButton.disabled = true;
  output.value = "";
  status.textContent = selectedFile ? selectedFile.name : "";
});

convertButton.addEventListener("click", async () => {
  if (!selectedFile) return;
  try {
    const result = convertRecording(await selectedFile.text());
    generatedCode = result.code;
    output.value = generatedCode;
    downloadButton.disabled = false;
    status.textContent = `Converted ${result.count} samples.`;
  } catch (error) {
    generatedCode = "";
    output.value = "";
    downloadButton.disabled = true;
    status.textContent = error.message;
  }
});

downloadButton.addEventListener("click", () => {
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([generatedCode], { type: "text/plain" }));
  link.download = "recorded_auton.cpp";
  link.click();
  URL.revokeObjectURL(link.href);
});
