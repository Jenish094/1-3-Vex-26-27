const minTravelDistanceInches = 1;
const minHeadingChangeRadians = 0.12;
const pi = Math.PI;

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

  const samples = [];
  for (const line of lines.slice(1)) {
    const fields = parseLine(line);
    if (fields.length < 4 || fields.slice(0, 4).some((field) => field === "")) continue;

    const sample = {
      timestamp: Number(fields[0]),
      xIn: Number(fields[1]),
      yIn: Number(fields[2]),
      headingRad: Number(fields[3])
    };
    if (Number.isInteger(sample.timestamp)
      && Number.isFinite(sample.xIn)
      && Number.isFinite(sample.yIn)
      && Number.isFinite(sample.headingRad)) {
      samples.push(sample);
    }
  }

  if (samples.length === 0) throw new Error("The CSV contains no valid samples.");
  return samples;
}

function distanceBetween(a, b) {
  return Math.hypot(b.xIn - a.xIn, b.yIn - a.yIn);
}

function headingChangeBetween(a, b) {
  let change = b.headingRad - a.headingRad;
  while (change > pi) change -= 2 * pi;
  while (change < -pi) change += 2 * pi;
  return Math.abs(change);
}

function convertRecording(text) {
  const samples = parseRecording(text);
  const filtered = [samples[0]];
  for (const candidate of samples.slice(1)) {
    const last = filtered[filtered.length - 1];
    if (distanceBetween(last, candidate) >= minTravelDistanceInches
      || headingChangeBetween(last, candidate) >= minHeadingChangeRadians) {
      filtered.push(candidate);
    }
  }

  const lines = [
    '#include "main.h"',
    '#include "odom.hpp"',
    "",
    "namespace auton {",
    "",
    "void recorded_routine() {",
    "    isAuton = true;"
  ];

  if (filtered.length >= 2) {
    const playbackTimeoutMs = Math.max(15000, samples[samples.length - 1].timestamp + 5000);
    lines.push('    const char *recorded_path = R"(');
    for (const sample of filtered) {
      lines.push(`        ${sample.xIn.toFixed(2)} ${sample.yIn.toFixed(2)}`);
    }
    lines.push(')";', `    robot.follow(recorded_path, ${playbackTimeoutMs}.0);`);
  } else {
    const sample = filtered[0];
    lines.push(`    odomSetPose(${sample.xIn}, ${sample.yIn}, ${sample.headingRad});`);
  }

  lines.push("    isAuton = false;", "}", "", "} // namespace auton", "");
  return { code: lines.join("\n"), count: filtered.length };
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
    status.textContent = `Converted ${result.count} filtered odometry points.`;
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
