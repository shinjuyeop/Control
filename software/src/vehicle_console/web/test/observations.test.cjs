const { test } = require("node:test");
const assert = require("node:assert/strict");
const { ageSnapshot } = require("./build/domain/freshness.js");
const { pageAllowed, capabilities } = require("./build/domain/profile.js");
const { brakeReasons, steeringFlags } = require("./build/domain/vehicle.js");

test("Vehicle navigation and action capabilities come from the server", () => {
  const vehicle = { profile: "vehicle", capabilities: ["observe_vehicle", "observe_runtime", "observe_diagnostics"] };
  for (const page of ["overview", "drive", "steering", "safety", "runtime", "diagnostics"])
    assert.equal(pageAllowed(vehicle, page), true);
  for (const page of ["bench", "faults", "scenarios", "display"])
    assert.equal(pageAllowed(vehicle, page), false);
  assert.equal(capabilities(vehicle).control, false);
  assert.equal(capabilities({ ...vehicle, capabilities: ["control_bench"] }).control, false);
  assert.equal(capabilities({ profile: "hil", capabilities: ["control_bench"] }).control, true);
  assert.equal(pageAllowed({ ...vehicle, capabilities: [] }, "drive"), false);
});

test("Source age advances through a silent connection and expires independently", () => {
  const frame = { topics: {
    "/drive/status": { age_ms: 50, stale_after_ms: 300, fresh: true, rx_hz: 50, data: { speed_mps: 1 } },
    "/steering/status": { age_ms: 50, stale_after_ms: 500, fresh: true, rx_hz: 50, data: {} },
    "/diagnostics": { age_ms: 0, stale_after_ms: 3000, fresh: true, data: { status: [{ name: "old", age_ms: 2900, fresh: true }, { name: "current", age_ms: 0, fresh: true }] } },
  }, runtime: { STM_A: { age_ms: 10, progress_age_ms: 2900, stale_after_ms: 3000, status: "ok" } } };
  const aged = ageSnapshot(frame, 250);
  assert.equal(aged.topics["/drive/status"].fresh, false);
  assert.equal(aged.topics["/drive/status"].rx_hz, 0);
  assert.equal(aged.topics["/steering/status"].fresh, true);
  assert.equal(aged.runtime.STM_A.status, "stale");
  assert.equal(aged.topics["/diagnostics"].data.status[0].fresh, false);
  assert.equal(aged.topics["/diagnostics"].data.status[1].fresh, true);
  assert.equal(frame.topics["/drive/status"].fresh, true);
  assert.equal(ageSnapshot({ topics: {}, runtime: {} }, 5000).topics["/drive/status"], undefined);
});

test("Safety reason bits and unsupported enable feedback retain their meaning", () => {
  assert.equal(brakeReasons(0), "NONE");
  assert.equal(brakeReasons(5), "COMMAND_BRAKE · ESTOP");
  assert.match(brakeReasons(2), /0x0002/);
  assert.equal(steeringFlags.find(([field]) => field === "enabled_feedback_valid")[2], "info");
});
