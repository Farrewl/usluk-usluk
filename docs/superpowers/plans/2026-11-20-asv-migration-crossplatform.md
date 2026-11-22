# ASV Migration to PX4 + WebSocket Relay + Cross-Platform Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate the Aterkia ASV backend from monolithic Python/PyQt + ArduRover logic toward the PX4/WebSocket/Relay architecture defined in `PROMPT_AGENT_ASV.md`, while keeping the shippable behavior stable and running on both Linux (NUC) and Windows (bench) before competition.

**Architecture:** Strangler-fig migration. Keep the Python/C control core as the operational oracle and safety baseline; introduce Rust WebSocket Gateway/Relay and gradually port C autonomy modules to Rust using golden-vector tests.

**Tech Stack:** Python (runtime glue), C (`core/src` via ctypes), OpenCV (ONNX via cv2.dnn), Rust (`tokio`, `axum`, `tokio-tungstenite`, `mavlink`, `serde`), C++ (V4L2, OpenCV).

**Spec:** `/home/ibnu/Downloads/URDC/usluk-usluk/PROMPT_AGENT_ASV.md`

## Global Constraints

- Firmware: PX4 (per user confirmation).
- Safety: failsafe must be HOLD, not RTL (fix `SafetyManager`). Manual RC/QGC/gamepad disabled in `competition_mode`.
- Gateway must not block autonomy loop; state via watch channels, events via bounded mpsc.
- No panic/unwrap in critical paths; no silent exception swallowing; logging with throttle.
- Tuning params configurable, not hard-coded; competition_mode forbids writes and manual.
- Target run time: <= 20 min mission + 5 setup + 5 clearing.
- Cross-platform: must run on Linux (NUC) and Windows (bench/dev) with `libaterkia.so` / `core/aterkia_core.dll`.

## Review Focus

1. **Failsafe transition:** `request_rtl` -> HOLD correctness; manual path shutdown under `competition_mode`. Test: safety manager behavior with PX4/Pixhawk HOLD command.
2. **Gateway blocking:** autonomy loop must not be blocked by telemetry/network gateway. Test: kill gateway mid-mission and verify control loop timing/state continuity.
3. **PX4 MAVLink command format:** offboard setpoint conversion (thrust/rates vs attitude) must match PX4 expectations.

## Tasks

### Task 1: Cross-Platform Runtime & Single Source of Truth

- [ ] **Step 1: Verify C core builds for Linux (`make -C core linux`) and Windows (`MSYS2 g++ -shared -fPIC`).**
- [ ] **Step 2: Harden `app/aterkia_core.py` loader for Windows (`libaterkia.dll`) vs Linux (`libaterkia.so`).**
- [ ] **Step 3: Verify camera abstraction (`camera_manager.py`) works for Linux V4L2 and Windows DirectShow. Run smoke test on Linux.**

### Task 2: PX4 Failsafe, Competition Mode, & Core Fixes

- [ ] **Step 1: Write failing tests for fuzzy gate gain bug (`fuzzy_gate_gain`).**
- [ ] **Step 2: Fix fuzzy MF edge cases and document valid distance range.**
- [ ] **Step 3: Refactor `SafetyManager` to enforce HOLD failsafe and gate manual/QGC/gamepad when `competition_mode = true`.**
- [ ] **Step 4: Write failing tests for failsafe behavior (HOLD vs RTL).**
- [ ] **Step 5: Fix failsafe logic and add competition_mode guards.**

### Task 3: WebSocket Gateway & Relay (Phase 1)

- [ ] **Step 1: Implement Python/Rust WebSocket telemetry gateway (`app/ws_server.py`).**
- [ ] **Step 2: Implement WebSocket bridge to Relay (`app/ws_bridge.py` or Rust `ws_bridge`).**
- [ ] **Step 3: Update `Web_ASV` frontend to consume relay WebSocket and display telemetry/video.**
- [ ] **Step 4: Add integration tests for WS gateway (start server, connect client, receive telemetry).**

### Task 4: Mission State Management (Behavioral / No-Waypoints)

- [ ] **Step 1: Finalize `app/behavior.py` decision logic for autonomous navigation without GPS waypoints.**
- [ ] **Step 2: Integrate behavior loop into `app/navigator.py` (already drafted, needs hardening).**
- [ ] **Step 3: Add tests for behavioral state transitions (GATE -> BOX -> DOCK).**

### Task 5: Cross-Platform & Deployment Polish

- [ ] **Step 1: Update `requirements.txt` to remove PyQt5, include `websockets`.**
- [ ] **Step 2: Update `README.md` and `setup.sh` for headless WS operation.**
- [ ] **Step 3: Prepare Windows deployment scripts (`scripts/setup_windows.bat`, `scripts/run_gui.bat` adjusted for headless WS).**

### Task 6: Migration Planning for Rust Core (Future Phases 3+)

- [ ] **Step 1: Create Rust migration task breakdown for `core/src/*.c` modules.**

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-11-20-asv-migration-crossplatform.md`. Please review the plan. Does it capture what you want?**

- **Subagent-driven** - A fresh subagent implements each task and a fresh reviewer checks it before the next one starts, then a whole-branch review at the end. Most thorough; costs a fresh context per task and per review.
- **Native** - I implement every task myself in this session, the way this harness runs work, then one fresh reviewer on the most capable model checks the whole branch. Cheapest and fastest; no independent review until the end.

**For this plan I recommend native, because the tasks involve significant cross-language integration (Python-Rust bridge, WebSockets, C) that benefits from a consistent single implementation context to maintain interface stability.** Does the plan capture what you want, and which approach should we use?
