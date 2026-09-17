# USV safety1 integration

AI-assisted change; source/host tests do not constitute vehicle acceptance.

- Only `NAV_SCRIPT_TIME(command=1)` changes timeout semantics. It enters HOLD on timeout or companion payload silence (>3 s after starting), rather than advancing the mission. A zero timeout is bounded to 255 seconds for this command. Other script commands retain upstream semantics.
- A matching `USV_DONE` still releases a successful or explicitly skipped sample. A matching `USV_FAIL` cancels it only while AUTO is currently executing that USV sample. Late failures do not override operator-selected MANUAL/RTL. Timeout/link loss also emits `USV_FAIL` to cancel the detector through the ROS bridge.
- Payload/control named values must come from this vehicle's sysid and component 191. Non-finite values, unrelated named values and malformed completion IDs do not refresh the payload watchdog.
- HOLD is not a claim of position hold. Confirm actual hull motion and propulsion behaviour on the specific vehicle.
- Source sysid must match ROS/MAVROS. The current QGC and FCU contract fixes payload component 191. Do not change one endpoint alone.
- There is no persistent cross-reboot transaction identity or end-to-end completion ACK. Stop the whole sampling chain before updating; packet loss must result in a safe hold, not inferred success.

Run `python3 -B tests/test_usv_sampling_safety.py` on Linux/WSL (requires g++), then build Rover for SITL and Pixhawk6C. The host test compiles the actual verifier body with a fake HAL/rover and covers timeout, companion loss and unrelated-script compatibility; it does not execute the full autopilot.

Before deployment test normal completion, explicit SKIP, cancellation, timeout, companion loss, mode-command failure and reboot/ID reuse in SITL and on a restrained bench. Record matched ROS/QGC/detector commits and actual output/mode evidence.

Local-only runtime smoke: `python3 -B tests/check_usv_sitl_sampling.py --binary build/usv-safety-sitl/sitl/bin/ardurover --scenario all`. This creates its own Rover simulator (default instance 178), temporary storage and loopback UDP endpoint; it never connects to an existing vehicle. The six scenarios cover success, cancellation, timeout, companion loss, MANUAL takeover and RTL takeover. Timeout/loss checks also require a same-ID failure notice and no advancement after a late completion. Passing these scenarios is not bench/boat acceptance.
