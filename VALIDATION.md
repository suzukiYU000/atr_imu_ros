# Validation

## ROS 2 Jazzy

- Built with colcon in the existing `conduit-slam-local:jazzy` image (Python 3.12).
- All 13 unit tests passed after the TSND151 extension.
- Two synthetic sensor streams received on distinct real DDS topics, checking stamps, frame IDs, SI fields and the unavailable-orientation flag.
- Installed launch entry point started and exited cleanly on SIGINT using a no-device configuration.
- Live AMWS020B auto-discovery and automatic reconnect succeeded. An ROS subscriber received **1,624 samples with every interval equal to 10 ms**. Sensor parser reported zero discarded bytes and zero sensor timestamp gaps.
- The other paired device remained unavailable; its retries did not interrupt the active stream.
- Driver shutdown received and confirmed the device's stopped state (`stop_confirmed=true`).
- Subscriber and driver counts differ slightly at the start/end of the bounded test; this is not a claim of end-to-end delivery of every sample. Best-effort QoS can still lose messages under load.

## ROS 1 Noetic

- Built with catkin in the official `ros:noetic-ros-base` image (Python 3.8).
- `catkin_make run_tests` / `catkin_test_results`: **13 tests, 0 errors, 0 failures, 0 skipped**.
- Two synthetic sensor streams received through the actual ROS 1 graph, checking message stamps, frame IDs, SI fields and orientation covariance.
- Installed roslaunch entry point started and exited cleanly on SIGINT using a no-device configuration.
- Live hardware through the ROS 1 adapter has not been tested; protocol and worker code are shared with the Jazzy branch.

## Regression coverage

Packet fragmentation/coalescing, BCC resynchronization, signed 24-bit SI conversion, midnight rollover and invalid timestamp detection, config validation, device deduplication, discovery-failure isolation, reconnect/shutdown, ROS Imu fields, and delayed measurement-end notifications during stop confirmation.

A test initially exposed a delayed stop event incorrectly rejected by a status query. Both branches include the fix and its regression test. Subscriber buffering was increased to 50 to accommodate Bluetooth bursts; the final hardware run above used that depth.

## Remaining limits

- Three physical devices produced concurrent ROS 2 data; TSND151 reliability remains limited as described below.
- No hardware clock synchronization or long-duration drift evaluation.
- TSND151 live samples were received, but timestamp gaps and stop-response timeout were observed. TSND121 and other firmware families are untested.
- Hardware tests used host networking and the host system DBus socket. Docker's default AppArmor profile blocked DBus access on this machine; only the temporary hardware-test container used `apparmor=unconfined`. This is not a requirement for native host operation or a blanket recommendation for deployment. Configure the narrow permissions appropriate to your environment.

No raw measurements or device-specific addresses are included in this repository.

## TSND151 extension

- Paired and trusted a TSND151. Read its actual model, serial, command-mode state and battery response successfully.
- Verified the exposed Serial Port service uses channel 1 (the AMWS020B devices use channel 2).
- Added paired `TSND151-*` / `TSND151_*` discovery and verified protocol/model recognition against the manufacturer's 2016-06-28 command manual.
- Autodiscovered devices use these verified model channel defaults; explicit devices may override `channel`, or use SDP resolution if no channel is specified.
- A TSND151 dropped the first information request immediately after SPP connect. Retrying the same read-only request on the same socket succeeded. The driver now retries that read-only handshake once; mutating commands are not blindly repeated.
- Added tests for mixed-model discovery, channel resolution and ambiguous services, TSND model recognition, and the initial handshake retry. Both branches now pass 13 tests.
- Subsequent live TSND151 attempts failed with Linux `EALREADY` (Operation already in progress), including an isolated host-side connection after the sensor was power-cycled. This is unresolved; do not interpret protocol support as a successful TSND live-stream test.
- During the final Jazzy run, another AMWS020B produced 1,374 subscriber samples, all at 10 ms intervals; parser and sensor timing errors were zero, and device stop was confirmed. TSND151 produced no samples in that run.

## Three-device hardware session (2026-11-11)

- After power-cycling the host Bluetooth adapter, ROS 2 Jazzy received concurrent streams from two AMWS020B devices and one TSND151. Connection establishment was serialized in this diagnostic run only; the production worker implementation was unchanged. Serialization alone had not resolved the preceding connection errors.
- During the 45-second window, AMWS020B subscribers received 4,207 and 4,054 samples respectively, with every received timestamp interval equal to 10 ms. Both sensor workers reported zero parser discards, zero timestamp gaps, and confirmed stop.
- TSND151 delivered 1,829 samples, with 27 sensor timestamp gaps and a maximum subscriber timestamp interval of 2,730 ms. Its parser discarded zero bytes, but the shutdown stop response timed out. Three-device reception is verified; reliable 100 Hz delivery and confirmed stop for TSND151 are not.
- Bluetooth adapter power cycling was a manual local recovery, not a driver action. It interrupts Bluetooth connections and is not automatically performed by the package.
