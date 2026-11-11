# Validation

## ROS 2 Jazzy

- Built with colcon in the existing `conduit-slam-local:jazzy` image (Python 3.12).
- All 9 unit tests passed.
- Two synthetic sensor streams received on distinct real DDS topics, checking stamps, frame IDs, SI fields and the unavailable-orientation flag.
- Installed launch entry point started and exited cleanly on SIGINT using a no-device configuration.
- Live AMWS020B auto-discovery and automatic reconnect succeeded. An ROS subscriber received **1,624 samples with every interval equal to 10 ms**. Sensor parser reported zero discarded bytes and zero sensor timestamp gaps.
- The other paired device remained unavailable; its retries did not interrupt the active stream.
- Driver shutdown received and confirmed the device's stopped state (`stop_confirmed=true`).
- Subscriber and driver counts differ slightly at the start/end of the bounded test; this is not a claim of end-to-end delivery of every sample. Best-effort QoS can still lose messages under load.

## ROS 1 Noetic

- Built with catkin in the official `ros:noetic-ros-base` image (Python 3.8).
- `catkin_make run_tests` / `catkin_test_results`: **9 tests, 0 errors, 0 failures, 0 skipped**.
- Two synthetic sensor streams received through the actual ROS 1 graph, checking message stamps, frame IDs, SI fields and orientation covariance.
- Installed roslaunch entry point started and exited cleanly on SIGINT using a no-device configuration.
- Live hardware through the ROS 1 adapter has not been tested; protocol and worker code are shared with the Jazzy branch.

## Regression coverage

Packet fragmentation/coalescing, BCC resynchronization, signed 24-bit SI conversion, midnight rollover and invalid timestamp detection, config validation, device deduplication, discovery-failure isolation, reconnect/shutdown, ROS Imu fields, and delayed measurement-end notifications during stop confirmation.

A test initially exposed a delayed stop event incorrectly rejected by a status query. Both branches include the fix and its regression test. Subscriber buffering was increased to 50 to accommodate Bluetooth bursts; the final hardware run above used that depth.

## Remaining limits

- Two physical devices streaming simultaneously have not yet been tested; only one charged device was available.
- No hardware clock synchronization or long-duration drift evaluation.
- No testing of TSND-series devices or other firmware families.
- Hardware tests used host networking and the host system DBus socket. Docker's default AppArmor profile blocked DBus access on this machine; only the temporary hardware-test container used `apparmor=unconfined`. This is not a requirement for native host operation or a blanket recommendation for deployment. Configure the narrow permissions appropriate to your environment.

No raw measurements or device-specific addresses are included in this repository.
