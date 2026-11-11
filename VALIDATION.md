# Validation

## Automated checks

- ROS 2 Jazzy: colcon build succeeded using an existing Jazzy image.
- Eight tests passed: packet fragmentation/coalescing, BCC recovery, signed SI conversion, midnight rollover and invalid clock detection, config validation, device deduplication, discovery failure isolation, reconnect/shutdown, and Imu message fields (some tests cover multiple cases).
- ROS 2 topic smoke test: two synthetic sensor streams received on distinct real DDS topics; frame IDs, SI fields and timestamps checked.

## Hardware scope

The precursor Bluetooth acquisition tool received 1,192 contiguous samples at 100 Hz from an AMWS020B and confirmed measurement stop. Another AMWS020B returned low-battery stop code 3. Those checks establish the protocol path, not validation of simultaneous ROS acquisition.

The ROS driver's live hardware reception is pending: both devices timed out during the latest read-only readiness check. Multi-device reconnect is tested using injected transports; simultaneous live hardware operation remains unverified until devices are available.
