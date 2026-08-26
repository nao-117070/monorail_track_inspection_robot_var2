# RK39A1A00007 Sensor Library

This library reads rotation angle data for RK39A1A00007 sensors from CAN frames using `BusLink`.

- `RK39A1A00007Sensor` reads raw angle values from frame IDs `baseFrameId + sensorId`.
- `getAngleDegrees()` converts raw 16-bit encoder values to degrees.
