# Telemetry component

This component allows "streams" of telemetry data to be established. A stream is data that is sent over a TCP connection.

Once a client is connected to the WiFi access point of the device (microslam_AP), it can connect to 192.168.4.1:42601 to receive telemetry data.

Note - for simplicity currently only a single connection is allowed. If a second client tries to connect to the same port, it will be refused.

## Data format

The data is serialised using the functions defined in [../../../slam/include/slam/serialisation.h](../../../slam/include/slam/serialisation.h) and
these same functions can be used to deserialise the data on the client side.

Different types of data are sent over the stream, and each type is identified by a unique ID in the header of the message.
The IDs are defined in the header file linked above.

