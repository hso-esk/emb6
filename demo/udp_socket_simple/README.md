UDP Simple Socket Demo
========
This Demo shows how to use the simple UDP socket interface. Therefore the demo is divided into a server and a client application. The client transmits data periodically to the server including a fixed payload and a sequence counter. The server replies with the same sequence number.
The demo makes use of the simplified Berkley Sockets alike interface to transmit and receive the data.
The server is defined as the DAG-Root within the network and its IP address is retrieved automatically. The Server just replies to the node it got the packet from.
This demo is mainly used to show how to use the UDP socket interface and to show basic connectivity.

## Configuration
The user can configure this demo by a set of compile time macros. If they are not specified explicitly a default value will be used.
* DEMO_UDP_SIMPLE_PORT: Port to use for the demo (default = 45287)
* DEMO_UDP_SOCKET_SIMPLE_SEND_INTERVAL: Interval in ms to transmit new data from the client to the server (default = 1000ms)
