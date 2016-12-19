UDP Socket Demo
========
The UDP socket demo shows how to use the UDP socket interface. Therefore the demo is divided into a server and a client application. The client transmits data periodically to the server including a fixed payload and a sequence counter. The server replies with a fixed pattern and the same sequence number. The size of the packet transmitted by the client is increased per message until a given maximum was reached.
The server the to where the client sends data is defined as the DAG-Root within the network and its IP address is retrieved automatically. The Server just replies to the node it got the packet from.
This demo is mainly used to show how to use the UDP socket interface and to show basic connectivity.

## Configuration
The user can configure this demo by a set of compile time macros. If they are not specified explicitly a default value will be used.
* DEMO_UDP_SOCKET_PORTA: Local port of the server, remote port of the client (default = 4211)
* DEMO_UDP_SOCKET_PORTB: Local port of the client, remote port of the server (default = 4212)
* DEMO_UDP_SEND_INTERVAL: Interval in ms to transmit new data from the client to the server (default = 200ms)
* DEMO_UDP_PKT_LEN_MAX: Maximum size that shall be used when increasing the packet length (default = 40)
