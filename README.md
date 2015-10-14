<img src="https://raw.github.com/hso-esk/_meta/master/emb6.png" width="300">
<img src="https://raw.github.com/hso-esk/_meta/master/HS-Logo_blau_60.png" width="300">

emb::6
========

`emb::6` is a scalable C-based 6LoWPAN stack for embedded devices developed by the
[*Laboratory of Embedded Systems and Communication Electronics (ESK)*](http://ei.hs-offenburg.de/labore/embedded-systems-und-kommunikationselektronik/) at
[*Offenburg University of Applied Sciences (HSO)*](http://www.hs-offenburg.de/) under supervision of *Prof. Dr. Axel Sikora* (axel.sikora@hs-offenburg.de).

Originally derived from Contiki several adaptations have been made such as the
removal of proto-threads. It follows a strict layer based architecture with a
modular software design allowing it to be used even on very restricted devices.


Introduction
------------

In the last decade, IPv6 over Low power Wireless Personal Area Networks, also
known as 6LoWPAN, has well evolved as a primary contender for short range
wireless communication and holds the promise of an Internet of Things, which is
completely based on the Internet Protocol. The IEEE 802.15.4 standard specifies
a maximum frame size of 127 bytes where the IPv6 specification requires a
minimum MTU of 1280 byte. With the 6LoWPAN adaptation layer it is possible to
make use of IPv6 in small and constrained wireless networks which follows the
IEEE 802.15.4 standard. In the meantime, various 6LoWPAN implementations are
available, be it open source or commercial. One of the open source
implementations is the C-based `emb::6` stack.

The `emb::6` stack is optimized to be used in constrained devices without an
operation system. The stack operates event driven with a scalable buffer
handling for optimization on different platforms. The typical field of
application is in wireless sensor networks, e.g. for home automation or
industrial environments.


Getting started
---------------

For the best introduction to get started with `emb::6`, please read the
documentation included. For the code comments you will also find a doxygen
project file within the doc folder.

Features
---------

### Functionality

There are several IOT capable stacks available may it be on open source or on
commercial basis. The `emb::6` networking stack provides several salient features
making it a unique offering. The main features and concepts of the stack are the
following:

* **Event Driven Operation** - Very small RAM overhead, one memory stack for the
whole system.      
* **Scalable Buffer Handling** - A common buffer module is used across layer and
module boundaries. This decreases memory usage and furthermore provides
scalability for usage on different hardware configurations and limitations.
* **Static memory management** - For additional stability during runtime.
* **Compile Time Options** - Usage of different compile-time settings help to
make an optimum selection regarding to the anticipated use cases as well as to
the hardware limitations such as memory size or computing performance.
* **Run Time Options** - Many stack parameters are accessible and changeable
during runtime via remote management.
* **Full IPv6 support** - The integrated IPv6 protocol is based on the
uIP-Stack. This provides full IPv6 support and guarantees further support
and maintenance.
* **BSP and HAL Abstraction** - Hardware dependencies are abstracted with a BSP
which offers an API between the target and the applications and with a hardware
abstraction layer HAL, which allows independence from the used
microcontroller IC.
* **Optimized for use in constrained devices**- The scalability of the stack
enables a manifold use in highly diverse embedded systems.
* **Routing functionality** - The routing functionality is provided by the RPL
protocol.
* **Layered Architecture** - The design of the software stack follows a strict
layered architecture.
* **Simple Setup and Configuration** - The setup and configuration can be easily
executed with the help of centralized configuration files.
* **Socket Interface**- A BSD like socket API allows easy integration of
customized applications.
* **Set of included Application Layer Protocols** - e.g. COAP (further will follow)


### Build System

The buildsystem of `emb::6` is completely based on SCons as a replacement for make
with improved features. SCons is based on Python, makeing the tool very powerful
for the build process. It allows to easily create new configurations e.g. regarding
MCU/Transceiver or application selection. Furhermore the build-time is decreased
significantly.

### Supported Targets

`emb::6` is very platform independent since it has no requirements to an OS and
hardware access is abstracted in a simple "single-file-based" hardware
abstraction layer. That makes it quite easy to port `emb::6` to other platforms.
However `emb::6` comes with support of several platforms by default:


Target | MCU | TRANSCEIVER
-------|-----| ------------
**atany900** | atmega1281 | at86rf212
**atany900pro3** | samd21g18a | at86rf212b
**atany900pro5** | samd20g18 | at86rf212b
**atany900basic** | samd20g18 | at86rf212b
**xpro_212** | samd20j18 | at86rf212
**xpro_212b** | samd20j18 | at86rf212b
**stk3600** | efm32lg990f256 | at86rf212b
**native**  | "linux" | [LCM](https://lcm-proj.github.io/)


Contact
--------

For questions or recommendations don't hesitate to contact emb6@hs-offenburg.de

[1]: http://ei.hs-offenburg.de/labore/embedded-systems-und-kommunikationselektronik/
