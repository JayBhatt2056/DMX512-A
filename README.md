 The goal of this project is to build a compound device capable of acting as a theatrical lighting controller 
or a controller theatrical lighting device, based on the DMX512-A protocol.
 The PC transmitter will accept commands from a PC via an virtual COM and will continuously transmit a 
serial stream to control up to 512 devices on a RS-485 communication bus.  The PC receiver will forward 
data received from devices on a communications bus and send these to the PC over a virtual COM port.
 The design also supports the transmission of deferred actions on the DMX512A bus to support 
unattended operations.
 Devices on the bus will extract information out of the asynchronous data stream and will control one or 
more devices.  They will also send an acknowledgment to the controller when requested.
