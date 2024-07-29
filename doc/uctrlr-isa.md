# SPI MicroController ISA

## Instructions

All instructions are 8bits.  Some instructions, such as `SEND` or `TXRX` will
be followed by subsequent bytes containing values to be sent over the interface.

- **START #**: Activates the CS# pin, deactivating all other pins.  Attempts
  to activate a non-existent device will quietly be treated as STOP commands.

  Note that there is no means of activating multiple CS# signals at once.

- **STOP**: Deactivates all CS# pins.  This is encoded as a START command with
  an all ones argument--i.e., the maximum device number.

- **READ #**: Reads `#` octets from the SPI interface, and forwards those
  octets to the AXI stream.  This will involve clocking the SPI interface `8*#`
  times.  The minimum read is 1 octet, and the maximum is 16.  The actual
  encoding is off by one, so that 0-15 encodes a read of between 1-16 values,
  even though the assembler will attempt to hide this reality.  The value sent
  across the SPI interface during this time is a don't care.  If the value on
  the MOSI pin is important, use a TXRX command.

- **SEND #,V1**,V2,V3,...: Sends between 1 and 16 bytes across the SPI
  interface, clocking it `8*#` times in the process.  The values transmitted
  are given as arguments to the command in subsequent bytes: V1, V2, V3, etc.
  As with the read, the minimum number of items to send is one and the maximum
  number is 16.  Likewise with the read, the encoded number of items to be sent
  is off by one.  Any value(s) returned on the MISO pin during these commands
  will be ignored.

- **TXRX #,V1**,V2,V3,...: This is a combination of the READ and SEND commands.
  Values to be sent are provided as arguments to this command.  Values read
  from the device will be fed to the AXI stream (external) interface.

- **LAST**: Sets an internal "LAST" flag.  If set, the following READ or TXRX
  command will set the AXI stream LAST flag on the last beat of the read.

- **HALT**: Halts the script.  Sets a CPU interrupt.

- **WAIT**: Deactivates all CS# signals and halts the processing of further
  commands.  Commands will begin again following the next synchronization
  signal.

- **TICK**: Issues a single clock tick to the SPI interface.  The return value
  is ignored.

  The argument is reserved for either a number of clock ticks, or the value
  to be sent over MOSI, or ... something still to be determined.

- **TARGET**: Sets the address of where to jump to.

  This permits simple looping.  More complicating looping will require CPU
  intervention. 

- **JUMP**: Jumps to the address of the last `TARGET` command.

- **CHAN #**: Specifies that any bytes read following this command will be
  forwarded to the given AXI stream channel.  (`TID` will be set to `#`.)
  Only relevant if/when the number of TID bits is non-zero.  In all other
  cases, this command operates as a NOOP.

- **NOOP**: Does nothing.

