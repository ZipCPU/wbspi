# SPI CPU Assembler

The main component of this software directory is the SPI CPU assembler.  This
assembler takes files, such as [testfil.s](testfil.s), containing
assembler commands, and it generates a binary file which can then be fed to
command the [SPICPU](../rtl/spicpu.v).

Assembler commands are:

- STOP will cause the SPI controller to raise CSN, and thereby clear the
  interface

- START <ID>: Will cause the SPI controller to lower CSN[ID].  ID's can range
  from 0 to 30, subject to the hardware the controller was built to support.
  If ID is greater than the number of chip enables supported by the SPI
  controller hardware, then this instruction becomes the equivalent of a
  STOP command and will raise all CSN lines.

- READ <NMBR>: Reads NMBR bytes from the SPI interface.  The outgoing MOSI
  lines will be set to zero during this time.  All bytes will be sent to the
  outgoing stream port.  If given sufficient backpressure, the SPI clock
  and hence the whole interface will stall.

- SEND <IMM> [, <IMM>]`*`: Sends the given bytes across the SPI interface.
  Any return values contained in the MISO lines will be ignored, and the
  outgoing AXI stream interface will thus become idle.  Immediate values
  can be of any form accepted by `strtoul()`.

- TXRX <IMM> [, <IMM>]`*`: This is the same as the `SEND` comand above, save
  that the receive values will be sent across the outgoing AXI stream interface
  instead of being ignored.

- LAST: This command is issued as a prefix command to any `READ` or `TXRX`
  commands.  If issued, then the last value read as part of those commands
  will have the TLAST flag set, to indicate that said value is the last value
  in a packet.

Those are the commands that will be used to control the SPI interface.  Another
4 commands exist as well, which are handled prior to the SPI interface:

- HALT: Once received, no further commands will be issued to the SPI
  interface without CPU intervention.  This instruction implies a STOP if CSn
  has not been deactivated.

- WAIT: Will pause all instructions to the SPI interface until an external
  synchronization signal has been received.  This instruction also implies a
  STOP if CSn has not yet been deactivated.

- TARGET: Sets the address for a future JUMP instruction to return to.

  The SPI controller does not support conditional jumps or halts.  Therefore,
  it can only support one of two control structures: Run from a start to a
  completion, or run from a start to a `TARGET` command followed by an infinite
  loop from the last `TARGET` command to the final `JUMP` command.

- JUMP: This is the other half of the `TARGET` loop structure.  Once `JUMP`
  is received, the CPU will `JUMP` to the `TARGET` instruction.  If CSn is
  active at this time, then this instruction will also deactivate all CSn
  signals (i.e. raise them).

Note that all logic is address independent: any jump address is defined by
the location of the last `TARGET` instructions.  This is designed to make it
easy to install the script anywhere in memory at a later time.

## Testing

(Current test comes from a different project.)

<!--
A test of the SPI assembler is provided.  The usage of this assembler can be
found via the `-h` option:

> spiasm -h

To test, use the assembler to build a binary:

> spiasm testfil.s -o dump.bin

You can then disassemble this file to see how well the assembler worked.

> spiasm -d dump.bin

-->
