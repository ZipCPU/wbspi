## Files

- [rawslave.v](rawslave.v): A raw SPI slave.  Notable for the fact that it
  allows the SPI interface to operate on one clock (SCK) and the rest of the
  design on another.  As a result, SCK can run (slightly) faster than the
  rest of the design.

- [spicpu.v](spicpu.v): A memory controlled SPI master.  This has been
  designed for controlling a telemetry stream.  Once configured with
  a memory address for its script, and possibly a (repeating) interrupt
  for a sync, this can run its script repeatedly outputting its results to an
  AXI stream.

## Status

Neither of these designs have ever seen the light of hardware.
