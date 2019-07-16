## (Wishbone accessible) SPI Cores

This repository will eventually contain some SPI cores with a Wishbone
interface.

For now, it only contains a [SPI slave with a raw interface](rtl/rawslave.v).
This slave core will send signals, `o_frame` at the end of a SPI transaction,
`o_rd` when the core wishes to read a byte from the local interface that will
then be sent over MISO, and `o_wr` when it wishes to write a byte to the local
interface once it has been read from the MOSI channel.  The first byte sent
over `MISO` is always undefined.

This core was really designed to be high speed.  By that I mean that the slave
can handle any SPI clock relationship to the system clock up to just less than
33% faster than the system clock.  A sad consequence of this is that it might
be difficult to reply to an incoming data request without losing an additional
8-bits of `MISO`.  So ... make certain this works in your environment before
and if you should try to use it.

Feel free to read more information about this core in the documentation of the
core itself.

## License

The core as written is licensed under GPL.  My eventual goal is to provide a
more LGPL-type license that's more appropriate to hardware, but I'm still
looking for such.  Feel free to contact me and let me know if this license
is not sufficient for you and we can discuss other options.
