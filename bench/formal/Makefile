################################################################################
##
## Filename: 	bench/formal/Makefile
##
## Project:	wbspi, a set of Serial Peripheral Interface  cores
##
## Purpose:	Only runs proofs when files have changed
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
##
## Copyright (C) 2020, Gisselquist Technology, LLC
##
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
##
## License:	GPL, v3, as defined and found on www.gnu.org,
##		http://www.gnu.org/licenses/gpl.html
##
##
################################################################################
##
##
.PHONY: all
all: rawslave

rawslave: rawslave_prf/PASS rawslave_cvr/PASS
rawslave_prf/PASS: rawslave.sby ../../rtl/rawslave.v
	sby -f rawslave.sby prf
rawslave_cvr/PASS: rawslave.sby ../../rtl/rawslave.v
	sby -f rawslave.sby cvr

.PHONY: clean
clean:
	rm -rf rawslave_*/
