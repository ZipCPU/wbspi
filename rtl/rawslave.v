////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rawslave.v
//
// Project:	wbspi, a set of Serial Peripheral Interface  cores
//
// Purpose:	This design implements a raw SPI slave.  Typically, were I to
//		implement a raw SPI slave, I'd make some assumptions about
//	the speed of the SPI clock being 3x slower than the system clock, and
//	then I'd synchronize all of the inputs to the system clock.  Not for
//	this design.  This design was created under the assumption that the
//	SPI SCK (clock) signal is up to 2x as fast as the system clock.  This
//	leads to some difficult clock domain crossing issues, solved herein.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2019, Gisselquist Technology, LLC
//
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//

`default_nettype	none
//
//
module rawslave(i_clk, i_reset,
	i_spi_csn, i_spi_sck, i_spi_mosi, o_spi_miso,
	o_we, o_byte, o_rd, i_byte);
	localparam	NFF = 2; // FF's in synchronizer set
	//
	input	wire	i_clk, i_reset;
	//
	// Verilator lint_off SYNCASYNCNET
	input	wire	i_spi_csn;
	// Verilator lint_on  SYNCASYNCNET
	input	wire	i_spi_sck, i_spi_mosi;
	output	wire	o_spi_miso;
	//
	output	reg		o_we;
	output	reg	[7:0]	o_byte;
	output	reg		o_rd;
	input	reg	[7:0]	i_byte;

	////////////////////////////////////////////////////////////////////////
	//
	// SPI clock domain
	//
	reg	[2:0]	spi_bitcount, spi_bitcount_n;
	reg		spi_rdstb, spi_rdreq;
	reg	[7:0]	spi_byte, xck_oreg;
	reg	[6:0]	spi_sreg;
	reg		xck_stb;
	reg	[7:0]	spi_output;

	initial	spi_bitcount = 3;
	always @(posedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_bitcount <= 0;
	else
		spi_bitcount <= spi_bitcount + 1;

	always @(posedge i_spi_sck)
	if (!i_spi_csn)
		spi_sreg <= { spi_sreg[5:0], i_spi_mosi };

	always @(posedge i_spi_sck)
	if (!i_spi_csn && spi_bitcount[2:0] == 3'b111)
		spi_byte <= { spi_sreg[6:0], i_spi_mosi };


	initial	xck_stb = 0;
	always @(posedge i_spi_sck)
	if (spi_bitcount[2:0] == 3'h7)
		xck_stb <= 1'b1;
	else if ((spi_bitcount[2:0] >= 3'h3)&&(spi_bitcount[2:0] < 3'h7))
		xck_stb <= 1'b0;

	//
	// Negative edge of the clock
	//
	//
	initial	spi_bitcount_n = 0;
	always @(negedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_bitcount_n <= 0;
	else
		spi_bitcount_n <= spi_bitcount + 1;

	initial	spi_rdstb = 1;
	always @(negedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_rdstb <= 1;
	else
		spi_rdstb <= (spi_bitcount_n[2:0] == 3'h7);

	initial	spi_rdreq = 1;
	always @(negedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_rdreq <= 1;
	else if (spi_rdstb)
		spi_rdreq <= 1;
	else
		spi_rdreq <= (spi_bitcount_n[2:0] <= 3'h3);


	// Output data can only change on the negative edge of the clock
	initial	spi_output = 0;
	always @(negedge i_spi_sck)
	if (spi_rdstb)
		spi_output <= xck_oreg;
	else
		spi_output <= spi_output << 1;

	////////////////////////////////////////////////////////////////////////
	//
	// System clock domain
	//
	reg			sync_spi_rd, sync_spi_stb;
	reg	[NFF-1:0]	sync_rd_pipe, sync_stb_pipe;
	reg			pre_stb;

	//
	// Clock synchronizers
	initial { sync_spi_rd, sync_rd_pipe } = -1;
	always @(posedge i_clk)
	if (i_reset)
		{ sync_spi_rd, sync_rd_pipe } <= { 1'b0, {(NFF){1'b1}} };
	else begin
		sync_rd_pipe <= { sync_rd_pipe[NFF-2:0], spi_rdreq };
		sync_spi_rd <= !sync_rd_pipe[NFF-1] && sync_rd_pipe[NFF-2];
	end

	//
	//
	initial	pre_stb = 0;
	initial { sync_spi_stb, sync_stb_pipe } = -1;
	always @(posedge i_clk)
	if (i_reset)
	begin
		{ sync_spi_stb, sync_stb_pipe } <= -1;
		pre_stb <= 1'b0;
	end else begin
		{ sync_spi_stb, sync_stb_pipe }
			<= { sync_stb_pipe, xck_stb };
		pre_stb <= sync_stb_pipe[1] && !sync_spi_stb;
	end

	//
	initial	o_we = 0;
	always @(posedge i_clk)
		o_we <= pre_stb && !i_reset;

	always @(posedge i_clk)
	if (pre_stb)
		o_byte <= spi_byte;

	//
	// oreg -- The output data register
	//
	initial	xck_oreg = 0;
	always @(posedge i_clk)
	if (i_reset)
		xck_oreg <= 8'h0;
	else if (sync_spi_rd)
		xck_oreg <= i_byte;
	// else if (o_we)
	//	xck_oreg <= i_byte;

	////////////////////////////////////////////////////////////////////////
	assign	o_spi_miso = spi_output[7]; // (!i_spi_csn) ? spi_output[7] : 1'bz;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////
//////	Formal properties
//////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
	localparam	F_CKBITS=3;

	////////////////////////////////////////////////////////////////////////
	//
	//	Generate some (assumed) clocking signals
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	(* anyconst *)	reg [F_CKBITS-1:0]	f_sysck_step,
						f_spick_step;
	reg [F_CKBITS-1:0]	f_sysck_counter, f_sysck_next,
				f_spick_counter, f_spick_next;
	reg			f_will_rise_clk, f_will_rise_sck;

	(* gclk *)	reg	gbl_clk;

	always @(*)
	begin
		assume(f_sysck_step > 0);
		assume(f_sysck_step <= { 1'b1, {(F_CKBITS-1){1'b0}} });

		assume(f_spick_step > 0);
		assume(f_spick_step <= { 1'b1, {(F_CKBITS-1){1'b0}} });

		// assume(f_sysck_step >  (f_spick_step >> 3));
		assume(f_sysck_step >  (f_spick_step >> 1));
	end

	always @(posedge gbl_clk)
	begin
		f_sysck_counter <= f_sysck_counter + f_sysck_step;
		f_spick_counter <= f_spick_counter + f_spick_step;
	end

	always @(*)
	begin
		assume(i_clk == f_sysck_counter[F_CKBITS-1]);

		if (!i_spi_csn)
			assume(i_spi_sck == f_spick_counter[F_CKBITS-1]);
	end

	always @(posedge gbl_clk)
	if ($fell(i_spi_csn))
		assume(i_spi_sck);

	always @(*)
	begin
		f_sysck_next = f_sysck_counter + f_sysck_step;
		f_spick_next = f_spick_counter + f_spick_step;

		f_will_rise_clk = !i_clk && f_sysck_next[F_CKBITS-1];
		f_will_rise_sck = !i_spi_sck && f_spick_next[F_CKBITS-1];
	end

	////////////////////////////////////////////////////////////////////////
	//
	//	f_past_valid
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	f_past_valid_gbl, f_past_valid_spi, f_past_valid_clk;

	initial f_past_valid_gbl = 1'b0;
	always @(posedge gbl_clk)
		f_past_valid_gbl <= 1'b1;

	initial f_past_valid_spi = 1'b0;
	always @(posedge i_spi_sck)
		f_past_valid_spi <= 1'b1;

	initial f_past_valid_clk = 1'b0;
	always @(posedge i_clk)
		f_past_valid_clk <= 1'b1;

	always @(*)
		assume(i_reset == !f_past_valid_clk);

	////////////////////////////////////////////////////////////////////////
	//
	//	Basic SPI protocol rules
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	f_past_csn, f_past_sck, f_clkd_while_idle;
	wire	f_active, f_fault;
	reg	[2:0]	f_idle_count;

	//
	// Always start de-selected
	always @(*)
	if (!f_past_valid_gbl)
		assume(i_spi_csn);

	initial	{ f_past_csn, f_past_sck } = 2'b11;
	always @(posedge gbl_clk)
		{ f_past_csn, f_past_sck } <= { i_spi_csn, i_spi_sck };

	assign	f_active = !i_spi_csn && !f_past_csn;
	assign	f_fault  =  i_spi_csn && !f_past_sck;

	initial	f_idle_count = -1;
	always @(posedge f_spick_counter[F_CKBITS-1])
	if (!i_spi_csn)
		f_idle_count <= 0;
	else if (!(&f_idle_count))
		f_idle_count <= f_idle_count + 1;

	always @(posedge gbl_clk)
	if (f_sysck_step >= f_spick_step)
	begin
		if (f_idle_count < 3'h4)
			assume(!$fell(i_spi_csn));
	end else if (f_idle_count < 3'hd)
		assume(!$fell(i_spi_csn));

	//
	// When i_spi_csn falls (i.e. activates), the clock must be stable
	// and *high*
	always @(*)
	if (i_spi_csn || f_past_csn)
		assume(i_spi_sck);

	//
	// The incoming data line from the master changes on a falling edge
	// *only*.
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && f_active && !$fell(i_spi_sck) && !$fell(i_spi_sck))
		assume($stable(i_spi_mosi));

	//
	// The outgoing data line changes on a rising edge *only*.  However,
	// any time i_spi_csn is high, it's a don't care.
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && !i_spi_csn && $past(!i_spi_csn)
			&& !$past(i_spi_sck) && !$rose(i_spi_sck))
		assert($stable(o_spi_miso));

	initial	f_clkd_while_idle <= 1'b1;
	always @(posedge gbl_clk)
	if (!i_spi_csn)
		f_clkd_while_idle <= 1'b0;
	else if ($rose(i_clk))
		f_clkd_while_idle <= 1'b1;

	always @(posedge gbl_clk)
	if (!f_clkd_while_idle && f_past_csn)
		assume(i_spi_csn);

	////////////////////////////////////////////////////////////////////////
	//
	//	Synchronous Input assumptions
	//
	////////////////////////////////////////////////////////////////////////
	//
	//

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && !$rose(i_clk))
	begin
		assume($stable(i_reset));
		assume($stable(i_byte));
		assert($stable(o_we));
		assert($stable(o_byte));
	end

	////////////////////////////////////////////////////////////////////////
	//
	//	Basic CDC rules
	//
	////////////////////////////////////////////////////////////////////////
	//
	//

	// If ever pre_stb is rises, spi_byte must be stable
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && $rose(pre_stb) && !f_fault)
		assert($stable(spi_byte));

`ifdef	VERIFIC
	always @(*)
	case({ sync_spi_stb, sync_stb_pipe })
	3'b000: begin end
	3'b001: begin end
	3'b011: begin end
	3'b100: begin end
	3'b110: begin end
	3'b111: begin end
	default: assert(0);
	endcase
`endif
	reg	i_reset_clk;
	initial	i_reset_clk = 1'b1;
	always @(posedge i_clk)
		i_reset_clk <= i_reset;

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && !i_spi_csn && $rose(i_spi_sck)
			&& ((spi_bitcount == 0) || spi_rdstb)
			&& !$past(i_reset_clk))
		assert($stable(xck_oreg));

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && !i_spi_csn && $rose(spi_rdstb))
		assert($stable(xck_oreg));

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && f_active && spi_rdstb)
		assert($stable(xck_oreg));

	////////////////////////////////////////////////////////////////////////
	//
	//	Receive sequence checking
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[8:0]	f_rxseq;
	reg	[3:0]	f_rxcdc;
	reg	[7:0]	f_data, f_rcvd;

	initial	f_rxseq = 1;
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && f_will_rise_sck)
	begin
		f_rxseq <= f_rxseq << 1;
		if (f_rxseq[7])
			f_rxseq[0] <= 1;
		if (f_rxseq[8] && !f_will_rise_clk)
			f_rxseq[8] <= 1;
		assert(f_rxseq != 0);
		f_data <= { f_data, i_spi_mosi };
		if (f_rxseq[7])
			f_rcvd <= { f_data[6:0], i_spi_mosi };
	end else if (i_spi_csn)
	begin
		f_rxseq[8:0] <= 1;
		if (f_rxseq[8] && !f_will_rise_clk)
			f_rxseq[8] <= 1;
	end else if (f_will_rise_clk && f_rxseq[8])
		f_rxseq[8] <= 0;

	always @(*)
	if (!i_spi_csn)
	begin
		if (f_rxseq[1])
			assert(spi_bitcount == 1);
		if (f_rxseq[2])
			assert(spi_bitcount == 2);
		if (f_rxseq[3])
			assert(spi_bitcount == 3);
		if (f_rxseq[4])
			assert(spi_bitcount == 4);
		if (f_rxseq[5])
			assert(spi_bitcount == 5);
		if (f_rxseq[6])
			assert(spi_bitcount == 6);
	end

	initial	f_rxcdc = 0;
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && f_will_rise_clk)
	begin
		f_rxcdc <= f_rxcdc << 1;
		if (f_rxseq[8])
		begin
			f_rxcdc[0] <= 1;
		end
	end

	always @(*)
	if (f_rxseq[8])
		assert(xck_stb);

	always @(*)
	if (f_rxcdc[0])
		assert((sync_stb_pipe == 2'b01) && !pre_stb && !o_we);
	else if (f_rxcdc[1])
		assert((sync_stb_pipe == 2'b11) && !pre_stb && !o_we);
	else if (f_rxcdc[2])
		assert( pre_stb && !o_we);
	else if (f_rxcdc[3])
		assert( !pre_stb &&  o_we);
	else
		assert(!pre_stb && !o_we && sync_stb_pipe != 2'b01);

	always @(posedge gbl_clk)
	if (|f_rxcdc)
	begin
		assert($stable(spi_byte));
		assert(f_rcvd == spi_byte);
	end

	always @(posedge gbl_clk)
	if (pre_stb)
	begin
		assert($stable(f_rcvd));
		assert(f_rcvd == spi_byte);
	end

	always @(*)
		cover(f_rxseq[9] && !i_spi_csn);

	always @(*)
		assume(!f_fault);

	////////////////////////////////////////////////////////////////////////
	//
	//	Transmitter sequence checking
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg		f_pending_rdstb;
	reg	[7:0]	f_txmit;

	// sync_spi_csn || o_rd
	// Cross clock domains
	// spi_rd_stb
	// Bits out

	/*
	initial	f_pending_rdstb <= 1'b0;
	always @(posedge gbl_clk)
	if (f_will_rise_clk && o_rd)
	begin
		f_pending_rdstb <= 1'b1;
		f_txmit <= i_byte;
	end else if (i_spi_csn && spi_rdstb && f_will_rise_sck)
		f_pending_rdstb <= 1'b0;

	// always @(*)
	// if (!f_pending_rdstb && !i_spi_csn)
	//	assert(!spi_rdstb);

	always @(posedge gbl_clk)
	if (f_pending_rdstb && $past(f_pending_rdstb) && !i_spi_csn && !spi_rdstb)
	begin
		assert($stable(f_txmit));
		assert(f_txmit == xck_oreg);
	end

	always @(posedge gbl_clk)
	if ($rose(i_spi_sck) && $fell(spi_rdstb))
		assert(spi_output <= xck_oreg);
	*/
	////////////////////////////////////////////////////////////////////////
	//
	reg	f_pending_wrstb;

	always @(posedge i_clk)
	if (f_past_valid_clk && $past(o_we))
		assert(!o_we);

	always @(posedge i_clk)
	if (f_past_valid_clk && $past(pre_stb))
		assert(o_we && !pre_stb);

	//
	// Insist that, for every rise of the xck_stb, that there's one and
	// only one corresponding strobe on the system clock frequency
	initial	f_pending_wrstb = 1'b0;
	always @(posedge gbl_clk)
	if ($rose(xck_stb))
		f_pending_wrstb <= 1'b1;
	else if (o_we)
		f_pending_wrstb <= 1'b0;


	always @(posedge gbl_clk)
	if(f_pending_wrstb)
	begin
		assert($stable(spi_byte));
		assert(xck_stb || sync_stb_pipe[1] || o_we);
	end else
		assert(!pre_stb);

	always @(posedge i_clk)
	if(f_pending_wrstb && $past(sync_stb_pipe == 2'b11))
		assert(o_we || pre_stb);

	////////////////////////////////////////////////////////////////////////
	//
	//	Cover properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(*)
		cover(o_we && o_byte == 8'hda);
`endif
endmodule
