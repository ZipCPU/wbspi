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
// Interface:
//
//	Upon receiving 8-bits from the MOSI pin on the SCK, the data will be
//	transferred from the SPI clock domain to a surrounding system clock
//	domain.  Once done, the o_we signal will go high, and o_byte will
//	contain that byte.
//
//	Any time the SPI interface becomes idle (CSN goes inactive/high), the
//	o_frame signal will be set--allowing interacting software to know
//	when the frame is starting.
//
//	o_frame will remain set until o_rd goes high.  The i_byte value present
//	when o_rd is high will be sent to the SPI port.  It will not be the
//	first 8-bits sent, but rather the second.  To avoid metastability
//	issues, this value can only be changed when o_rd is high.
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
	o_frame, o_we, o_byte, o_rd, i_byte);
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
	output	reg		o_frame;
	output	reg		o_we;
	output	reg	[7:0]	o_byte;
	output	reg		o_rd;
	input	reg	[7:0]	i_byte;

	////////////////////////////////////////////////////////////////////////
	//
	// SPI clock domain
	//
	reg	[2:0]	spi_bitcount;
	reg	[3:0]	spi_bitcount_n;
	reg		spi_rdreq, spi_frame;
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
	// This is where the master reads from the SPI port
	//
	initial	spi_bitcount_n = 0;
	always @(negedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_bitcount_n <= 0;
	else if (!spi_bitcount_n[3])
		spi_bitcount_n <= spi_bitcount_n + 1;
	else
		spi_bitcount_n[2:0] <= spi_bitcount_n[2:0] + 1;

	initial	spi_rdreq = 0;
	always @(negedge i_spi_sck)
	if (spi_bitcount_n[2:0] < 3'h4)
		spi_rdreq <= 1;
	else
		spi_rdreq <= 0;


	// Output data can only change on the negative edge of the clock
	initial	spi_output = 0;
	always @(negedge i_spi_sck, posedge i_spi_csn)
	if (i_spi_csn)
		spi_output <= 0;
	else if (spi_bitcount_n[2:0] == 3'h7)
		spi_output <= xck_oreg;
	else
		spi_output <= spi_output << 1;

	////////////////////////////////////////////////////////////////////////
	//
	// System clock domain
	//
	reg			sync_spi_rd, sync_spi_stb, sync_spi_frame;
	reg	[NFF-1:0]	sync_rd_pipe, sync_stb_pipe, sync_frame_pipe;
	reg			pre_stb;

	//
	// Clock synchronizers
	initial { sync_spi_rd, sync_rd_pipe } = 0;
	always @(posedge i_clk)
	if (i_reset)
		{ sync_spi_rd, sync_rd_pipe } <= { 1'b0, {(NFF){1'b0}} };
	else begin
		{ sync_spi_rd, sync_rd_pipe }
			<= { sync_rd_pipe[NFF-1:0], spi_rdreq&&!i_spi_csn };
	end

	always @(*)
		o_rd = !sync_spi_rd && sync_rd_pipe[NFF-1];

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
		pre_stb <= sync_stb_pipe[NFF-1] && !sync_spi_stb;
	end

	initial { sync_spi_frame, sync_frame_pipe } = -1;
	always @(posedge i_clk)
	if (i_reset)
	begin
		{ sync_spi_frame, sync_frame_pipe } <= -1;
	end else begin
		{ sync_spi_frame, sync_frame_pipe }
			<= { sync_frame_pipe, i_spi_csn };
	end

	initial o_frame = 1'b1;
	always @(posedge i_clk)
	if (i_reset)
		o_frame <= 1'b1;
	else if (o_rd)
		o_frame <= 0;
	else if (!sync_spi_frame && sync_frame_pipe[NFF-1])
		o_frame <= 1;

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
	always @(*)
		xck_oreg = i_byte;

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
	localparam	F_CKBITS=5;
	localparam	F_CK_MIN_CSN_TO_CSN   = 4'h8;
	localparam	F_CK_MIN_CSN_RECOVERY = 4'h3;
	localparam	F_CK_MIN_CSN_ACTIVE   = 4'h2;


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
	reg			f_will_rise_clk, f_will_rise_sck,
				f_will_fall_sck;

	(* gclk *)	reg	gbl_clk;

	always @(*)
	begin
		assume(f_sysck_step > 0);
		assume(f_sysck_step <= { 1'b1, {(F_CKBITS-1){1'b0}} });

		assume(f_spick_step > 0);
		assume(f_spick_step <= { 1'b1, {(F_CKBITS-1){1'b0}} });

		// assume(f_sysck_step >  (f_spick_step >> 3));
		//
		// If the SPI clock is too fast, we cannot support an 8b
		// interface
		// assume(f_sysck_step >  f_spick_step - (f_spick_step>>3));
		assume(f_sysck_step >  (f_spick_step >> 1)
					+ (f_spick_step >> 2));

		// While the following isn't strictly necessary, it helps
		// keep the proof moving along
		assume(f_sysck_step >= { 3'b001, {(F_CKBITS-3){1'b0}}});
		// assume(f_spick_step > { 3'b001, {(F_CKBITS-3){1'b0}}});
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
		f_will_fall_sck = !i_spi_csn && i_spi_sck && !f_spick_next[F_CKBITS-1];
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
	reg	[3:0]	f_csn_hi_count, f_csn_to_csn_count,
			f_csn_lo_count;

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

	initial	f_csn_to_csn_count = -1;
	always @(posedge gbl_clk)
	if ($fell(i_spi_csn))
		f_csn_to_csn_count <= 0;
	else if ($rose(i_clk) && (!(&f_csn_to_csn_count)))
		f_csn_to_csn_count <= f_csn_to_csn_count + 1;

	initial	f_csn_hi_count = -1;
	always @(posedge gbl_clk)
	if (!i_spi_csn)
		f_csn_hi_count <= 0;
	else if ($rose(i_clk) && (!(&f_csn_hi_count)))
		f_csn_hi_count <= f_csn_hi_count + 1;

	initial	f_csn_lo_count = 0;
	always @(posedge gbl_clk)
	if (i_spi_csn)
		f_csn_lo_count <= 0;
	else if ($rose(i_clk) && (!(&f_csn_lo_count)))
		f_csn_lo_count <= f_csn_lo_count + 1;

	always @(posedge gbl_clk)
	if (f_csn_to_csn_count <= F_CK_MIN_CSN_TO_CSN)
		assume(!$fell(i_spi_csn));

	always @(posedge gbl_clk)
	if (f_csn_hi_count <= F_CK_MIN_CSN_RECOVERY)
		assume(!$fell(i_spi_csn));

	always @(posedge gbl_clk)
	if (f_csn_lo_count <= F_CK_MIN_CSN_ACTIVE)
		assume(!$rose(i_spi_csn));

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

	always @(posedge gbl_clk)
	if ($fell(i_spi_csn))
		assert(o_frame);

	always @(*)
	if (sync_spi_frame)
		assert(o_frame);

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
		//
		// Synchronous output assertions
		assert($stable(o_we));
		assert($stable(o_byte));
		assert($stable(o_frame));
		assert($stable(o_rd));
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
	if (f_past_valid_gbl && spi_bitcount_n[2:0] == 3'h7)
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
		begin
			assert(spi_bitcount == 1);
			assert(f_data[0] == spi_sreg[0]);
		end
		if (f_rxseq[2])
		begin
			assert(spi_bitcount == 2);
			assert(f_data[1:0] == spi_sreg[1:0]);
		end
		if (f_rxseq[3])
		begin
			assert(spi_bitcount == 3);
			assert(f_data[2:0] == spi_sreg[2:0]);
		end
		if (f_rxseq[4])
		begin
			assert(spi_bitcount == 4);
			assert(f_data[3:0] == spi_sreg[3:0]);
		end
		if (f_rxseq[5])
		begin
			assert(spi_bitcount == 5);
			assert(f_data[4:0] == spi_sreg[4:0]);
		end
		if (f_rxseq[6])
		begin
			assert(spi_bitcount == 6);
			assert(f_data[5:0] == spi_sreg[5:0]);
		end
		if (f_rxseq[7])
		begin
			assert(spi_bitcount == 7);
			assert(f_data[6:0] == spi_sreg[6:0]);
		end
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

/*
	always @(*)
	case({ sync_spi_frame, sync_frame_pipe })
	3'b000: begin end
	3'b001: begin end
	3'b011: begin assert(!sync_spi_frame); end
	3'b111: begin end
	3'b110: begin end
	3'b100: begin end
	default: assert(0);
	endcase
*/
	wire	[2:0] f_sync_frame;
	assign	f_sync_frame = { sync_spi_frame, sync_frame_pipe };
	always @(posedge gbl_clk)
	assert(	(f_sync_frame == 3'h0)
		||(f_sync_frame == 3'h1)
		||(f_sync_frame == 3'h3)
		||(f_sync_frame == 3'h7)
		||(f_sync_frame == 3'h6)
		||(f_sync_frame == 3'h4));

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
	reg	[7:0]	f_txmit, f_txseq;

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && (!$past(o_frame) && !$fell(o_rd)))
		assume($stable(i_byte));

	// always @(posedge gbl_clk)
	// if (f_past_valid_gbl && (!$past(o_frame) && !$fell(o_rd)))
		// assume($stable(i_byte));

	initial	f_txseq = 0;
	always @(posedge gbl_clk)
	if (i_spi_csn)
		f_txseq <= 0;
	else if (!i_spi_csn && f_will_fall_sck)
	begin
		f_txseq <= f_txseq << 1;
		if (spi_bitcount_n[2:0] == 3'h7)
		begin
			f_txseq[0] <= 1'b1;
			f_txmit <= i_byte;
		end
	end

	always @(*)
	if (!i_spi_csn && |f_txseq)
	begin
		if (f_txseq[0])
		begin
			assert(spi_bitcount_n[2:0] == 3'b000);
			assert(f_txmit[7:0] == spi_output);
		end
		if (f_txseq[1])
		begin
			assert(spi_bitcount_n[2:0] == 3'b001);
			assert(f_txmit[6:0] == spi_output[7:1]);
		end
		if (f_txseq[2])
		begin
			assert(spi_bitcount_n[2:0] == 3'b010);
			assert(f_txmit[5:0] == spi_output[7:2]);
		end
		if (f_txseq[3])
		begin
			assert(spi_bitcount_n[2:0] == 3'b011);
			assert(f_txmit[4:0] == spi_output[7:3]);
		end
		if (f_txseq[4])
		begin
			assert(spi_bitcount_n[2:0] == 3'b100);
			assert(f_txmit[3:0] == spi_output[7:4]);
		end
		if (f_txseq[5])
		begin
			assert(spi_bitcount_n[2:0] == 3'b101);
			assert(f_txmit[2:0] == spi_output[7:5]);
		end
		if (f_txseq[6])
		begin
			assert(spi_bitcount_n[2:0] == 3'b110);
			assert(f_txmit[1:0] == spi_output[7:6]);
		end
		if (f_txseq[7])
		begin
			assert(spi_bitcount_n[2:0] == 3'b111);
			assert(f_txmit[0] == spi_output[7]);
		end
	end

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

	always @(*)
	if (!i_spi_csn && xck_stb)
		assert(spi_bitcount == 3'h7 || spi_bitcount == 0 || spi_bitcount <= 3'h3);

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
	//	Random induction properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	always @(posedge gbl_clk)
	if ($rose(i_spi_sck))
		assert(spi_bitcount == spi_bitcount_n[2:0]);

	always @(posedge gbl_clk)
	if (f_past_valid_gbl && $fell(i_spi_csn))
		assert(sync_spi_rd[0] == sync_spi_rd[1]);

	always @(*)
	if (spi_bitcount_n[2:0] == 3'h7)
		assert(!o_frame);

	always @(*)
	if (spi_bitcount_n[3])
		assert(!o_frame);

	always @(*)
	if (&sync_rd_pipe)
		assert(o_rd || !o_frame);

	////////////////////////////////////////////////////////////////////////
	//
	//	Cover properties
	//
	////////////////////////////////////////////////////////////////////////
	//
	//
	reg	[2:0]	f_bytes_sent, f_bytes_rcvd;
	reg		f_cvrspd_equal, f_cvrspd_fast, f_cvrspd_slow;
	(* anyconst *)	reg	f_cvrprop;

	initial	f_bytes_rcvd = 0;
	always @(posedge i_clk)
	if (o_we && (!(&f_bytes_rcvd)))
		f_bytes_rcvd <= f_bytes_rcvd + 1;

	initial	f_bytes_sent = 0;
	always @(posedge gbl_clk)
	if (f_past_valid_gbl && $fell(f_txseq[7]) && (!(&f_bytes_sent)))
		f_bytes_sent <= f_bytes_sent + 1;

	always @(*)
		f_cvrspd_fast = (f_spick_step == { 1'b1, {(F_CKBITS-1){1'b0}}})
			&& (f_sysck_step == (f_spick_step >> 1)
						+ (f_spick_step >> 2) + 1);

	always @(*)
		f_cvrspd_equal = (f_sysck_step == f_spick_step);

	always @(*)
		f_cvrspd_slow = (f_sysck_step == { 1'b1, {(F_CKBITS-1){1'b0}}})
			&& (f_spick_step < (f_sysck_step >> 1));

	always @(*)
		cover(o_we && o_byte == 8'hda);

	always @(posedge gbl_clk)
	if (f_cvrprop && $changed(i_byte))
		assume((i_byte == $past(i_byte) + 8'h11)
			&&(i_byte[7:4] != i_byte[3:0]));

	always @(posedge gbl_clk)
	if (f_cvrprop && $changed(o_byte))
		assume((o_byte == $past(o_byte) + 8'h11)
			&&(o_byte[7:4] != o_byte[3:0]));

	always @(posedge gbl_clk)
	if (f_past_valid_gbl)
	begin
		cover($changed(i_byte));
		cover($changed(o_byte));
		if ($rose(o_we))
		begin
			cover($stable(o_byte));
			cover($changed(o_byte));
		end
		if ($fell(o_rd))
		begin
			cover($stable(i_byte));
			cover($changed(i_byte));
		end
	end

	//
	//
	//
	always @(posedge gbl_clk)
		// 44 steps
		cover(f_bytes_rcvd == 2 && $fell(i_spi_csn) && f_cvrspd_fast);
	always @(posedge gbl_clk)
		// 42 steps
		cover(f_bytes_sent == 2 && $fell(i_spi_csn) && f_cvrspd_fast);

	always @(posedge gbl_clk)
		// 44 steps
		cover(f_bytes_rcvd == 2 && $fell(i_spi_csn) && f_cvrspd_equal);
	always @(posedge gbl_clk)
		// 40 steps
		cover(f_bytes_sent == 2 && $fell(i_spi_csn) && f_cvrspd_equal);

	always @(posedge gbl_clk)
		// 83 clocks
		cover(f_bytes_rcvd == 2 && $fell(i_spi_csn) && f_cvrspd_slow);
	always @(posedge gbl_clk)
		// 110 steps
		cover(f_bytes_sent == 2 && $fell(i_spi_csn) && f_cvrspd_slow);
`endif
endmodule
