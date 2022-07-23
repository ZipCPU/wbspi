////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	spicpu.v
// {{{
// Project:	wbspi, a set of Serial Peripheral Interface cores
//
// Purpose:	To make it easy to script SPI commands.
//
// Registers:
// {{{
//	0,0: Control/status
//		(&CSN),4b idx, SCK, MISO[idx]
//		[22:8]: (Current decoded  command)
//		[7:0]: Currently executing command
//	0,1: Override
//		Provide a means of sending commands directly, without scripting
//		them.  Commands may be written to bits [7:0] as thought they
//		were issued from memory.
//		Reading from this register will return a shift (left) register
//		of all that's been read, with the following exceptions:
//		- A START instruction clears the register.
//		- [31]: LAST bit was set on the last one
//		- [30:28]: Bit field indicating if bytes [23:16], [15:8], [7:0]
//		  have valid data in them respectively.
//	0,2: Address
//		Turns on autonomous mode, instructing the CPU to start at the
//		address given.
//	0,3: Clock divider
//	(??) 1,x: Sets/reads the internal memory of this peripheral.
// }}}
//
// Commands:	All instructions are 8-bits.  Many support immediates following.
// {{{
//	00011111 STOP		Raise all CSn pins to deactivate the interface.
//				STOP commands may also be implied by other
//				commands.
//	000iiiii START ID	Drop CSn[iii] to activate (implies STOP for
//				anything else that's active.)  If we are only
//				configured for a single CSN, then any START
//				command will activate (lower) that CSN.
//	001nmbr	READ #NMBR	Read #NMBR+1 bytes.  No immediates follow.
//				#NMBR ranges from 0--15, for a READ of 1--16
//				bytes.  Data will be read from the selected CSN.
//	010nmbr	SEND #NMBR	Send #NMBR bytes
//				This command is followed by the bytes to send.
//				Any return responses are ignored.
//	011nmbr	TXRX #NMBR	Send #NMBR bytes.
//				#NMBR immediates follow.
//				Any return responses are sent up the chain.
//	100xxxx	LAST		The next item read will have the AXI stream
//				TLAST flag bit set when sent across the stream.
//	1010xxx	HALT		No further instructions accepted until reset
//				or CPU override/restart.  Implies STOP.
//	1011xxx	WAIT		Just like HALT, but will also restart on an
//				interrupt.  Implies STOP if active.
//	1100xxx	TARGET		Sets the target address for a future JUMP
//				instruction.  Implies STOP if active.
//	1101xxx	JUMP		Jumps to the jump target address.  Implies STOP.
//	111xxxx	NOP		No operation.  Reserved for future instructions.
//
//	'x' bits are reserved and should be set to zero.
// }}}
// Decoded commands:
// {{{
//	These are the ones sent downstream.  Since they are internal, they
//	don't need to fit within 8-bits.
//
//	CSN, (&CSN), LAST, KEEP, 8'bSEND
//	 -  CSN : The CSN bits to be set
//	 - &CSN : True if all CSN bits are high and the channel is to be
//			made idle
//	 - LAST : True if the LAST flag is to be set when the result is placed
//			into the output AXI stream
//	 - KEEP : True if the result of this operation is to be sent to the
//			output AXI stream.
//	 - 8bSEND: An 8-bit value to output over the SPI channel.  Always set.
//
//	No command implies keeping CSN,&CSN as they are, and the clock off.
// }}}
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
// }}}
// Copyright (C) 2022, Gisselquist Technology, LLC
// {{{
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
// }}}
// License:	GPL, v3, as defined and found on www.gnu.org,
// {{{
//		http://www.gnu.org/licenses/gpl.html
//
////////////////////////////////////////////////////////////////////////////////
//
`default_nettype none
// }}}
module	spicpu #(
		// {{{
		parameter	ADDRESS_WIDTH = 25,
		parameter	DATA_WIDTH    = 32,
		parameter	NCE = 1,	// Number of CS# sigs to control
		// parameter	LGMEM = 5,	// Address width of internal mem
		parameter [ADDRESS_WIDTH+$clog2(DATA_WIDTH/8)-1:0] RESET_ADDRESS
				= 0,
		// localparam	AW = LGMEM,
		parameter [0:0]	 OPT_MANUAL  = 0,
		// Verilator lint_off UNUSED
		parameter 	 MANUAL_CSN  = 12,
		parameter 	 MANUAL_BIT  = 11,
		parameter 	 MANUAL_SCK  = 10,
		parameter 	 MANUAL_MOSI =  9,
		parameter 	 MANUAL_MISO =  8,
		// Verilator lint_on  UNUSED
		parameter [0:0]	 OPT_LOWPOWER = 0,
		parameter [0:0]	 OPT_START_HALTED= 0,
		parameter [0:0]	 OPT_SHARED_MISO = 0,
		parameter [0:0]	 DEF_CPOL = 1,
		// DEF_CKCOUNT -- Default number of system clocks per SCK
		// {{{
		// This number gets divided by two, so *DON'T* set it to
		// anything less than 2.
		parameter [11:0] DEF_CKCOUNT = 20	// Divide clk by 20
		// Dividing by 20 should yield a clock rate of SYSCLK/20
		//   or 100MHz / 20 = 5MHz
		// }}}
		// }}}
	) (
		// {{{
		input	wire	i_clk, i_reset,
		//
		// WISHBONE/slave control interface
		// {{{
		input	wire			i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[1:0]		i_wb_addr,
		input	wire	[32-1:0]	i_wb_data,
		input	wire	[32/8-1:0]	i_wb_sel,
		output	wire			o_wb_stall,
		output	reg			o_wb_ack,
		output	reg	[32-1:0]	o_wb_data,
		// }}}
		// WISHBONE/master control interface
		// {{{
		output	wire			o_pf_cyc, o_pf_stb, o_pf_we,
		output wire [ADDRESS_WIDTH-1:0]	o_pf_addr,
		output	wire [DATA_WIDTH-1:0]	o_pf_data,
		output	wire [DATA_WIDTH/8-1:0]	o_pf_sel,
		input	wire			i_pf_stall, i_pf_ack,
		input	wire [DATA_WIDTH-1:0]	i_pf_data,
		input	wire			i_pf_err,
		// }}}
		// SPI interface
		// {{{
		output	reg	[NCE-1:0]	o_spi_csn,
		output	reg			o_spi_sck,
		output	reg			o_spi_mosi,
		input	wire [(OPT_SHARED_MISO ? 0:(NCE-1)):0]	i_spi_miso,
		// }}}
		// Outgoing AXI-Stream interface
		// {{{
		output	reg		M_AXIS_TVALID,
		input	wire		M_AXIS_TREADY,
		output	reg	[7:0]	M_AXIS_TDATA,
		output	reg		M_AXIS_TLAST,
		// }}}
		input	wire		i_sync_signal
		// }}}
	);

	// Local declarations
	// {{{
	localparam	[1:0]	ADR_CONTROL  = 2'b00,
				ADR_OVERRIDE = 2'b01,
				ADR_ADDRESS  = 2'b10,
				ADR_CKCOUNT  = 2'b11;

	localparam		BAW = ADDRESS_WIDTH
					+ $clog2(DATA_WIDTH/8); // Byte addr wid

	localparam	[2:0]	CMD_START  = 3'h0, // CMD_STOP = 3'h0,
				CMD_READ   = 3'h1, CMD_SEND  = 3'h2,
				CMD_TXRX   = 3'h3,
				CMD_LAST   = 3'h4;
				// CMD_NOOP   = 3'h7;
	localparam	[3:0]	CMD_HALT   = 4'ha, CMD_WAIT  = 4'hb,
				CMD_TARGET = 4'hc,
				CMD_JUMP   = 4'hd;

	wire		bus_write, bus_read, bus_jump, bus_override, bus_manual;
	wire	[1:0]	bus_write_addr, bus_read_addr;
	wire	[31:0]	bus_write_data;
	wire	[3:0]	bus_write_strb;

	reg	[15:0]		ovw_data;
	reg	[7:0]		ovw_cmd;

	wire			cpu_reset, cpu_clear_cache;
	reg			cpu_new_pc;

	reg			next_valid, next_illegal, next_cpu;
	wire			next_ready;
	reg	[7:0]		next_insn;
	reg	[BAW-1:0]	next_insn_addr;

	reg			dcd_stop, dcd_valid;
	wire			dcd_ready;
	reg			dcd_active, dcd_last, dcd_send, dcd_keep,
				dcd_byte;
	reg			r_stopped, r_wait;
	reg	[NCE-1:0]	dcd_csn;
	reg	[9:0]		dcd_command;

	reg			imm_cycle;
	reg	[5:0]		imm_count;


	wire			pf_valid, pf_ready, pf_illegal;
	wire	[7:0]		pf_insn;
	reg	[BAW-1:0]	pf_jump_addr;
	wire	[BAW-1:0]	pf_insn_addr;

	reg	[11:0]	edge_counter, ckcount;
	reg		spi_ckedge;

	wire		spi_stall;
	reg		spi_active, spi_last, spi_keep;
	reg	[6:0]	spi_srout, spi_srin;
	reg	[4:0]	spi_count;

	wire		miso;

	wire	set_tvalid;

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Bus handling
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	bus_write      = i_wb_stb && i_wb_we && !o_wb_stall;
	assign	bus_write_addr = i_wb_addr;
	assign	bus_write_data = i_wb_data;
	assign	bus_write_strb = i_wb_sel;

	assign	bus_read       = i_wb_stb && !i_wb_we && !o_wb_stall;
	assign	bus_read_addr  = i_wb_addr;

	assign	o_wb_stall = 1'b0;

	initial	o_wb_ack = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_wb_ack <= 1'b0;
	else
		o_wb_ack <= i_wb_stb && !o_wb_stall;

	always @(posedge i_clk)
	if (OPT_LOWPOWER && i_reset)
		o_wb_data <= 0;
	else if (bus_read || !OPT_LOWPOWER)
	begin
		o_wb_data <= 0;
		case(bus_read_addr)
		ADR_CONTROL:  begin end
		ADR_OVERRIDE: o_wb_data <= { ovw_data, manual_data, ovw_cmd };
		ADR_ADDRESS:  o_wb_data[BAW-1:0] <= pf_insn_addr;
		ADR_CKCOUNT:  o_wb_data[11:1] <= ckcount;
		default: begin end
		endcase

	end else if (OPT_LOWPOWER)
		o_wb_data <= 0;

	assign	bus_override = r_stopped && bus_write
			&& bus_write_addr == ADR_OVERRIDE
			&& bus_write_strb[0]
			&& (!OPT_MANUAL || !bus_write_data[MANUAL_BIT]
				|| !bus_write_strb[MANUAL_BIT/8]);
	assign	bus_manual = OPT_MANUAL && bus_write
			&& bus_write_addr == ADR_OVERRIDE
			&& bus_write_data[MANUAL_BIT]
			&& bus_write_strb[MANUAL_BIT/8];
	assign	bus_jump = bus_write && bus_write_addr == ADR_ADDRESS
			&& (&bus_write_strb) && r_stopped;
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Instruction fetch
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	cpu_reset       = r_stopped;
	assign	cpu_clear_cache = 1'b0;

`ifndef	FORMAL
	dblfetch #(
		.ADDRESS_WIDTH(BAW), .DATA_WIDTH(DATA_WIDTH), .INSN_WIDTH(8)
	) u_fetch (
		// {{{
		.i_clk(i_clk), .i_reset(i_reset || cpu_reset),
		//
		.i_new_pc(cpu_new_pc), .i_clear_cache(cpu_clear_cache),
		.i_ready(pf_ready), .i_pc(pf_jump_addr),
		.o_valid(pf_valid), .o_illegal(pf_illegal),
		.o_insn(pf_insn), .o_pc(pf_insn_addr),
		//
		.o_wb_cyc(o_pf_cyc), .o_wb_stb(o_pf_stb), .o_wb_we(o_pf_we),
		.o_wb_addr(o_pf_addr), .o_wb_data(o_pf_data),
		.i_wb_stall(i_pf_stall),
		.i_wb_ack(i_pf_ack), .i_wb_data(i_pf_data),
		.i_wb_err(i_pf_err)
		// }}}
	);
`endif

	assign	o_pf_sel = -1;
	assign	pf_ready = !r_stopped && (!next_valid || next_ready);

	// next_valid
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		next_valid <= 0;
	else if (bus_override || (pf_valid && pf_ready))
		next_valid <= 1;
	else if (next_ready)
		next_valid <= 0;

`ifdef	FORMAL
	always (*)
	if (pf_ready)
		assert(!r_stopped);

	always (*)
	if (pf_ready)
		assert(!next_valid || next_ready);
`endif
	// }}}

	// next_illegal
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		next_illegal <= 0;
	else if (r_stopped)
		next_illegal <= 0;
	else if (pf_valid && pf_ready)
		next_illegal <= pf_illegal;
	// }}}

	// next_cpu
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		next_cpu <= 1'b0;
	else if (bus_override)
		next_cpu <= 1'b1;
	else if (pf_valid && pf_ready)
		next_cpu <= 1'b0;
	// }}}

	// next_insn, ovw_cmd
	// {{{
	always @(posedge i_clk)
	if (r_stopped && bus_write && bus_write_addr == ADR_OVERRIDE
			&& bus_write_strb[0]
			&& (!OPT_MANUAL || !bus_write_data[MANUAL_BIT]
				|| !bus_write_strb[MANUAL_BIT/8]))
	begin
		next_insn <= bus_write_data[7:0];
	end else if (pf_valid && pf_ready)
		next_insn <= pf_insn;

	always @(posedge i_clk)
	if (r_stopped && bus_write && bus_write_addr == ADR_OVERRIDE
			&& bus_write_strb[0]
			&& (!OPT_MANUAL || !bus_write_data[MANUAL_BIT]
				|| !bus_write_strb[MANUAL_BIT/8]))
		ovw_cmd <= bus_write_data[7:0];
	// }}}

	// next_insn_addr
	// {{{
	always @(posedge i_clk)
	if (!next_valid || next_ready)
	begin
		if (r_stopped)
			next_insn_addr <= 0;
		else if (pf_valid)
			next_insn_addr <= pf_insn_addr;
	end
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Instruction decode
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// localparam	LASTBIT = 9;
			// KEEPBIT = 8;
			// Bits 7-0 are data bits

	assign	next_ready = (!dcd_valid || !spi_stall)
		&& (!imm_cycle || dcd_send)
		&& (next_cpu || (!r_stopped && (!r_wait || i_sync_signal)));
	assign	dcd_ready = !spi_stall;

	// cpu_new_pc, pf_jump_addr
	// {{{
	always @(posedge i_clk)
	begin
		cpu_new_pc <= 1'b0;

		if (bus_jump)
			cpu_new_pc <= 1'b1;
		if (next_valid && next_ready
				&& !imm_cycle && next_insn[7:4] == CMD_JUMP)
			cpu_new_pc <= 1'b1;

		// pf_jump_addr
		// {{{
		if (next_valid && next_ready
				&& !imm_cycle && next_insn[7:4] == CMD_TARGET)
			pf_jump_addr <= next_insn_addr + 1;	// TARGET
		if (bus_jump)
			pf_jump_addr <= bus_write_data[BAW-1:0];
		// }}}

		if (i_reset)
		begin
			cpu_new_pc <= 1'b0;
			pf_jump_addr <= RESET_ADDRESS;
		end
	end
	// }}}

	// r_stopped
	// {{{
	always @(posedge i_clk)
	begin
		if (next_valid && next_ready
				&& !imm_cycle && next_insn[7:4] == CMD_HALT)
			r_stopped <= 1'b1;
		if (next_valid && next_ready && next_cpu)
			r_stopped <= 1'b1;
		if ((!imm_cycle || dcd_send) && next_valid && next_illegal)
			r_stopped <= 1'b1;
		if (bus_jump)
			r_stopped <= 1'b0;

		if (i_reset)
			r_stopped <= OPT_START_HALTED;
	end
	// }}}

	// r_wait
	// {{{
	always @(posedge i_clk)
	begin
		if (i_sync_signal)
			r_wait <= 1'b0;

		if (next_valid && next_ready && !imm_cycle
						&& next_insn[7:4] == CMD_WAIT)
			r_wait <= 1'b1;
		if ((!imm_cycle || dcd_send)&&next_valid&& next_illegal)
			r_wait <= 1'b0;
		if (bus_jump)
			r_wait <= 1'b0;

		if (i_reset)
			r_wait <= 1'b0;
	end
	// }}}

	// Massive decoder state machine
	always @(posedge i_clk)
	begin
		if (dcd_ready)
			dcd_valid <= 1'b0;

		if (dcd_stop)
			dcd_active <= 1'b0;

		if (next_valid && next_ready)
		begin
			// {{{
			dcd_stop  <= 1'b0;
			dcd_valid <= next_valid || dcd_stop;
			dcd_byte  <= 1'b0;
			if (imm_cycle)
			begin
				// {{{
				dcd_byte <= 1'b1;
				dcd_valid <= 1'b1;
				if (!dcd_send)
				begin // READ # in progress
					// {{{
					dcd_command[9:0] <= {
						(dcd_last && imm_count == 1),
						dcd_keep, 8'h00 };
					imm_count <=  imm_count - 1;
					imm_cycle <= (imm_count > 1);
					if (imm_count == 1)
						dcd_last <= 1'b0;
					// }}}
				end else if (next_valid)
				begin // SEND #
					// {{{
					dcd_command[9:0] <= { 1'b0,
						dcd_keep, next_insn[7:0] };
					imm_count <= imm_count - 1;
					imm_cycle <= (imm_count > 1);
					if (!dcd_keep || imm_count == 1)
						dcd_last <= 1'b0;
					// }}}
				end else begin // Waitn on (unavail) send data
					// {{{
					dcd_valid <= 1'b0;
					if (OPT_LOWPOWER)
						dcd_command[9:0] <= 10'h00;
					// }}}
				end
				// }}}
			end else if (next_valid) casez(next_insn[7:4])
			{ CMD_START, 1'b? }: begin // START #ID
				// {{{
				dcd_valid <= 1'b1;
				dcd_byte  <= 1'b0;
				dcd_last  <= 1'b0;

				if (next_insn[4:0] >= NCE)
				begin
					dcd_csn    <= -1;
					dcd_active <= 1'b0;
					dcd_stop   <= (dcd_active);
					if (!dcd_active)
						dcd_valid <= 1'b0;
				end else begin
					dcd_csn    <= ~(1<< next_insn[4:0]);
					dcd_active <= 1'b1;
				end end
				// }}}
			{ CMD_READ,  1'b? }: begin // READ #NMBR
				// {{{
				dcd_valid <= 1'b1;
				dcd_byte  <= 1'b1;
				dcd_send  <= 1'b0; // Not sending data
				dcd_keep  <= 1'b1; // but we are keeping results
				imm_cycle <= (next_insn[4:0] > 0);
				imm_count <= { 1'b0, next_insn[4:0] }; // Remaining items
				dcd_last  <= dcd_last && (next_insn[4:0] > 0);
				dcd_command[9:0] <= {
					(dcd_last && (next_insn[4:0] == 5'h0)),
					1'b1, 8'h0 };
				end
				// }}}
			{ CMD_SEND,  1'b? }: begin // SEND #NMBR
				// {{{
				dcd_valid <= 1'b0; // No data(yet), so not valid
				dcd_send  <= 1'b1; // Sending data
				dcd_keep  <= 1'b0; // Not keeping the results
				dcd_byte  <= 1'b1;
				// dcd_last --- no change

				imm_cycle <= 1'b1;
				imm_count <= next_insn[4:0] + 1; // items
				end
				// }}}
			{ CMD_TXRX,  1'b? }: begin // TXRX #NMBR
				// {{{
				dcd_valid <= 1'b0; // No data(yet) to send
				dcd_send  <= 1'b1; // We're sending data
				dcd_keep  <= 1'b1; // Keep the results
				dcd_byte  <= 1'b1;
				// dcd_last --- no change
				imm_cycle <= 1'b1;
				imm_count <= next_insn[4:0] + 1;
				end
			// }}}
			{ CMD_LAST,  1'b? }:
				{ dcd_valid, dcd_last } <= 2'b01; // LAST insn
			CMD_HALT, CMD_WAIT: begin	// WAIT, HALT
				// {{{
				dcd_valid  <= (dcd_active);
				dcd_csn    <= -1;
				dcd_active <= 1'b0;
				dcd_byte   <= 1'b0;
				dcd_last   <= 1'b0;

				// { r_stopped, r_wait } <= { !next_insn[4],
				//				next_insn[4] };
				if (dcd_active)
				begin
					// Implied stop
					dcd_stop   <= 1'b1;
				end end
				// }}}
			CMD_TARGET: begin // TARGET
				// {{{
				dcd_valid  <= (dcd_active);
				pf_jump_addr <= next_insn_addr + 1; // TARGET
				dcd_csn    <= -1;
				dcd_active <= 1'b0;
				dcd_byte   <= 1'b0;
				dcd_last   <= 1'b0;

				if (dcd_active)
				begin
					// Implied stop
					dcd_stop   <= 1'b1;
				end end
				// }}}
			CMD_JUMP:   begin // JUMP
				// {{{
				dcd_valid  <= (dcd_active);
				dcd_csn    <= -1;
				dcd_active <= 1'b0;
				dcd_byte   <= 1'b0;
				dcd_last   <= 1'b0;

				if (dcd_active)
				begin
					// Implied stop
					dcd_stop   <= 1'b1;
				end end
				// }}}
			default: dcd_valid <= 1'b0;
			endcase

			if ((!imm_cycle || dcd_send)&&next_valid&& next_illegal)
			begin
				dcd_valid  <=  dcd_active;
				dcd_active <= (dcd_active && !dcd_stop);
				dcd_csn    <= -1;
				dcd_byte   <= 1'b0;
				r_stopped  <= 1'b1;
			end
			// }}}
		end

		if (i_reset)
		begin
			// {{{
			dcd_valid  <=  0;
			dcd_active <=  0;
			dcd_csn    <= -1;
			dcd_stop   <=  0;
			imm_cycle  <=  0;
			imm_count  <=  0;
			// }}}
		end
	end
	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// (Optional) Manual SPI control
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// Offers a simple bit-banging interface, should it be required
	wire			r_manual, manual_sck, manual_mosi;
	wire	[NCE-1:0]	manual_csn;
	wire	[7:0]		manual_data;

	generate if (OPT_MANUAL)
	begin : GEN_MANUAL;
		// {{{
		reg	manual, r_sck, r_mosi;
		reg	[NCE-1:0]	r_csn;

		// r_manual
		// {{{
		always @(posedge i_clk)
		if (i_reset || !r_stopped || bus_jump)
			r_manual <= 1'b0;
		else if (bus_write && bus_write_addr == ADR_OVERRIDE
				&& bus_write_strb[MANUAL_BIT/8])
			manual <= bus_write_data[MANUAL_BIT];
		// }}}

		// r_csn
		// {{{
		initial	r_csn = -1;
		always @(posedge i_clk)
		begin
			if (bus_write && bus_write_addr == ADR_OVERRIDE
				&& bus_write_strb[MANUAL_BIT/8])
			begin
				r_csn <= ~(1 << bus_write_data[15:12]);

				if (!bus_write_data[MANUAL_BIT])
					r_csn <= -1;
			end

			if (i_reset || !r_stopped)
				r_csn <= -1;
		end
		// }}}

		// r_sck, r_mosi
		// {{{
		always @(posedge i_clk)
		begin
			if (bus_manual)
			begin
				r_sck  <= bus_write_data[MANUAL_SCK];
				r_mosi <= bus_write_data[MANUAL_MOSI];
			end

			if (i_reset)
				r_sck <= DEF_CPOL;
		end
		// }}}

		assign	r_manual = manual;
		// }}}
	end else begin : NO_MANUAL_CONTROL
		// {{{
		assign	r_manual    = 1'b0;
		assign	manual_csn  = -1;
		assign	manual_sck  = DEF_CPOL;
		assign	manual_mosi = 0;
		assign	manual_data = 0;

		// Keep Verilator happy
		// {{{
		// Verilator lint_off UNUSED
		wire	unused_manual;
		assign	unused_manual = &{ 1'b0, bus_manual };
		// Verilator lint_on  UNUSED
		// }}}
		// }}}
	end endgenerate

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SPI clock generation
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	// ckcount
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		ckcount <= DEF_CKCOUNT/2;
	else if (bus_write && bus_write_addr == ADR_CKCOUNT)
	begin
		// ckcount = divisor / 2, since we count half edges
		if (bus_write_strb[1])
			ckcount[10:7] <= bus_write_data[11:8];
		if (bus_write_strb[0])
			ckcount[6:0] <= bus_write_data[7:1];
	end
	// }}}

	// edge_counter
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		edge_counter <= 0;
	else if (edge_counter > 0)
		edge_counter <= edge_counter - 1;
	else if (dcd_valid || spi_count > 0)
		edge_counter <= ckcount - 1;
	// }}}

	// spi_ckedge -- the one bit signal that causes everything below to mv
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		spi_ckedge <= 1;
	else
		spi_ckedge <= (edge_counter == 1);
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// SPI execution
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	spi_stall = !spi_ckedge || spi_count > 0;

	// o_spi_csn, spi_active, spi_last, spi_keep
	// {{{
	always @(posedge i_clk)
	if (i_reset)
	begin
		o_spi_csn <= -1;
		spi_active <= 1'b0;
	end else begin
		if (dcd_valid && !spi_stall)
		begin
			o_spi_csn  <= dcd_csn;
			spi_active <= dcd_active;
		end

		if (r_manual)
			o_spi_csn <= manual_csn;
	end

	always @(posedge i_clk)
	if (dcd_valid && !spi_stall)
	begin
		spi_last   <= dcd_command[9];
		spi_keep   <= dcd_command[8];
	end
	// }}}

	// o_spi_sck
	// {{{
	initial	o_spi_sck = DEF_CPOL;
	always @(posedge i_clk)
	if (i_reset)
		o_spi_sck <= DEF_CPOL;
	else begin
		if (!spi_stall)
			o_spi_sck <= DEF_CPOL;
		else if (spi_ckedge && (!M_AXIS_TVALID || M_AXIS_TREADY
				|| !spi_keep || !spi_active || spi_count > 1))
			o_spi_sck <= !o_spi_sck;

		if (r_manual)
			o_spi_sck <= manual_sck;
	end
	// }}}

	// o_spi_mosi, spi_srout
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		{ o_spi_mosi, spi_srout } <= 0;
	else begin
		if (dcd_valid && !spi_stall)
			{ o_spi_mosi, spi_srout } <= dcd_command[7:0];
		else if (spi_ckedge && (o_spi_sck ^ DEF_CPOL))
			{ o_spi_mosi, spi_srout } <= { spi_srout, 1'b0 };

		if (r_manual)
			o_spi_mosi <= manual_mosi;
	end
	// }}}

	generate if (OPT_SHARED_MISO)
	begin : GEN_SHARED_MISO
		assign	miso = i_spi_miso;
	end else begin : MISO_SELECT
		assign	miso = |(i_spi_miso & ~o_spi_csn);
	end endgenerate

	// spi_srin -- incoming shift register
	// {{{
	always @(posedge i_clk)
	if (OPT_LOWPOWER && i_reset)
		spi_srin <= 0;
	else if (OPT_LOWPOWER && dcd_valid && !spi_stall)
		spi_srin <= 0;
	else if (spi_ckedge && !(o_spi_sck ^ DEF_CPOL))
		spi_srin <= { spi_srin[5:0], miso };
	// }}}

	// spi_count -- a bit counter, not to be confused with the edge counter
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		spi_count <= 0;
	else if (dcd_valid && !spi_stall)
		spi_count <= dcd_byte ? 16 : 2;
	else if (spi_ckedge && spi_count > 0
			&& (!M_AXIS_TVALID || M_AXIS_TREADY
				|| !spi_keep || !spi_active || spi_count > 1))
		spi_count <= spi_count - 1;
	// }}}

	// }}}
	////////////////////////////////////////////////////////////////////////
	//
	// Outgoing AXI Stream
	// {{{
	////////////////////////////////////////////////////////////////////////
	//
	//

	assign	set_tvalid = spi_keep && spi_active && spi_ckedge
				&& spi_count == 1 && !r_manual;

	// M_AXIS_TVALID
	// {{{
	always @(posedge i_clk)
	if (i_reset)
		M_AXIS_TVALID <= 1'b0;
	else if (!M_AXIS_TVALID || M_AXIS_TREADY)
		M_AXIS_TVALID <= set_tvalid;
	// }}}

	// M_AXIS_TDATA
	// {{{
	always @(posedge i_clk)
	if (OPT_LOWPOWER && i_reset)
		M_AXIS_TDATA <= 8'h0;
	else if (!M_AXIS_TVALID || M_AXIS_TREADY)
	begin
		if (!OPT_LOWPOWER || set_tvalid)
		begin
			M_AXIS_TDATA <= { spi_srin, miso };
			ovw_data <= { ovw_data[7:0], spi_srin, miso };
		end else if (OPT_LOWPOWER)
			M_AXIS_TDATA <= 8'b0;
	end
	// }}}

	// M_AXIS_TLAST
	// {{{
	always @(posedge i_clk)
	if (!M_AXIS_TVALID || M_AXIS_TREADY)
	begin
		if (!OPT_LOWPOWER || set_tvalid)
			M_AXIS_TLAST <= spi_last;
		else
			M_AXIS_TLAST <= 1'b0;
	end
	// }}}

	// }}}

	// Keep Verilator happy
	// {{{
	// Verilator lint_off UNUSED
	wire	unused;
	assign	unused = &{ 1'b0, i_wb_cyc };
	// Verilator lint_on  UNUSED
	// }}}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Formal properties
// {{{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
`ifdef	FORMAL
`endif
// }}}
endmodule
