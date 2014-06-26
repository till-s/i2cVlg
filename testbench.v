module pin_drv(pin, o);
	inout pin;
	input  o;
	bufif0 #(1,0,1)(pin, 0, o);
endmodule

`define CHECK_TIMING

`include "i2c-master.v"
`include "i2c-slave.v"

module test;

	tri1 sda, scl;
	reg  clk, rst, ws;
	reg  [7:0] dat;
	reg  [`C_SZ-1:0] cmd;
	wire [`S_SZ-1:0] status;

	reg  [7:0] slv_dat;

	wire mst_scl_out, mst_sda_out, slv_sda_out;

	wire [7:0] slv_dat_out;
	wire slv_ws, slv_rs;

task sync;
	begin
		#2 ws = 1;
		#2 ws = 0;
		if ( ! status[`SB_ERR] ) begin
			@(negedge status[`SB_BSY]);
		end
		if ( status[`SB_ERR] ) begin
			cmd = `C_CLRS;
			#2 ws = 1;
			#2 ws = 0;
		end
	end
endtask

	pin_drv slv_sda_drv(sda, slv_sda_out);

	i2c_slave slv(
		.clk(clk),
		.scl(scl),
		.sda(sda),
		.sda_out(slv_sda_out),
		//.act_out(),
		.rs_out(slv_rs),
		.dat_in(slv_dat),
		.ws_out(slv_ws),
		.dat_out(slv_dat_out),
		.rst_in(rst)
	);


	pin_drv mst_sda_drv(sda, mst_sda_out);
	pin_drv mst_scl_drv(scl, mst_scl_out);
    i2c_master #(.US(100))  mst(
		.clk(clk),
		.sda(sda),
		.sda_out(mst_sda_out),
		.scl(scl),
		.scl_out(mst_scl_out),
		.cmd(cmd),
		.stat_out(status),
		.dat(dat),
		/* .dat_out(), */
		.ws(ws),
		.rst(rst));

	// CLOCK
	always #1 clk=~clk;

	// Read slave
	always @(posedge slv_ws) begin
		slv_dat <= slv_dat_out;
	end

	always @(posedge slv_rs) begin
		slv_dat <= slv_dat + 1;
	end

	initial
	begin
		$dumpvars;
		clk = 0;
		rst = 1;
		dat = 8'h5a;
		slv_dat = 8'h55;
		cmd = `C_STRT | `C_STOP;
		#4 rst =0;
		#100; /* let the thing come up */
		sync;

		cmd = `C_STRT;
		sync;

		dat = (7'h3b << 1) | 1'b1;
		cmd = `C_STRT | `C_WRTE;
		sync;

		cmd = `C_READ;
		sync;

		cmd = `C_READ | `C_NACK;
		sync;

		cmd = `C_WRTE;
		sync;

		cmd = `C_STOP;
		sync;

		dat = (7'h3b << 1) | 1'b0;
		cmd = `C_STRT | `C_WRTE;
		sync;

		cmd = `C_WRTE | `C_STOP;
		sync;

				
		#200;
		$finish;
	end
endmodule
