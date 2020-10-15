module pin_drv(pin, o);
	inout pin;
	input  o;
	bufif0 #(1,0,1)(pin, 0, o);
endmodule

`define CHECK_TIMING

`include "i2c-edge-detect.v"
`include "i2c-master.v"
`include "i2c-slave.v"
`include "i2c_test_ram.v"
`include "i2c_chain.v"

`define NELM 8

module test;

parameter US=100;
parameter I2C_MODE=2;

	tri1 sda, scl;
	reg  clk, rst, ws;
	reg  [7:0] dat;
	wire [7:0] mst_dat_out;
	reg  [`C_SZ-1:0] cmd;
	wire [`S_SZ-1:0] status;

	reg  [7:0] slv_dat;

	wire mst_scl_in, mst_scl_out;
	wire mst_sda_in, mst_sda_out;
	wire slv_scl_in;
	wire slv_scl_out = 1;
	wire slv_sda_in, slv_sda_out;

	wire tst_sda_in, tst_sda_out, tst_sda_t;
	wire tst_scl_in, tst_scl_out, tst_scl_t;

	wire buf_scl_out, buf_sda_out;

	wire [7:0] slv_dat_out;
	wire slv_ws, slv_rs;

	integer i;

task wstrob;
	begin
		@(posedge clk);
		ws = 1;
		@(posedge clk);
		ws = 0;
	end
endtask

task sync;
	begin
		wstrob;
		if ( ! status[`SB_ERR] ) begin
			while ( status[`SB_BSY] ) begin
				@(posedge clk);
			end
		end
		if ( status[`SB_ERR] ) begin
			cmd = `C_CLRS;
			wstrob;
		end
	end
endtask

	pin_drv   sda_drv(.pin(sda), .o(buf_sda_out) );
	pin_drv   scl_drv(.pin(scl), .o(buf_scl_out) );

	i2c_chain c1(
		.iic_bus_sda_i( sda ),
		.iic_bus_sda_t( buf_sda_out ),

		.iic_bus_scl_i( scl ),
		.iic_bus_scl_t( buf_scl_out ),

		.iic_ups_1_sda_i( slv_sda_in ),
		.iic_ups_1_sda_o( 1'b0 ),
		.iic_ups_1_sda_t( slv_sda_out ),

		.iic_ups_1_scl_i( slv_scl_in ),
		.iic_ups_1_scl_o( 1'b0 ),
		.iic_ups_1_scl_t( slv_scl_out ),

		.iic_ups_2_sda_i( mst_sda_in ),
		.iic_ups_2_sda_o( 1'b0 ),
		.iic_ups_2_sda_t( mst_sda_out ),

		.iic_ups_2_scl_i( mst_scl_in ),
		.iic_ups_2_scl_o( 1'b0 ),
		.iic_ups_2_scl_t( mst_scl_out ),

		.iic_ups_3_sda_i( tst_sda_in ),
		.iic_ups_3_sda_o( tst_sda_out ),
		.iic_ups_3_sda_t( tst_sda_t ),

		.iic_ups_3_scl_i( tst_scl_in ),
		.iic_ups_3_scl_o( tst_scl_out ),
		.iic_ups_3_scl_t( tst_scl_t )
	);

/*
	pin_drv slv_sda_drv(sda, slv_sda_out);
*/

	i2c_slave #(.US(US),.I2C_MODE(I2C_MODE)) slv(
		.clk(clk),
		.scl(slv_scl_in),
		.sda(slv_sda_in),
		.sda_out(slv_sda_out),
		//.act_out(),
		.rs_out(slv_rs),
		.dat_in(slv_dat),
		.ws_out(slv_ws),
		.dat_out(slv_dat_out),
		.rst(rst)
	);

	i2c_test_ram #(.US(US),.I2C_MODE(I2C_MODE)) ram(
		.clk( clk ),
		.aresetn( !rst ),
		.scl_i( tst_scl_in ),
		.scl_o( tst_scl_out ),
		.scl_t( tst_scl_t ),
		.sda_i( tst_sda_in ),
		.sda_o( tst_sda_out ),
		.sda_t( tst_sda_t )
	);

	defparam ram.slv.MYADDR=7'h3a;

/*
	pin_drv mst_sda_drv(sda, mst_sda_out);
	pin_drv mst_scl_drv(scl, mst_scl_out);
*/
    i2c_master #(.US(US), .I2C_MODE(I2C_MODE))  mst(
		.clk(clk),
		.sda(mst_sda_in),
		.sda_out(mst_sda_out),
		.scl(mst_scl_in),
		.scl_out(mst_scl_out),
		.cmd(cmd),
		.stat_out(status),
		.dat(dat),
		.dat_out(mst_dat_out),
		.ws(ws),
		.rst(rst));

	// CLOCK
	always #1 clk=~clk;

	// Read slave
	always @(posedge clk) begin
		if ( slv_ws ) begin
			if ( slv_dat_out != 8'haa) begin
				$display("SLV write mismatch, got %02x ,expected %02x", slv_dat_out, 8'haa);
			end
			slv_dat <= slv_dat_out;
		end
	end

	always @(posedge clk) begin
		if ( slv_rs )
			slv_dat <= slv_dat + 1;
	end

	initial
	begin
		$dumpvars;
		clk = 0;
		rst = 1;
		dat = 8'h5a;
		slv_dat = 8'h55;
		for ( i=0; i<10; i+=1 )
			@(posedge clk);
		rst =0;
		#(10*US); /* let the thing come up */
		@(posedge clk);
		cmd = 0;
		wstrob;

		cmd = `C_STRT | `C_STOP;
		sync;

		cmd = `C_STRT;
		sync;

		dat = (7'h3b << 1) | 1'b1;
		cmd = `C_STRT | `C_WRTE;
		sync;

		cmd = `C_READ;
		sync;

		if ( mst_dat_out != 8'h55 ) begin
			$display("SLV readback mismatch; got 0x%02x, exp 0x%02x", mst_dat_out, 8'h55);
		end

		cmd = `C_READ | `C_NACK;
		sync;

		if ( mst_dat_out != 8'h56 ) begin
			$display("SLV readback mismatch; got 0x%02x, exp 0x%02x", mst_dat_out, 8'h56);
		end

		cmd = `C_WRTE;
		sync;

		cmd = `C_STOP;
		sync;

		dat = (7'h3b << 1) | 1'b0;
		cmd = `C_STRT | `C_WRTE;
		sync;

        dat = 8'haa;
		cmd = `C_WRTE | `C_STOP;
		sync;

		cmd = `C_STRT | `C_WRTE;
		dat = {7'h3a, 1'b0};
		sync;
		cmd = `C_WRTE;
		dat = 0; // write index
		sync;
		for ( i=0; i<`NELM; i=i+1 ) begin
			dat = dat+8'h11;
			sync;
		end

		cmd = `C_STRT | `C_WRTE;
		dat = {7'h3a, 1'b1};
		sync;

		cmd = `C_READ;
		for ( i=0; i<`NELM; i=i+1 ) begin
			if ( i == `NELM - 1 )
				cmd = cmd | `C_NACK;
			sync;
			if ( mst_dat_out != 8'h11*(i+1) ) begin
				$display("Readback mismatch: got %x expected %x\n", mst_dat_out, 8'h11*(i+1));
			end
		end

		cmd = `C_STOP;
		sync;

				
		#(20*US);
		$finish;
	end
endmodule
