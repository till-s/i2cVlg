module pin_drv(pin, o);
parameter  integer RISE_DELAY = 1;
parameter  integer FALL_DELAY = 0;
localparam integer HI_Z_DELAY = RISE_DELAY;
	inout pin;
	input  o;
	bufif0 #(RISE_DELAY,FALL_DELAY,HI_Z_DELAY)(pin, 0, o);
endmodule

`define CHECK_TIMING

`include "i2c-edge-detect.v"
`include "i2c-sync.v"
`include "i2c-master.v"
`include "i2c-slave.v"
`include "i2c_test_ram.v"
`include "i2c_chain.v"

`define NELM 8

`define NOERR (`S_DON)

module test;

`define DEFAULT_US       100
`define DEFAULT_I2C_MODE 2
`include "i2c-timing-params.vh"

localparam HALF_PER       = 1;

localparam SCL_STRETCH    = 4*PER_LO;

localparam SCL_RISE_DELAY = (PER_TR + SCL_STRETCH)*2*HALF_PER;
localparam SCL_FALL_DELAY = PER_TF*2*HALF_PER;
localparam SDA_RISE_DELAY = PER_TR*2*HALF_PER;
localparam SDA_FALL_DELAY = PER_TF*2*HALF_PER;

	tri1 sda, scl;
	reg  clk = 1'b0, rst = 1'b1, ws = 1'b0;
	reg  [7:0] dat;
	wire [7:0] mst_dat_out;
	reg  [`C_SZ-1:0] cmd;
	wire [`S_SZ-1:0] status;
	wire [`D_SZ-1:0] i2cm_debug;

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
		ws <= 1;
		@(posedge clk);
		ws <= 0;
		@(posedge clk);
	end
endtask

task sync(input [`S_SZ-1:0] exp);
	begin
		wstrob;
		if ( ! status[`SB_ERR] ) begin
			while ( ! status[`SB_DON] ) begin
				@(posedge clk);
			end
		end
		if ( status[`SB_ERR] ) begin
			if ( status != exp ) begin
				$display("Error status 0x%x (unexpected -- expected: 0x%x)\n", status, exp);
				$finish;
			end
			cmd = `C_CLRS;
			wstrob;
		end
	end
endtask

	pin_drv #(.RISE_DELAY(SDA_RISE_DELAY), .FALL_DELAY(SDA_FALL_DELAY)) sda_drv(.pin(sda), .o(buf_sda_out) );
	pin_drv #(.RISE_DELAY(SCL_RISE_DELAY), .FALL_DELAY(SCL_FALL_DELAY)) scl_drv(.pin(scl), .o(buf_scl_out) );

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
		.scl_in(slv_scl_in),
		.sda_in(slv_sda_in),
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
		.sda_in(mst_sda_in),
		.sda_out(mst_sda_out),
		.scl_in(mst_scl_in),
		.scl_out(mst_scl_out),
		.cmd(cmd),
		.stat_out(status),
		.dat(dat),
		.dat_out(mst_dat_out),
		.ws(ws),
		.rst(rst),
		.debug(i2cm_debug)
	);

	// CLOCK
	always #HALF_PER clk=~clk;

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
		dat = 8'h5a;
		slv_dat = 8'h55;
		/* wait for buffers to settle */
		while ( sda !== 1'b1 || scl !== 1'b1 ) begin
			@(posedge clk);
		end
		for ( i=0; i<10; i=i+1 ) begin
			@(posedge clk);
		end
		rst =0;
		#(10*US); /* let the thing come up */
		@(posedge clk);
		cmd = 0;
		wstrob;

		cmd = `C_STRT | `C_STOP;
		sync(`NOERR);

		cmd = `C_STRT;
		sync(`NOERR);

		dat = (7'h3b << 1) | 1'b1;
		cmd = `C_STRT | `C_WRTE;
		sync(`NOERR);

		cmd = `C_READ;
		sync(`NOERR);

		if ( mst_dat_out != 8'h55 ) begin
			$display("SLV readback mismatch; got 0x%02x, exp 0x%02x", mst_dat_out, 8'h55);
		end

		cmd = `C_READ | `C_NACK;
		sync(`NOERR);

		if ( mst_dat_out != 8'h56 ) begin
			$display("SLV readback mismatch; got 0x%02x, exp 0x%02x", mst_dat_out, 8'h56);
		end

		cmd = `C_WRTE;
		sync(`S_ERR | `S_DON | `S_BBY );

		cmd = `C_STOP;
		sync(`NOERR);

		dat = (7'h3b << 1) | 1'b0;
		cmd = `C_STRT | `C_WRTE;
		sync(`NOERR);

		dat = 8'haa;
		cmd = `C_WRTE | `C_STOP;
		sync(`NOERR);

		cmd = `C_STRT | `C_WRTE;
		dat = {7'h3a, 1'b0};
		sync(`NOERR);

		cmd = `C_WRTE;
		dat = 0; // write index
		sync(`NOERR);
		for ( i=0; i<`NELM; i=i+1 ) begin
			dat = dat+8'h11;
			sync(`NOERR);
		end

		cmd = `C_STRT | `C_WRTE;
		dat = {7'h3a, 1'b1};
		sync(`NOERR);

		cmd = `C_READ;
		for ( i=0; i<`NELM; i=i+1 ) begin
			if ( i == `NELM - 1 )
				cmd = cmd /*| `C_NACK*/;
			sync(`NOERR);
			if ( mst_dat_out != 8'h11*(i+1) ) begin
				$display("Readback mismatch: got %x expected %x\n", mst_dat_out, 8'h11*(i+1));
			end
		end

		cmd = `C_STOP;
		sync(`S_ERR | `S_DON | `S_BBY | `S_LRA );

		/* read one w/o ack */
		cmd = `C_STOP | `C_READ | `C_NACK;
		sync(`NOERR);

				
		#(20*US);
		$finish;
	end
endmodule
