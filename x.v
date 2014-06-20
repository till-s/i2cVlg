module pin_drv(pin, o);
	inout pin;
	input  o;
	bufif0 #(1,0,1)(pin, 0, o);
endmodule

`define CHECK_TIMING

`include "i2c-master.v"
`include "i2c-slave.v"

`ifdef ZZZ
module i2c_s(sda, scl);
inout sda, scl;
reg rst;
reg [7:0] dat;
parameter N_DAT=4;
reg [7:0] dat_out [N_DAT-1:0];
reg sda_r, sda_l;
reg ack;
parameter ST_IDLE=3'b000;
parameter ST_ADDR=3'b001;
parameter ST_READ=3'b010;
parameter ST_WRTE=3'b011;
reg [2:0] state;
parameter MYADDR=(7'h3b);
integer   i,j;

	pin_drv sda_drv(sda, sda_r);

	initial begin 
		rst = 0;
		sda_r = 1;
		for ( j=0; j<N_DAT; j = j + 1 ) begin
			dat_out[j] = { 4'ha + j, 4'ha + j };
		end
		j     = 0;
		state = ST_IDLE;
	end

	// detect STOP
	always @(posedge sda) begin
		if ( scl != 0 ) begin
			rst <= 1;
		end
	end

	// detect START
	always @(negedge sda) begin
		if ( scl != 0 ) begin
			state <= ST_ADDR;	
			i     <= 0;
			if ( state != ST_IDLE ) begin
				$display("Start detector: RESTART from: %x\n", state);
			end 
		end
	end

	always @(scl or posedge rst) begin
		if ( rst == 1 ) begin
			state <= ST_IDLE;
			rst   <= 0;
			sda_r <= 1'b1;
		end else begin
			case ( state ) 
				ST_WRTE: if ( scl == 0 ) begin
					i <= i+1;
					if ( i < 7 ) begin
						sda_r <= dat_out[j][6 - i];
					end else if ( i == 7 ) begin
						sda_r <= 1'b1;
						j     <= (j == N_DAT ? 0 : j + 1);
					end else begin
						if ( ack ) begin
							sda_r = dat_out[j][7];
						end
						i <= 0;
					end
				end else begin
					if ( i == 8 ) begin
						ack = !sda;	
					end
				end

				ST_ADDR,
				ST_READ: begin
					if ( scl == 1 ) begin
						if ( i < 8 ) begin
							i <= i + 1;
							dat <= {dat[6:0], sda};
						end
					end else begin
						if ( i == 8 ) begin
							if ( state != ST_ADDR || (dat & 8'hfe) == (MYADDR << 1 ) ) begin
								sda_r <= 1'b0;
								i <= i+1;
							end else begin
								/* NO ACK */
								state <= ST_IDLE;
							end
						end else if ( i == 9 ) begin
							if ( state == ST_ADDR ) begin
								if ( dat[0] == 0 ) begin
									state <= ST_READ;
								end else begin
									state <= ST_WRTE;
									sda_r = dat_out[j][7];	
								end
							end
							i <= 0;
						end
					end
				end

				default: state = ST_IDLE;
			endcase
		end
	end

`ifdef XXX
	always @(state or rst)  begin
		if ( rst ) begin
			state <= ST_IDLE;	
			rst   <= 0;
		end else
		case ( state )

			ST_IDLE: begin
			
				@(negedge sda or posedge rst) begin
					if ( rst != 0 ) begin
						rst <= 0;
					end else begin
						state <= ST_ADDR;
					end
				end
			end

			ST_WRTE: begin
				for ( i=0; i<8; i=i+1 ) begin
					@(negedge scl or posedge rst) begin
						if ( rst != 0 ) begin
							i = 8;
						end else begin
							sda_r <= dat[7-i];
						end
					end
				end
				if ( ! rst ) begin
					@(negedge scl or posedge rst) begin
						if ( ! rst ) begin
							sda_r <= 1'b1;
							@(negedge scl or posedge rst)
								;
						end
					end
				end
				if ( rst ) begin
					sda_r <= 1'b1;
					state <= ST_IDLE;
				end
			end

			ST_ADDR,
				// START detected
			ST_READ: begin
				for ( i=0; i<8; i=i+1 ) begin
					@(posedge scl or rst) begin
						if ( rst != 0 ) begin
							i=8;
						end else begin
							sda_l = sda;
							dat <= { dat[6:0], sda_l };
						end
					end
				end
				if ( ! rst ) begin
					if ( state != ST_ADDR || (dat & 8'h3f) == MYADDR ) begin
						@(negedge scl or rst) begin
							if ( ! rst ) begin
								sda_r <= 0;
								@(negedge scl or rst) begin
									sda_r <= 1;
									if ( ! rst && state == ST_ADDR ) begin
										if ( sda_l == 0 ) begin
											state <= ST_READ;
										end else begin
											state <= ST_WRTE;
											dat   <= 8'h5a;
										end
									end
								end
							end
						end
					end else begin
						/* state == ST_ADDR && dat != MYADDR */	
						state <= ST_IDLE;
					end	
				end
				if ( rst ) begin
					rst   <= 0;
					state <= ST_IDLE;
				end
			end

			default: state <= ST_IDLE; 
		endcase
	end
`endif
endmodule
`endif

`ifdef COMM

`define CB_STRT 0
`define CB_STOP 1
`define CB_READ 2
`define CB_WRTE 3
`define CB_NACK 4

`define C_STRT   (1<<`CB_STRT)
`define C_STOP   (1<<`CB_STOP)
`define C_READ   (1<<`CB_READ)
`define C_WRTE   (1<<`CB_WRTE)
`define C_NACK   (1<<`CB_NACK)
`define C_CLRS   0

`define SB_BSY 0
`define SB_ERR 1
`define SB_ALO 2
`define SB_ACK 3

`define S_BSY (1<<`SB_BSY)
`define S_ERR (1<<`SB_ERR)
`define S_ALO (1<<`SB_ALO)
`define S_ACK (1<<`SB_ACK)

`define C_SZ 5
`define S_SZ 4

module i2c_m(status, dat_out, clk, sda, scl, cmd_in, dat_in, ds, rst);
parameter PER=2;
parameter SDA_PER=1;
input clk;
inout sda;
inout scl;
input [7:0]dat_in;
input [`C_SZ-1:0]cmd_in;
input ds;
input rst;
output [`S_SZ-1:0]status;
output [7:0] dat_out;

parameter W_NONE=2'b00;
parameter W_CLKH=2'b01;
parameter W_CLKL=2'b10;
reg [1:0]  wai;

parameter ST_IDLE=3'b000;
parameter ST_BITR=3'b010;
parameter ST_BITW=3'b011;
parameter ST_STRT=3'b100;
parameter ST_STOP=3'b101;
parameter ST_DONE=3'b110;
reg [2:0]  state;
reg [31:0] div;
reg [3:0]  i;
reg sda_r, scl_r;
reg [8:0]  dat;
reg        started;
reg [`S_SZ-1:0]  status;
parameter  GST_IDLE=2'b00;
parameter  GST_STRT=2'b01;
parameter  GST_READ=2'b10;
parameter  GST_WRTE=2'b11;
reg [1:0]  gstat;

reg [`C_SZ-1:0]  cmd;

wire dat_out = dat[8:1];

	pin_drv sda_pin(sda, sda_r);
	pin_drv scl_pin(scl, scl_r);

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			state <= ST_IDLE;
			wai   <= W_NONE;
			div   <= 0;
			sda_r <= 1;
			scl_r <= 1;
			status<= 0;
			gstat <= GST_IDLE;
		end else if ( ! status[`SB_BSY] && ds ) begin
			if ( cmd_in == `C_CLRS ) begin
				/* clear status */
				status <= 0;
			end else begin
				/* If no start then set state to ST_BITW/ST_BITR */
				/* LSB is ACK bit and must be set when writing */
				/* all bits must be set (in dat) when reading */
				/* sanity checks
					- must START if we are currently idle
					- cannot READ while WRITing and vice versa -- unless restart
					- cannot READ and WRITE simultaneously
					- cannot READ just after START (must address)
				 */
				if ( ( ! cmd_in[`CB_STRT] && (
						gstat == GST_IDLE
					 || (gstat == GST_WRTE && cmd_in[`CB_READ])
					 || (gstat == GST_READ && cmd_in[`CB_WRTE])
					 ) )
					 || (cmd_in[`CB_READ] && cmd_in[`CB_WRTE])
					 || ((cmd_in[`CB_STRT] || gstat == GST_STRT) && cmd_in[`CB_READ])
				   ) begin
					status <= `S_ERR;
				end else begin
					status <= `S_BSY;
					div    <= PER;
					if ( cmd_in[`CB_WRTE] ) begin
						dat    <= {dat_in, 1'b1};
						state  <= ST_BITW;
					end else if ( cmd_in[`CB_READ] ) begin
						dat    <= { 8'hff, cmd_in[`CB_NACK] };
						state  <= ST_BITR;
					end
					if ( cmd_in[`CB_STRT] ) begin
						gstat  <= GST_STRT; /* must send address */
						state  <= ST_STRT;
						if ( gstat != GST_IDLE ) begin
							/* restart */
							scl_r  <= 1;
							wai    <= W_CLKH;
						end else begin
							sda_r  <= 0;
						end
					end else if ( (cmd_in & ~`C_NACK) == `C_STOP ) begin
						/* only STOP */
						state  <= ST_STOP;
						sda_r  <= 0;
					end
					if ( cmd_in[`CB_WRTE] && (cmd_in[`CB_STRT] || gstat == GST_STRT) ) begin
						gstat <= dat_in[0] ? GST_READ : GST_WRTE;
					end
					cmd    <= cmd_in;
					i      <= 8;
				end
			end
		end else begin

			case ( wai )
				W_CLKH:
					if ( scl == 1 ) begin
						wai <= W_NONE;
						if ( sda_r && !sda && state != ST_BITR ) begin
							$display("Arbitration lost\n");
							/* this clears the busy flag also */
							status <= (`S_ERR | `S_ALO);
							/* arbitration lost */
							/* sda_r and scl_r are both 1 at this point */
							state <= ST_IDLE;
							gstat <= GST_IDLE;
						end else begin
							if ( state == ST_BITW || state == ST_BITR ) begin
								if ( i > 0 ) begin
									i <= i - 1;
								end else if ( i == 0 ) begin
									state <= ST_DONE;
								end
								if ( state == ST_BITR ) begin
									dat[i] <= sda;
								end
							end else if ( state == ST_STRT ) begin
								/* restart - now that scl is high we may pull sda low */
								sda_r = 0;
							end
						end
					end

				W_CLKL:
					if ( scl == 0 ) begin
						wai <= W_NONE;
						case ( state )
							ST_BITW: begin
								sda_r <= dat[i];
								if ( i == 0 )
									state <= ST_BITR;
							end

							ST_BITR: begin
								sda_r <= dat[i];
								if ( i == 0 )
									state <= ST_BITW;
							end

							ST_DONE: begin
								/* If we should generate stop */	
								if ( cmd[`CB_STOP] ) begin
									sda_r <= 0;
									state <= ST_STOP;
									div   <= PER;
								end else begin
									sda_r <= 1;
									state <= ST_IDLE;
									status[`SB_BSY] <= 0;
									status[`SB_ACK] <= ~dat[0];
								end
							end

							default: /* never reached */;
						endcase
					end

				default:
					if ( div > 0 ) begin
						div <= div - 1;
						if ( div == 1 ) begin
							case ( state )
								ST_STRT:
								begin
									scl_r <= 0;
									wai   <= W_CLKL;
									div   <= PER;
									if ( cmd[`CB_WRTE] )
										state <= ST_BITW;
									else if ( cmd[`CB_READ] )
										state <= ST_BITR;
									else
										state <= ST_DONE;
								end

								ST_BITR, ST_BITW:
								begin
									scl_r <= ~scl_r;
									div   <= PER;
									wai   <= scl_r ? W_CLKL : W_CLKH;
								end

								ST_DONE:
								begin
									scl_r <= 0;
									wai   <= W_CLKL;
								end

								ST_STOP:
								begin
									if ( scl_r == 0 ) begin
										scl_r <= 1;
										wai   <= W_CLKH;
										div   <= PER;
									end else begin
										sda_r <= 1;
										div   <= SDA_PER;
										wai   <= W_CLKH; /* delay a little bit and check SDA */
										state <= ST_IDLE;
										gstat <= GST_IDLE;
										status[`SB_BSY] <= 0;
										status[`SB_ACK] <= ~dat[0];
									end
								end

								default:
									;
							endcase
						end
					end
			endcase
		end
	end
endmodule

`endif

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
    i2c_master  mst(
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
		#1 rst = 0;
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
