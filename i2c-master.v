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

module i2c_master(clk, sda, sda_out, scl, scl_out, cmd, stat_out, dat, dat_out, ws, rst);
parameter PER=2;
parameter SDA_PER=1;
input clk;
inout sda;
inout scl;
input [7:0]dat;
input [`C_SZ-1:0]cmd;
input ws;
input rst;
output [`S_SZ-1:0]stat_out = status;
output [7:0]      dat_out  = dat_r[8:1];

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
reg [8:0]  dat_r;
reg        started;
reg [`S_SZ-1:0]  status;
parameter  GST_IDLE=2'b00;
parameter  GST_STRT=2'b01;
parameter  GST_READ=2'b10;
parameter  GST_WRTE=2'b11;
reg [1:0]  gstat;

reg [`C_SZ-1:0]  cmd_r;

	always @(posedge clk or posedge rst) begin
		if ( rst != 0 ) begin
			state <= ST_IDLE;
			wai   <= W_NONE;
			div   <= 0;
			sda_r <= 1;
			scl_r <= 1;
			status<= 0;
			gstat <= GST_IDLE;
		end else if ( ! status[`SB_BSY] && ws ) begin
			if ( cmd == `C_CLRS ) begin
				/* clear status */
				status <= 0;
			end else begin
				/* If no start then set state to ST_BITW/ST_BITR */
				/* LSB is ACK bit and must be set when writing */
				/* all bits must be set (in dat_r) when reading */
				/* sanity checks
					- must START if we are currently idle
					- cannot READ while WRITing and vice versa -- unless restart
					- cannot READ and WRITE simultaneously
					- cannot READ just after START (must address)
				 */
				if ( ( ! cmd[`CB_STRT] && (
						gstat == GST_IDLE
					 || (gstat == GST_WRTE && cmd[`CB_READ])
					 || (gstat == GST_READ && cmd[`CB_WRTE])
					 ) )
					 || (cmd[`CB_READ] && cmd[`CB_WRTE])
					 || ((cmd[`CB_STRT] || gstat == GST_STRT) && cmd[`CB_READ])
				   ) begin
					status <= `S_ERR;
				end else begin
					status <= `S_BSY;
					div    <= PER;
					if ( cmd[`CB_WRTE] ) begin
						dat_r  <= {dat, 1'b1};
						state  <= ST_BITW;
					end else if ( cmd[`CB_READ] ) begin
						dat_r  <= { 8'hff, cmd[`CB_NACK] };
						state  <= ST_BITR;
					end
					if ( cmd[`CB_STRT] ) begin
						gstat  <= GST_STRT; /* must send address */
						state  <= ST_STRT;
						if ( gstat != GST_IDLE ) begin
							/* restart */
							scl_r  <= 1;
							wai    <= W_CLKH;
						end else begin
							sda_r  <= 0;
						end
					end else if ( (cmd& ~`C_NACK) == `C_STOP ) begin
						/* only STOP */
						state  <= ST_STOP;
						sda_r  <= 0;
					end
					if ( cmd[`CB_WRTE] && (cmd[`CB_STRT] || gstat == GST_STRT) ) begin
						gstat <= dat[0] ? GST_READ : GST_WRTE;
					end
					cmd_r  <= cmd;
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
									dat_r[i] <= sda;
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
								sda_r <= dat_r[i];
								if ( i == 0 )
									state <= ST_BITR;
							end

							ST_BITR: begin
								sda_r <= dat_r[i];
								if ( i == 0 )
									state <= ST_BITW;
							end

							ST_DONE: begin
								/* If we should generate stop */	
								if ( cmd_r[`CB_STOP] ) begin
									sda_r <= 0;
									state <= ST_STOP;
									div   <= PER;
								end else begin
									sda_r <= 1;
									state <= ST_IDLE;
									status[`SB_BSY] <= 0;
									status[`SB_ACK] <= ~dat_r[0];
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
									if ( cmd_r[`CB_WRTE] )
										state <= ST_BITW;
									else if ( cmd_r[`CB_READ] )
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
										status[`SB_ACK] <= ~dat_r[0];
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
