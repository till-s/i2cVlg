module i2c_slave(clk, scl, sda, sda_out, act_out, rs_out, dat_in, ws_out, dat_out, as_out, rst_in);

parameter MYADDR=(7'h3b);

input clk; /* for resync if necessary */
input scl, sda;
output sda_out;
output act_out;
output rs_out;
input  [7:0]dat_in;
output ws_out;
output [7:0]dat_out;
output as_out;
input     rst_in;

reg       rst;
reg [7:0] dat;
parameter N_DAT=4;
reg sda_r;
reg ack;
reg act_out;
reg rs_out;
reg ws_out;
reg as_out;
parameter ST_IDLE=3'b000;
parameter ST_ADDR=3'b001;
parameter ST_READ=3'b010;
parameter ST_WRTE=3'b011;
reg [2:0]  state;
reg [3:0]  i;

wire [7:0] dat_out = dat;
wire       sda_out = sda_r;

	// reset
	always @(posedge rst_in) begin
		rst <= 1;
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

	always @(posedge scl) begin
		if ( i >= 8 )
			i <= 0;
		else
			i <= i + 1;
	end

	// take away read/write strobe after one pulse 
	always @(posedge scl) begin
		if ( rs_out ) rs_out <= 0;
		if ( ws_out ) ws_out <= 0;
		if ( as_out ) as_out <= 0;
	end

	always @(scl or posedge rst) begin
		if ( rst == 1 ) begin
			state   <= ST_IDLE;
			rst     <= 0;
			sda_r   <= 1;
			act_out <= 0;
			rs_out  <= 0;
			ws_out  <= 0;
		end else begin
			case ( state ) 
				ST_WRTE:
					if ( scl == 0 ) begin
						if ( i == 0 ) begin
							if ( ack ) begin
								dat   <= dat_in;
								sda_r <= dat_in[7];
							end
						end else if ( i < 8 ) begin
							sda_r <= dat[7 - i];
						end else begin
							sda_r <= 1;
						end
					end else begin
						if ( i == 8 ) begin
							ack    <= !sda;	
							rs_out <= !sda;
						end 
					end

				ST_ADDR,
				ST_READ:
					if ( scl == 0 ) begin
						if ( i == 0 ) begin
							/* release SDA during ACK (and after START) */
							sda_r <= 1;
						end else if ( i == 8 ) begin
							if ( state != ST_ADDR || (dat[7:1] == MYADDR) ) begin
								sda_r <= 1'b0; /* ACK */
								if ( state == ST_ADDR ) begin
									if ( dat[0] ) begin
										/* on the next rising edge of SCL: i==8 and the code above
										 * will raise ack
										 */
										state <= ST_WRTE;
									end else begin
										state <= ST_READ;
									end
								end
							end else begin
								/* NO ACK */
								state <= ST_IDLE;
							end
						end
					end else begin
						/* shift data in on positive clock edge */
						if ( i < 8 ) begin
							dat <= {dat[6:0], sda};
						end else begin
							/* raise write strobe during ACK cycle */
							if ( state == ST_ADDR )
								as_out <= 1;
							else
								ws_out <= 1;
						end
					end

				default: state = ST_IDLE;
			endcase
		end
	end
endmodule
