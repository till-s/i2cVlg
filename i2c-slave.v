module i2c_slave(clk, scl, sda, sda_out, act_out, rs_out, dat_in, ws_out, dat_out, as_out, rst);

parameter MYADDR=(7'h3b);

`include "i2c-timing-params.vh"

input clk; /* for resync if necessary */
input scl, sda;
output sda_out;
output act_out;
output rs_out;
input  [7:0]dat_in;
output ws_out;
output [7:0]dat_out;
output as_out;
input     rst;

reg [7:0] dat, new_dat;
reg sda_r, new_sda;
reg ack, got_ack;
reg act_out, new_act;
reg rs_out;
reg ws_out;
reg as_out, as_gen;
reg strobe;
localparam ST_IDLE=3'b000;
localparam ST_ADDR=3'b001;
localparam ST_READ=3'b010;
localparam ST_WRTE=3'b011;
reg [2:0]  state, new_state;
reg [3:0]  i;

wire [7:0] dat_out = dat;
wire       sda_out = sda_r;

wire       sta, sto, scl_lohi, scl_hilo;

	i2c_bby_detect #(.US(US),.I2C_MODE(I2C_MODE))  sta_sto_det(.clk(clk), .sda(sda), .scl(scl), .sto(sto), .sta(sta), .rst(rst));

	i2c_edge_detect #(.US(US),.I2C_MODE(I2C_MODE)) scl_det(.clk(clk), .lin(scl), .hilo(scl_hilo), .lohi(scl_lohi), .rst(rst));

	// i counter
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			i <= 0;
		end else begin
			if ( sta ) begin
				 i <= 0;
			end else if ( scl_lohi ) begin
				if ( i >= 8 )
					i <= 0;
				else
					i <= i + 1;
			end
		end
	end

	/* New state computation */
	always @(*) begin

		new_state = state;
		new_sda   = sda_r;
		new_dat   = dat;
		got_ack   = 1'b0;
		strobe    = 1'b0;
		new_act   = 1'b0;

		if ( scl_hilo ) begin
			case ( state ) 
				ST_WRTE:
					if ( i == 0 ) begin
						if ( ack ) begin
							new_sda = dat[7];
						end
					end else if ( i < 8 ) begin
						new_sda = dat[7 - i];
					end else begin
						new_sda = 1'b1;
					end

				ST_ADDR,
				ST_READ:
					if ( i == 0 ) begin
						/* release SDA during ACK (and after START) */
						new_sda = 1'b1;
					end else if ( i == 8 ) begin
						if ( state != ST_ADDR || (dat[7:1] == MYADDR) ) begin
							new_sda = 1'b0; /* ACK */
							if ( state == ST_ADDR ) begin
								new_act = 1'b1;
								if ( dat[0] ) begin
									/* on the next rising edge of SCL: i==8 and the code below
									 * will raise ack
									 */
									new_state = ST_WRTE;
								end else begin
									new_state = ST_READ;
								end
							end
						end else begin
							/* NO ACK */
							new_state <= ST_IDLE;
						end
					end

				default: new_state = ST_IDLE;
			endcase
		end else if ( scl_lohi ) begin
//			new_sda = 1;
			case ( state ) 
				ST_WRTE:
					if ( i == 8 ) begin
						got_ack = !sda;	
						strobe  = 1;
					end

				ST_ADDR,
				ST_READ:
					/* shift data in on positive clock edge */
					if ( i < 8 ) begin
						new_dat = {dat[6:0], sda};
					end else begin
						/* raise write strobe during ACK cycle */
						strobe = 1;
					end

				default: new_state = ST_IDLE;
			endcase
		end
	end

	// ACK state machine
	always @(posedge clk or posedge rst) begin
		if ( rst )
			ack <= 0;
		else begin
			if ( i == 1 || sto || sta ) 
				/* ack remembers if the last byte was ACKed.
				 * Once i==1 we can clear this condition
				 */
				ack <= 0;
			else if ( got_ack )
				ack <= 1'b1;
		end
	end

	// ACT state machine
	always @(posedge clk or posedge rst) begin
		if ( rst )
			act_out <= 0;
		else begin
			if ( sto )
				act_out <= 0;
			if ( new_act )
				act_out <= 1;
		end
	end


	// dat + sda update
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			sda_r <= 1'b1;
		end else begin
			/* clock in on the cycle following read strobe */
			dat   <= rs_out ? dat_in : new_dat;
			sda_r <= new_sda;
		end
	end

	// state machine
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			state <= ST_IDLE;
		end else begin
			if ( sta ) begin
				state <= ST_ADDR;
				if ( state != ST_IDLE ) begin
					$display("Start detector: RESTART from: %x\n", state);
				end 
			end else if ( sto ) begin
				state <= ST_IDLE;
			end else begin
				state <= new_state;
			end
		end
	end

	// strobes
	always @(posedge clk or posedge rst) begin
		if ( rst ) begin
			rs_out <= 0;
			ws_out <= 0;
			as_out <= 0;
			as_gen <= 0;
		end else begin
			rs_out <= 0;
			ws_out <= 0;
			as_out <= 0;
			if ( got_ack )
				rs_out <= 1;
			if ( new_act )
				as_gen <= 1;
			if ( strobe ) begin
				if ( as_gen ) begin
					as_out <= 1;
					as_gen <= 0;
				end else if ( state == ST_READ ) begin
					ws_out <= 1;
				end
			end
		end
	end

endmodule
