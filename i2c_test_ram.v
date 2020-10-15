//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/27/2014 01:53:21 PM
// Design Name: 
// Module Name: i2c_test_ram
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module i2c_test_ram(
    input  clk,
    input  aresetn,
    input  sda_i,
    output sda_o,
    output sda_t,
    input  scl_i,
    output scl_o,
    output scl_t
    );
parameter US        = 100;
parameter I2C_MODE  = 0;
parameter LD_NBYTES = 3;

wire rs,ws;

reg [7:0] mem [ (1<<LD_NBYTES) - 1 : 0 ];

wire [7:0] dat_out;

reg [LD_NBYTES-1:0] idx;
reg as_seen;

    assign sda_o = 0;
    assign scl_o = 0;
    assign scl_t = 1;
    
    i2c_slave #(.US(US),.I2C_MODE(I2C_MODE)) slv(
        .clk(clk),
        .scl_in(scl_i),
        .sda_in(sda_i),
        .sda_out(sda_t),
        .rs_out(rs),
        .ws_out(ws),
        .as_out(as),
        .dat_in( mem[idx] ),
        .dat_out( dat_out ),
        .rst( !aresetn )
        );
        
    always @(posedge clk or negedge aresetn) begin
        if ( ! aresetn ) begin
            idx     <= 0;
            as_seen <= 0;
        end else begin
            if ( as )
                as_seen <= 1;
            if ( rs ) begin
                as_seen <= 0;
                idx     <= idx + 1;
            end else if ( ws ) begin
                /* First write after addressing sets index */
                if ( as_seen ) begin
                    idx      <= dat_out;
               		as_seen  <= 0;
                end else begin
					mem[idx] <= dat_out;
					idx      <= idx + 1;
				end
            end
        end
    end
endmodule
