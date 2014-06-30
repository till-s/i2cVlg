

module tri_chain #(
	parameter integer N_IFS = 2
)
(
	input  iic_bus_i,
	output iic_bus_o,
	output iic_bus_t,
	output [N_IFS-1:0] iic_ups_i,
	input  [N_IFS-1:0] iic_ups_o,
	input  [N_IFS-1:0] iic_ups_t);

assign iic_ups_i = { N_IFS { iic_bus_i } };

	assign iic_bus_o = & iic_ups_o;
	assign iic_bus_t = & iic_ups_t;

endmodule

module i2c_chain
(
	input  iic_bus_sda_i,
	output iic_bus_sda_o,
	output iic_bus_sda_t,

	input  iic_bus_scl_i,
    output iic_bus_scl_o,
    output iic_bus_scl_t,
    
    output iic_ups_1_sda_i,
    input  iic_ups_1_sda_o,
    input  iic_ups_1_sda_t,
    output iic_ups_1_scl_i,
    input  iic_ups_1_scl_o,
    input  iic_ups_1_scl_t,
    
    output iic_ups_2_sda_i,
    input  iic_ups_2_sda_o,
    input  iic_ups_2_sda_t,
    output iic_ups_2_scl_i,
    input  iic_ups_2_scl_o,
    input  iic_ups_2_scl_t,    
    
    output iic_ups_3_sda_i,
    input  iic_ups_3_sda_o,
    input  iic_ups_3_sda_t,
    output iic_ups_3_scl_i,
    input  iic_ups_3_scl_o,
    input  iic_ups_3_scl_t
);
localparam N_IFS = 3;

wire [N_IFS-1:0] iic_ups_sda_i;

	assign iic_ups_1_sda_i = iic_ups_sda_i [ 0 ];
	assign iic_ups_2_sda_i = iic_ups_sda_i [ 1 ];
	assign iic_ups_3_sda_i = iic_ups_sda_i [ 2 ];

wire [N_IFS-1:0] iic_ups_sda_o = {
    iic_ups_3_sda_o,
    iic_ups_2_sda_o,
    iic_ups_1_sda_o
};
wire [N_IFS-1:0] iic_ups_sda_t = {
    iic_ups_3_sda_t,
    iic_ups_2_sda_t,
    iic_ups_1_sda_t
};

wire [N_IFS-1:0] iic_ups_scl_i;

	assign iic_ups_1_scl_i = iic_ups_scl_i [ 0 ];
	assign iic_ups_2_scl_i = iic_ups_scl_i [ 1 ];
	assign iic_ups_3_scl_i = iic_ups_scl_i [ 2 ];

wire [N_IFS-1:0] iic_ups_scl_o = {
    iic_ups_3_scl_o,
    iic_ups_2_scl_o,
    iic_ups_1_scl_o
};
wire [N_IFS-1:0] iic_ups_scl_t = {
    iic_ups_3_scl_t,
    iic_ups_2_scl_t,
    iic_ups_1_scl_t
};


    tri_chain #(.N_IFS(N_IFS)) sda_chain(
        .iic_bus_i(iic_bus_sda_i),
        .iic_bus_o(iic_bus_sda_o),
        .iic_bus_t(iic_bus_sda_t),
        .iic_ups_i(iic_ups_sda_i),
        .iic_ups_o(iic_ups_sda_o),
        .iic_ups_t(iic_ups_sda_t)
    );
    
    tri_chain #(.N_IFS(N_IFS)) scl_chain(
        .iic_bus_i(iic_bus_scl_i),
        .iic_bus_o(iic_bus_scl_o),
        .iic_bus_t(iic_bus_scl_t),
        .iic_ups_i(iic_ups_scl_i),
        .iic_ups_o(iic_ups_scl_o),
        .iic_ups_t(iic_ups_scl_t)
    );
endmodule
