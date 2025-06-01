`include "m_definitions.svh"

module riscv_m_unit(
	input logic clk, 
	input logic resetn, 
	
	input logic valid,
	input logic[31:0] instruction,
	input logic[31:0] rs1,
	input logic[31:0] rs2,

	output logic wr,
	output logic[31:0] rd,
	output logic busy,
	output logic ready
	);

//// DATA SIGNALS
logic [31:0] rs1_neg, rs2_neg;

// ALU inputs
logic [31:0] R; // remainder
logic [62:0] D; // divisor
logic [31:0] Z; // quotient
logic signed [32:0] A;
logic signed [32:0] B;
// ALU outputs
logic sub_neg;
logic [31:0] sub_result;  // subtraction's result
logic [31:0] div_rem;     // divider's result
logic [31:0] div_rem_neg; // divider's result inverted
logic signed [65:0] alu_out;     // multiplier's result

//// CONTROL SIGNALS
logic [`MUX_R_LENGTH-1:0] mux_R;
logic [`MUX_D_LENGTH-1:0] mux_D;
logic [`MUX_Z_LENGTH-1:0] mux_Z;
logic [`MUX_A_LENGTH-1:0] mux_A;
logic [`MUX_B_LENGTH-1:0] mux_B;
logic [`MUX_DIV_REM_LENGTH-1:0] mux_div_rem;
logic [`MUX_OUT_LENGTH-1:0] mux_out;
logic [`MUX_ALUOUT_LENGTH-1:0] mux_aluout;

// this line should be removed after controller is updated with custom instructions
assign mux_aluout = `MUX_ALUOUT_MULT;


//// SUB-BLOCK INSTANTIATION

// CONTROLLER
m_controller controller (
    .clk(clk), .resetn(resetn), .pcpi_valid(valid), // control input
    .instruction(instruction), .rs1(rs1), .rs2(rs2), // data inputs
    .mux_R(mux_R), .mux_D(mux_D), .mux_Z(mux_Z), .mux_A(mux_A), .mux_B(mux_B), // control inputs
    .mux_div_rem(mux_div_rem), .mux_out(mux_out), .pcpi_ready(ready), .pcpi_wr(wr), .pcpi_busy(busy), // control inputs
    .rs1_neg(rs1_neg), .rs2_neg(rs2_neg) // data outputs
);


// REGISTER FILE
m_registers registers(
    .clk(clk), .resetn(resetn), .mux_A(mux_A), .mux_B(mux_B), .sub_neg(sub_neg), .mux_R(mux_R), .mux_D(mux_D), .mux_Z(mux_Z), // control inputs
    .rs1(rs1), .rs2(rs2), .rs1_neg(rs1_neg), .rs2_neg(rs2_neg), .sub_result(sub_result), .alu_out(alu_out), // data inputs
    .A(A), .B(B), .R(R), .D(D), .Z(Z) // data outputs
);


// ALU
m_alu alu(
    .mux_div_rem(mux_div_rem), .mux_aluout(mux_aluout), // control inputs
    .R(R), .D(D), .Z(Z), .A(A), .B(B), // data inputs
    .sub_neg(sub_neg), // control outputs
    .sub_result(sub_result), .div_rem(div_rem), .div_rem_neg(div_rem_neg), .alu_out(alu_out) // data outputs
);


// COMBINATORIAL BLOCK
always_comb begin
    // MUX output selection
    unique case (mux_out)
        `MUX_OUT_ZERO:          rd = '0;
        `MUX_OUT_DIV_REM:       rd = div_rem;
        `MUX_OUT_DIV_REM_NEG:   rd = div_rem_neg;
        `MUX_OUT_ALUOUT_LOWER:  rd = product[31:0];
        `MUX_OUT_ALUOUT_UPPER:  rd = product[63:32];
        `MUX_OUT_ALL1:          rd = {32{1'b1}};
        `MUX_OUT_MINUS_1:       rd = -32'd1;
    endcase
end



endmodule
