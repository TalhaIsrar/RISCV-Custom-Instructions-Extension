`include "m_definitions.svh"

module m_alu(
    // CONTROL INPUTS
    //input logic clk, resetn,
    //input logic [`MUX_A_LENGTH-1:0]   mux_A,
    //input logic [`MUX_B_LENGTH-1:0]   mux_B,
    input logic [`MUX_DIV_REM_LENGTH-1:0] mux_div_rem,
    input logic opcode,
    // DATA INPUTS
    input logic [31:0] R, // remainder
    input logic [62:0] D, // divisor
    input logic [31:0] Z, // quotient
    input logic signed [32:0] A,
    input logic signed [32:0] B,
    // CONTROL OUTPUTS
    output logic sub_neg,
    // DATA OUTPUTS
    output logic [31:0] sub_result,
    output logic [31:0] div_rem,
    output logic [31:0] div_rem_neg,
    output logic signed [65:0] alu_out
);


//// SUBTRACTOR (FOR DIVISION)
// Auxiliary signed values to instantiate signed subtractor
logic signed [63:0] sub_result_sign;
logic [62:0] sub_a, sub_b;

// Instantiate subtractor
assign sub_a = {31'd0,R}; // Add 0 to the left
assign sub_b = D;
assign sub_result_sign = sub_a - sub_b; // Perform subtraction
// concatenation to avoid overwriting bit sign being overwritten
assign sub_result = sub_result_sign[31:0];
assign sub_neg = sub_result_sign[63];



//// MULTIPLIER

// Instantiate multiplier
always_comb begin
    alu_out = A * B; // Perform multiplication
end

//// DIVISION/REMAINDER SELECTION
always_comb begin
    // Select which division result is passed to the output
    unique case(mux_div_rem)
        `MUX_DIV_REM_R: div_rem = R;
        `MUX_DIV_REM_Z: div_rem = Z;
    endcase
    div_rem_neg = -div_rem; // also calculate its inverse, in case it is needed
end


endmodule