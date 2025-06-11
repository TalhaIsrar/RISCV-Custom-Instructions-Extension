`include "m_definitions.svh"

module m_registers(
    // CONTROL INPUTS
    input logic clk, resetn,
    input logic [`MUX_A_LENGTH-1:0]   mux_A,
    input logic [`MUX_B_LENGTH-1:0]   mux_B,
    //input logic ALU_neg, // whether the result in the ALU is negative. Not needed
    input logic sub_neg,
    input logic [`MUX_R_LENGTH-1:0] mux_R, // multiplexer selection for remainder
    input logic [`MUX_D_LENGTH-1:0] mux_D, // multiplexer selection for divisor
    input logic [`MUX_Z_LENGTH-1:0] mux_Z, // multiplexer selection for quotient
    // DATA INPUTS
    input logic [31:0] rs1, rs2, // registers at the input
    input logic [31:0] rs1_neg, rs2_neg,
    input logic [31:0] sub_result, // result from the subtractor
    input logic [65:0] alu_out,
    // CONTROL OUTPUTS
    // DATA OUTPUTS
    output logic signed [32:0] A,
    output logic signed [32:0] B,
    output logic [31:0] R, // remainder
    output logic [62:0] D, // divisor
    output logic [31:0] Z  // quotient
);


// REGISTERS
logic [31:0] next_R; // remainder
logic [62:0] next_D; // divisor
logic [31:0] next_Z; // quotient
logic [65:0] next_alu; // product
logic [65:0] alu; // product
logic signed [32:0] next_A, next_B; // operands of ALU

// SEQUENTIAL BLOCK
// All registers are updated
always_ff @(posedge clk) // Synchronous reset, match registers inside DSP
begin
    if(~resetn)
    begin
        R <= '0;
        D <= '0;
        Z <= '0;
        alu <= '0;
        A <= '0;
        B <= '0;
    end
    else
    begin
        R <= next_R;
        D <= next_D;
        Z <= next_Z;
        alu <= next_alu;
        A <= next_A;
        B <= next_B;
    end
end


// COMBINATORIAL BLOCK
// Update registers according to selection signals activated
always_comb
begin
    // Default values are values already saved in registers (redundant to avoid latches)
    next_R = R;
    next_D = D;
    next_Z = Z;
    next_alu = alu_out;
    next_A = {1'b0 , R};
    next_B = {1'b0 , D[62:31]};

    unique case (mux_R)
        `MUX_R_KEEP:       next_R = R;
        `MUX_R_A:          next_R = rs1;
        `MUX_R_A_NEG:      next_R = rs1_neg;
        `MUX_R_SUB_KEEP:   next_R = sub_neg ? R : sub_result;
        `MUX_R_MULT_LOWER: next_R = alu[31:0];
    endcase;

    unique case (mux_D)
        `MUX_D_KEEP:  next_D = D;
        `MUX_D_B:     next_D = {rs2,31'd0};
        `MUX_D_B_NEG: next_D = {rs2_neg,31'd0};
        `MUX_D_SHR:   next_D = {1'b0,D[62:1]};
        `MUX_D_Q:     next_D = {36'd0,Q_LOGIC,13'd0};
    endcase

    unique case (mux_Z)
        `MUX_Z_KEEP:    next_Z = Z;
        `MUX_Z_ZERO:    next_Z = '0;
        `MUX_Z_SHL_ADD: begin
            next_Z[31:1] = Z[30:0];
            next_Z[0]    = sub_neg ? 1'b0 : 1'b1;
        end
        `MUX_Z_MULT_UPPER: next_Z = ((mux_A == `MUX_A_R_SIGNED || mux_B == `MUX_B_D_SIGNED)) ? {alu[65], alu[62:32]} : alu[63:32];
    endcase

    unique case (mux_A)
        `MUX_A_R_UNSIGNED: next_A[32] = 1'b0;  // add 0 to the left
        `MUX_A_R_SIGNED  : next_A[32] = R[31]; // extend bit sign
        `MUX_A_ZERO      : next_A     = 33'd0; // make it 0
    endcase
    unique case (mux_B)
        `MUX_B_D_UNSIGNED: next_B[32] = 1'b0;  // add 0 to the left
        `MUX_B_D_SIGNED  : next_B[32] = D[62]; // extend bit sign
        `MUX_B_ZERO      : next_B     = 33'd0; // make it 0
    endcase

end

endmodule