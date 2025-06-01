`include "m_definitions.svh"

module m_alu(
    // CONTROL INPUTS
    //input logic clk, resetn,
    //input logic [`MUX_A_LENGTH-1:0]   mux_A,
    //input logic [`MUX_B_LENGTH-1:0]   mux_B,
    input logic [`MUX_DIV_REM_LENGTH-1:0] mux_div_rem,
    input logic [ `MUX_ALUOUT_LENGTH-1:0] mux_aluout,
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



//// ALU/MULTIPLIER

logic is_sub;
logic signed [16:0] sum_a, sum_b, sum_Q;
logic signed [17:0] sum;
logic signed [18:0] sum_corrected;
logic sum_greater_Q, sub_custom_neg;
logic signed [65:0] product;

// Instantiate adder and multiplier
always_comb begin
    sum = '0; // initialize sum
    // determine whether the operation is subtraction or not (addition assumed otherwise)
    is_sub = (mux_aluout==`MUX_ALUOUT_SUBTR);
    // format operands with signal
    sum_a = {A[32],A[14:0],is_sub};
    sum_b = {B[32],B[14:0],1'b0};
    // perform sum/subtraction
    sum = sum_a + (is_sub ? ~sub_b : sub_b);
    sum[0] = 1'b1; // set this bit in case a further subtraction is needed
    // If sum, subtract Q (add -Q). If subtraction, add Q
    sum_Q[16:1] = is_sub ? {2'b00, Q_LOGIC} : {2'b11, ~Q_LOGIC};
    sum_Q[0] = ~is_sub;
    // Perform additional subtraction/sum
    sum_corrected = sum + sum_Q; 

    // Perform multiplication
    product = A * B; // Perform multiplication

    // Determine which add/sub result should be passed to the output (mod operation)
    sum_greater_Q = sum[17:1] > {3'b000, Q_LOGIC};
    sub_custom_neg = sum[17];
    // Assign outputs
    case (mux_aluout)
        `MUX_ALUOUT_MULT:  alu_out = product;
        `MUX_ALUOUT_ADDER: alu_out = sum_greater_Q  ? {48'd0, sum_corrected[18:1]} : {49'd0, sum[17:1]};
        `MUX_ALUOUT_SUBTR: alu_out = sub_custom_neg ? {48'd0, sum_corrected[18:1]} : {49'd0, sum[17:1]};
        default:           alu_out = product;
    endcase
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