`include "../soc/m_definitions.svh"

`timescale 1ns / 1ns

int number_failures = 0;
int number_tests = 0;

module m_unit_tb;
    timeunit 1ns / 1ns;
    reg clk;
    reg resetn;
    reg m_valid;
	reg[31:0] m_instruction;
	reg[31:0] m_rs1;
	reg[31:0] m_rs2;

    reg m_wr;
	reg[31:0] m_rd;
    reg m_busy;
    reg m_ready;


    riscv_m_unit m_unit
    (
	.clk	(clk),
	.resetn	(resetn),
    .valid  (m_valid),
    .instruction (m_instruction),
    .rs1    (m_rs1),
    .rs2    (m_rs2),

    .wr (m_wr),
    .rd (m_rd),
    .busy (m_busy),
    .ready (m_ready)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    task test(func3 instruction,input [31:0] rs1,input [31:0] rs2,input [31:0] expected);
        begin
            @(posedge clk);
            m_valid <= '1;
            m_instruction <= {16'h0200,1'b0,instruction,12'h033};
            m_rs1 <= rs1;
            m_rs2 <= rs2;

            @(posedge clk);
            m_valid <= '0;
            wait(m_ready);

            if (m_rd == expected) begin
		$display("Passed: insn %h, rs1 %h, rs2 %h, Result %h", instruction, rs1, rs2, expected);
            end else begin
		$display("Failed: insn %h, rs1 %h, rs2 %h, Expected %h, Calculated %h", instruction, rs1, rs2, expected, m_rd);
            end
            @(posedge clk);
        end
    endtask

    task test_general(input logic [6:0] opcode, input func3 func3);
    begin
        logic [31:0] expected, rs1, rs2;
        logic [63:0] intermediate_result;
        logic signed [32:0] rs1_ext, rs2_ext;
        logic signed [65:0] intermediate_result_ext;

        @(posedge clk);
        if(opcode == OPCODE) begin
            rs1 = $urandom_range({32{1'b1}}, 0);
            rs2 = $urandom_range({32{1'b1}}, 0);
            case (func3)
                MUL: begin
                    intermediate_result = rs1*rs2;
                    expected = intermediate_result[31:0];
                end
                MULH: begin
                    intermediate_result = $signed(rs1)*$signed(rs2);
                    expected = intermediate_result[63:32];
                end
                MULHSU: begin
                    rs1_ext = {rs1[31],rs1};
                    rs2_ext = {1'b0,rs2};
                    intermediate_result_ext = rs1_ext*rs2_ext;
                    expected = {intermediate_result_ext[65],intermediate_result_ext[62:32]};
                end
                MULHU: begin
                    intermediate_result = rs1*rs2;
                    expected = intermediate_result[63:32];
                end
                DIV: begin
                    expected = $signed(rs1)/$signed(rs2);
                end
                DIVU: begin
                    expected = rs1/rs2;
                end
                REM: begin
                    expected = $signed(rs1)%$signed(rs2);
                end
                REMU:  begin
                    expected = rs1%rs2;
                end
                default: begin
                    expected = '0;
                end
            endcase
        end else if (opcode == OPCODE_CUSTOM) begin
            case (func3)
                ADDMOD: begin
                    rs1 = $urandom_range(`Q-1);
                    rs2 = $urandom_range(`Q-1);
                    expected = (rs1 + rs2) % `Q;
                end
                SUBMOD: begin
                    rs1 = $urandom_range(`Q-1);
                    rs2 = $urandom_range(`Q-1);
                    expected = $signed(rs1 - rs2) % `Q; // might generate the negative result
                    if (expected[31]) expected = $signed(expected) + `Q; // convert to positive by adding Q
                end
                MODQ: begin
                    rs1 = $urandom_range({32{1'b1}}, 0);
                    rs2 = {18'd0,Q_LOGIC};
                    expected = rs1 % `Q;
                end 
                default: begin
                    $error("Wrong func3");
                end
            endcase
        end else begin
            $error("Wrong opcode");
        end
        m_valid <= '1;
        m_instruction <= {16'h0200,1'b0,func3,5'd0,opcode};
        m_rs1 <= rs1;
        m_rs2 <= rs2;

        @(posedge clk);
        m_valid <= '0;
        wait(m_ready);

        if (m_rd == expected) begin
		    //$display("Passed at %08d: insn DIVU, rs1 %h, rs2 %h, Result %h", $time, m_rs1, m_rs2, expected);
        end else begin
		    $display("Failed at %08d: instn %s, rs1 %h, rs2 %h, Expected %h, Calculated %h", $time, get_operation_name(opcode, func3), m_rs1, m_rs2, expected, m_rd);
            number_failures++;
        end
        number_tests++;
        @(posedge clk);
    end
    endtask

    task test_ADDMOD();
    begin
        test_general(OPCODE_CUSTOM, ADDMOD);
    end
    endtask

    task test_SUBMOD();
    begin
        test_general(OPCODE_CUSTOM, SUBMOD);
    end
    endtask

    task test_MODQ();
    begin
        test_general(OPCODE_CUSTOM, MODQ);
    end
    endtask

    initial begin
        resetn = 0;
        //m_valid = 0;
        //m_instruction = 0;
        //m_rs1 = 0;
        //m_rs2 = 0;

        @(posedge clk);
        resetn = 0;
        @(posedge clk);
        resetn = 1;
        @(posedge clk);

        $display("Testing MUL");
        // MUL (lower bits)
        test(MUL, 32'h1111FFFF, 32'h1111FFFF, 32'hDDDC0001); // MUL 1111FFFF * 1111FFFF = 01236543DDDC0001
        test(MUL, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000001); // MUL 4294967295 * 4294967295 = 18446744065119617025
        test(MUL, 32'h00000002, 32'hFFFFFFFF, 32'hFFFFFFFE); // MULH 2 * -1 = -1

        $display("Testing MULH");
        // MULH (upper bits)
        test(MULH, 32'hFFFFFFFB, 32'hFFFFFFFC, 32'h00000000); // MULH -5 * -4 = 20 // MSB 32 Bits 0
        test(MULH, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'h00000000); // MULH -1 * -1 = 1
        test(MULH, 32'h00000002, 32'hFFFFFFFF, 32'hFFFFFFFF); // MULH 2 * -1 = -1

        $display("Testing MULHSU");
        // MULHSU (upper bits)
        test(MULHSU, 32'hFFFFFFFB, 32'h00000004, 32'hFFFFFFFF); // MULHSU -5 * 4 = -20 // MSB 32 Bits 1
        test(MULHSU, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'hFFFFFFFF); // MULHSU -1 * 4294967295 = -4294967295

        $display("Testing MULHU");
        // MULHU (upper bits)
        test(MULHU, 32'h1111FFFF, 32'h1111FFFF, 32'h01236543); // MULHU 1111FFFF * 1111FFFF = 01236543DDDC0001
        test(MULHU, 32'hFFFFFFFF, 32'hFFFFFFFF, 32'hFFFFFFFE); // MULHU 4294967295 * 4294967295 = 18446744065119617025

        $display("Testing DIV");
        // DIV
        test(DIV, 32'hFFFFFFF3, 32'h00000005, 32'hFFFFFFFE); // DIV -13 / 5 = -2
        test(DIV, 32'h00000005, 32'hFFFFFFF3, 32'h00000000); // DIV 5 / -13 = 0
        test(DIV, 32'hFFFFFFF3, 32'h00000000, 32'hFFFFFFFF); // DIV -13 / 0 = -1              DIV BY 0
        test(DIV, 32'h00000005, 32'h00000000, 32'hFFFFFFFF); // DIV   5 / 0 = -1              DIV BY 0
        test(DIV, 32'h80000000, 32'hFFFFFFFF, 32'h80000000); // DIV h80000000 / -1 = 80000000 Overflow
        test(DIV, 32'd34, 32'd23, 32'd1);
        test(DIV, -32'd34, 32'd23, -32'd1);
        test(DIV, 32'd34, -32'd23, -32'd1);
        test(DIV, -32'd34, -32'd23, 32'd1);

        $display("Testing DIVU");
        // DIVU
        test(DIVU, 32'h0000000D, 32'h00000005, 32'h00000002); // DIVU 13 / 5 = 2
        test(DIVU, 32'h00000005, 32'h0000000D, 32'h00000000); // DIVU 5 / 13 = 0
        test(DIVU, 32'h0000000D, 32'h00000000, 32'hFFFFFFFF); // DIVU 13 / 0 = MAX             DIV BY 0
        test(DIVU, 32'd34, 32'd23, 32'd1);
        test(DIVU, '0, 32'd74, '0);
        test(DIVU, 32'hFFFFFFFF, '0, 32'hFFFFFFFF);

        $display("Testing REM");
        // REM
        test(REM, 32'hFFFFFFF3, 32'h00000005, 32'hFFFFFFFD); // REM -13 % 5 = -3
        test(REM, 32'h00000005, 32'hFFFFFFF3, 32'h00000005); // REM 5 % -13 = 5
        test(REM, 32'hFFFFFF30, 32'h00003001, 32'hFFFFFF30); // REM hFFFFFF30 % h3001 = hFFFFFF30
        test(REM, 32'hFFFFFFF3, 32'h00000000, 32'hFFFFFFF3); // REM -13 % 0 = -13             DIV BY 0
        test(REM, 32'h80000000, 32'hFFFFFFFF, 32'h00000000); // REM h80000000 / -1 = 0        Overflow

        $display("Testing REMU");
        // REMU
        test(REMU, 32'h0000000D, 32'h00000005, 32'h00000003); // REMU 13 % 5 = 3
        test(REMU, 32'h00000005, 32'h0000000D, 32'h00000005); // REMU 5 % 13 = 5
        test(REMU, 32'h00003000, 32'h00003001, 32'h00003000); // REMU h3000 % h3001 = h3000
        test(REMU, 32'h0000000D, 32'h00000000, 32'h0000000D); // REMU 13 % 0 = 13              DIV BY 0

        $display("Testing ADDMOD");
        for (int i=0; i<100; i++) begin
            test_ADDMOD();
        end

        $display("Testing SUBMOD");
        for (int i=0; i<100; i++) begin
            test_SUBMOD();
        end

        $display("Testing MODQ");
        for (int i=0; i<100; i++) begin
            test_MODQ();
        end

        $display("Number of failed tests: %d", number_failures);
        $display("Total number of tests:  %d", number_tests);
        $stop;

	$stop;
    end
endmodule
