/*
 * serv_decode.v : SERV module decoding instruction word into control signals
 *
 * SPDX-FileCopyrightText: 2018 Olof Kindgren <olof@award-winning.me>
 * SPDX-License-Identifier: ISC
 */
`default_nettype none
module serv_decode
  #(parameter [0:0] PRE_REGISTER = 1,
    parameter [0:0] MDU = 0,
    /* LINUX=1 adds what an M-mode no-MMU Linux needs (see qerv-linux branch notes):
       A extension (AMO/LR/SC as one dbus transaction carrying funct5, executed outside the
       core), exact 12-bit CSR decode (unimplemented CSRs read 0, writes ignored), mip
       readable, fence/fence.i/wfi as no-ops. LINUX=0 is bit-identical to upstream. */
    parameter [0:0] LINUX = 0,
    /* SMODE=1 (with LINUX=1): exact decode of ecall/ebreak/mret/sret/sfence.vma, the S-mode
       CSRs (S group of the RF CSRs, o_csr_sel for the ones in serv_csr). */
    parameter [0:0] SMODE = 0)
  (
   input wire        clk,
   //Input
   input wire [31:2] i_wb_rdt,
   input wire        i_wb_en,
   //To state
   output reg       o_sh_right,
   output reg       o_bne_or_bge,
   output reg       o_cond_branch,
   output reg       o_e_op,
   output reg       o_ebreak,
   output reg       o_branch_op,
   output reg       o_shift_op,
   output reg       o_rd_op,
   output reg       o_two_stage_op,
   output reg       o_dbus_en,
   //MDU
   output reg       o_mdu_op,
   //A extension (LINUX)
   output reg       o_amo,
   output reg [4:0] o_amo_op,
   //Extension
   output reg [2:0] o_ext_funct3,
   //To bufreg
   output reg       o_bufreg_rs1_en,
   output reg       o_bufreg_imm_en,
   output reg       o_bufreg_clr_lsb,
   output reg       o_bufreg_sh_signed,
   //To ctrl
   output reg       o_ctrl_jal_or_jalr,
   output reg       o_ctrl_utype,
   output reg       o_ctrl_pc_rel,
   output reg       o_ctrl_mret,
   output reg       o_ctrl_sret,
   output reg       o_sfence,
   //To alu
   output reg       o_alu_sub,
   output reg [1:0] o_alu_bool_op,
   output reg       o_alu_cmp_eq,
   output reg       o_alu_cmp_sig,
   output reg [2:0] o_alu_rd_sel,
   //To mem IF
   output reg       o_mem_signed,
   output reg       o_mem_word,
   output reg       o_mem_half,
   output reg       o_mem_cmd,
   //To CSR
   output reg       o_csr_en,
   output reg [1:0] o_csr_addr,
   output reg       o_csr_s,
   output reg [12:0] o_csr_sel,
   output reg       o_csr_mstatus_en,
   output reg       o_csr_mie_en,
   output reg       o_csr_mcause_en,
   output reg       o_csr_mip_en,
   output reg [1:0] o_csr_source,
   output reg       o_csr_d_sel,
   output reg       o_csr_imm_en,
   output reg       o_mtval_pc,
   //To top
   output reg [3:0] o_immdec_ctrl,
   output reg [3:0] o_immdec_en,
   output reg       o_op_b_source,
   //To RF IF
   output reg       o_rd_mem_en,
   output reg       o_rd_csr_en,
   output reg       o_rd_alu_en);

   reg [4:0] opcode;
   reg [2:0] funct3;
   /* Instruction bits 31:20. Upstream only keeps bits 20, 21, 22, 25, 26 and 30; the rest
      are only used when LINUX=1 (CSR address, AMO funct5) and are optimized away otherwise. */
   reg [11:0] imm31_20;
   wire       op20  = imm31_20[0];
   wire       op21  = imm31_20[1];
   wire       op22  = imm31_20[2];
   wire       imm25 = imm31_20[5];
   wire       op26  = imm31_20[6];
   wire       imm30 = imm31_20[10];

   //A extension: AMO opcode 0101111 (opcode[6:2] = 01011)
   wire co_amo    = LINUX & (opcode == 5'b01011);
   wire [4:0] co_amo_op = imm31_20[11:7];
   //MISC-MEM (fence, fence.i): opcode 0001111. Decoded as a one-stage no-op when LINUX=1
   //(upstream treats it as a data bus read that does not write rd)
   wire misc_mem  = LINUX & !opcode[4] & !opcode[3] & !opcode[2] & opcode[0];

   wire co_mdu_op     = MDU & (opcode == 5'b01100) & imm25;

   wire co_two_stage_op =
	(~opcode[2] & !misc_mem) | (funct3[0] & ~funct3[1] & ~opcode[0] & ~opcode[4]) |
	(funct3[1] & ~funct3[2] & ~opcode[0] & ~opcode[4]) | co_mdu_op;
   wire co_shift_op = (opcode[2] & ~funct3[1]) & !co_mdu_op;
   wire co_branch_op = opcode[4];
   wire co_dbus_en    = ~opcode[2] & ~opcode[4] & !misc_mem;
   wire co_mtval_pc   = opcode[4];
   wire co_mem_word   = funct3[1];
   wire co_rd_alu_en  = !opcode[0] & opcode[2] & !opcode[4] & !co_mdu_op;
   wire co_rd_mem_en  = (!opcode[2] & !opcode[0]) | co_mdu_op | co_amo;
   wire [2:0] co_ext_funct3 = funct3;

   //jal,branch =     imm
   //jalr       = rs1+imm
   //mem        = rs1+imm
   //shift      = rs1
   wire co_bufreg_rs1_en = !opcode[4] | (!opcode[1] & opcode[0]);
   //AMO addresses are rs1 without an immediate
   wire co_bufreg_imm_en = !opcode[2] & !co_amo;

   //Clear LSB of immediate for BRANCH and JAL ops
   //True for BRANCH and JAL
   //False for JALR/LOAD/STORE/OP/OPIMM?
   wire co_bufreg_clr_lsb = opcode[4] & ((opcode[1:0] == 2'b00) | (opcode[1:0] == 2'b11));

   //Conditional branch
   //True for BRANCH
   //False for JAL/JALR
   wire co_cond_branch = !opcode[0];

   wire co_ctrl_utype       = !opcode[4] & opcode[2] & opcode[0];
   wire co_ctrl_jal_or_jalr = opcode[4] & opcode[0];

   //PC-relative operations
   //True for jal, b* auipc, ebreak
   //False for jalr, lui
   wire co_ctrl_pc_rel = (opcode[2:0] == 3'b000)  |
                          (opcode[1:0] == 2'b11)  |
                          (opcode[4] & opcode[2]) & op20|
                          (opcode[4:3] == 2'b00);
   //Write to RD
   //True for OP-IMM, AUIPC, OP, LUI, SYSTEM, JALR, JAL, LOAD
   //False for STORE, BRANCH, MISC-MEM
   wire co_rd_op = (opcode[2] |
                     (!opcode[2] & opcode[4] & opcode[0]) |
                     (!opcode[2] & !opcode[3] & !opcode[0]) |
                     co_amo);

   //
   //funct3
   //

   wire co_sh_right   = funct3[2];
   wire co_bne_or_bge = funct3[0];

   //Matches system ops except ecall/ebreak/mret
   wire csr_op = opcode[4] & opcode[2] & (|funct3);


   //op20
   wire co_ebreak = op20;


   //opcode & funct3 & op21

   wire sys0 = opcode[4] & opcode[2] & !(|funct3);
   wire co_ctrl_mret = SMODE ? sys0 & (imm31_20 == 12'h302) : opcode[4] & opcode[2] & op21 & !(|funct3);
   wire co_ctrl_sret = SMODE & sys0 & (imm31_20 == 12'h102);
   //SMODE: sfence.vma (funct7 0001001, any rs1/rs2) runs as a no-op; o_sfence flushes the TLB
   wire co_sfence = SMODE & sys0 & (imm31_20[11:5] == 7'b0001001);
   //Matches system opcodes except CSR accesses (funct3 == 0)
   //and mret (!op21). With LINUX, wfi (0x105, the only one with bit 22 set) is excluded and
   //executes as a no-op instead of trapping like ebreak.
   wire co_e_op = SMODE ? sys0 & (imm31_20[11:1] == 11'd0) :
                  opcode[4] & opcode[2] & !op21 & !(|funct3) & !(LINUX & op22);

   //opcode & funct3 & imm30

   wire co_bufreg_sh_signed = imm30;

   /*
    True for sub, b*, slt*
    False for add*
    op    opcode f3  i30
    b*    11000  xxx x   t
    addi  00100  000 x   f
    slt*  0x100  01x x   t
    add   01100  000 0   f
    sub   01100  000 1   t
    */
   wire co_alu_sub = funct3[1] | funct3[0] | (opcode[3] & imm30) | opcode[4];

   /*
    Bits 26, 22, 21 and 20 are enough to uniquely identify the eight supported CSR regs
    mtvec, mscratch, mepc and mtval are stored externally (normally in the RF) and are
    treated differently from mstatus, mie and mcause which are stored in serv_csr.

    The former get a 2-bit address as seen below while the latter get a
    one-hot enable signal each.

    Hex|2 222|Reg     |csr
    adr|6 210|name    |addr
    ---|-----|--------|----
    300|0_000|mstatus | xx
    304|0_100|mie     | xx
    305|0_101|mtvec   | 01
    340|1_000|mscratch| 00
    341|1_001|mepc    | 10
    342|1_010|mcause  | xx
    343|1_011|mtval   | 11

    */

   /*
    With LINUX, the full 12-bit address is matched so that every other CSR (mhartid, misa,
    pmpaddr0/pmpcfg0, counters, ...) reads as 0 and ignores writes, instead of aliasing one
    of the implemented ones. mip (0x344) is added, read-only.
    */
   wire csr_is_mstatus  = imm31_20 == 12'h300;
   wire csr_is_mie      = imm31_20 == 12'h304;
   wire csr_is_mtvec    = imm31_20 == 12'h305;
   wire csr_is_mscratch = imm31_20 == 12'h340;
   wire csr_is_mepc     = imm31_20 == 12'h341;
   wire csr_is_mcause   = imm31_20 == 12'h342;
   wire csr_is_mtval    = imm31_20 == 12'h343;
   wire csr_is_mip      = imm31_20 == 12'h344;

   //true  for mtvec,mscratch,mepc and mtval
   //false for mstatus, mie, mcause
   //SMODE: stvec 0x105, sscratch 0x140, sepc 0x141, stval 0x143 use the same low address
   //bits as the M ones, in the S group of the RF CSRs (o_csr_s)
   wire csr_is_srf = (imm31_20 == 12'h105) | (imm31_20 == 12'h140) |
                     (imm31_20 == 12'h141) | (imm31_20 == 12'h143);
   wire csr_valid = LINUX ? (csr_is_mtvec | csr_is_mscratch | csr_is_mepc | csr_is_mtval |
                             (SMODE & csr_is_srf)) :
                    (op20 | (op26 & !op21));
   wire co_csr_s = SMODE & csr_is_srf;
   //SMODE: CSRs implemented in serv_csr (gen_smode), one-hot
   wire [12:0] co_csr_sel = {13{SMODE & csr_op}} &
                            {imm31_20 == 12'hC81, imm31_20 == 12'hC01, imm31_20 == 12'h180,
                             imm31_20 == 12'h303, imm31_20 == 12'h302, imm31_20 == 12'h142,
                             imm31_20 == 12'h342, imm31_20 == 12'h144, imm31_20 == 12'h344,
                             imm31_20 == 12'h104, imm31_20 == 12'h304, imm31_20 == 12'h100,
                             imm31_20 == 12'h300};

   wire co_rd_csr_en = csr_op;

   wire co_csr_en         = csr_op & csr_valid;
   wire co_csr_mstatus_en = csr_op & (LINUX ? csr_is_mstatus : (!op26 & !op22 & !op20));
   wire co_csr_mie_en     = csr_op & (LINUX ? csr_is_mie     : (!op26 &  op22 & !op20));
   wire co_csr_mcause_en  = csr_op & (LINUX ? csr_is_mcause  : (         op21 & !op20));
   wire co_csr_mip_en     = csr_op & LINUX & csr_is_mip;

   wire [1:0] co_csr_source = funct3[1:0];
   wire co_csr_d_sel = funct3[2];
   wire co_csr_imm_en = opcode[4] & opcode[2] & funct3[2];
   wire [1:0] co_csr_addr = {op26 & op20, !op26 | op21};

   wire co_alu_cmp_eq = funct3[2:1] == 2'b00;

   wire co_alu_cmp_sig = ~((funct3[0] & funct3[1]) | (funct3[1] & funct3[2]));

   wire co_mem_cmd  = opcode[3];
   wire co_mem_signed = ~funct3[2];
   wire co_mem_half   = funct3[0];

   wire [1:0] co_alu_bool_op = funct3[1:0];

   wire [3:0] co_immdec_ctrl;
   //True for S (STORE) or B (BRANCH) type instructions
   //False for J type instructions
   assign co_immdec_ctrl[0] = opcode[3:0] == 4'b1000;
   //True for OP-IMM, LOAD, STORE, JALR  (I S)
   //False for LUI, AUIPC, JAL           (U J)
   assign co_immdec_ctrl[1] = (opcode[1:0] == 2'b00) | (opcode[2:1] == 2'b00);
   assign co_immdec_ctrl[2] = opcode[4] & !opcode[0];
   assign co_immdec_ctrl[3] = opcode[4];

   wire [3:0] co_immdec_en;
   assign co_immdec_en[3] = opcode[4] | opcode[3] | opcode[2] | !opcode[0];                 //B I J S U
   //AMO (R-type) keeps rs1/rs2 like OP; with W=1 these registers double as rs1/rs2 addresses
   assign co_immdec_en[2] = ((opcode[4] & opcode[2]) | !opcode[3] | opcode[0]) & !co_amo;    //  I J   U
   assign co_immdec_en[1] = ((opcode[2:1] == 2'b01) | (opcode[2] & opcode[0]) | co_csr_imm_en) & !co_amo;//    J   U
   assign co_immdec_en[0] = ~co_rd_op;                                                       //B     S

   wire [2:0] co_alu_rd_sel;
   assign co_alu_rd_sel[0] = (funct3 == 3'b000); // Add/sub
   assign co_alu_rd_sel[1] = (funct3[2:1] == 2'b01); //SLT*
   assign co_alu_rd_sel[2] = funct3[2]; //Bool

   //0 (OP_B_SOURCE_IMM) when OPIMM
   //1 (OP_B_SOURCE_RS2) when BRANCH or OP
   wire co_op_b_source = opcode[3];

   generate
      if (PRE_REGISTER) begin : gen_pre_register

         always @(posedge clk) begin
            if (i_wb_en) begin
               funct3   <= i_wb_rdt[14:12];
               opcode   <= i_wb_rdt[6:2];
               imm31_20 <= i_wb_rdt[31:20];
            end
         end

         always @(*) begin
            o_sh_right         = co_sh_right;
            o_bne_or_bge       = co_bne_or_bge;
            o_cond_branch      = co_cond_branch;
            o_dbus_en          = co_dbus_en;
            o_mtval_pc         = co_mtval_pc;
	    o_two_stage_op     = co_two_stage_op;
            o_e_op             = co_e_op;
            o_ebreak           = co_ebreak;
            o_branch_op        = co_branch_op;
            o_shift_op         = co_shift_op;
            o_rd_op            = co_rd_op;
            o_mdu_op           = co_mdu_op;
            o_amo              = co_amo;
            o_amo_op           = co_amo_op;
            o_ext_funct3       = co_ext_funct3;
            o_bufreg_rs1_en    = co_bufreg_rs1_en;
            o_bufreg_imm_en    = co_bufreg_imm_en;
            o_bufreg_clr_lsb   = co_bufreg_clr_lsb;
            o_bufreg_sh_signed = co_bufreg_sh_signed;
            o_ctrl_jal_or_jalr = co_ctrl_jal_or_jalr;
            o_ctrl_utype       = co_ctrl_utype;
            o_ctrl_pc_rel      = co_ctrl_pc_rel;
            o_ctrl_mret        = co_ctrl_mret;
            o_ctrl_sret        = co_ctrl_sret;
            o_sfence           = co_sfence;
            o_alu_sub          = co_alu_sub;
            o_alu_bool_op      = co_alu_bool_op;
            o_alu_cmp_eq       = co_alu_cmp_eq;
            o_alu_cmp_sig      = co_alu_cmp_sig;
            o_alu_rd_sel       = co_alu_rd_sel;
            o_mem_signed       = co_mem_signed;
            o_mem_word         = co_mem_word;
            o_mem_half         = co_mem_half;
            o_mem_cmd          = co_mem_cmd;
            o_csr_en           = co_csr_en;
            o_csr_addr         = co_csr_addr;
            o_csr_s            = co_csr_s;
            o_csr_sel          = co_csr_sel;
            o_csr_mstatus_en   = co_csr_mstatus_en;
            o_csr_mie_en       = co_csr_mie_en;
            o_csr_mcause_en    = co_csr_mcause_en;
            o_csr_mip_en       = co_csr_mip_en;
            o_csr_source       = co_csr_source;
            o_csr_d_sel        = co_csr_d_sel;
            o_csr_imm_en       = co_csr_imm_en;
            o_immdec_ctrl      = co_immdec_ctrl;
            o_immdec_en        = co_immdec_en;
            o_op_b_source      = co_op_b_source;
            o_rd_csr_en        = co_rd_csr_en;
            o_rd_alu_en        = co_rd_alu_en;
            o_rd_mem_en        = co_rd_mem_en;
         end

      end else begin : gen_post_register

         always @(*) begin
            funct3   = i_wb_rdt[14:12];
            opcode   = i_wb_rdt[6:2];
            imm31_20 = i_wb_rdt[31:20];
         end

         always @(posedge clk) begin
            if (i_wb_en) begin
               o_sh_right         <= co_sh_right;
               o_bne_or_bge       <= co_bne_or_bge;
               o_cond_branch      <= co_cond_branch;
               o_e_op             <= co_e_op;
               o_ebreak           <= co_ebreak;
               o_two_stage_op     <= co_two_stage_op;
               o_dbus_en          <= co_dbus_en;
               o_mtval_pc         <= co_mtval_pc;
               o_branch_op        <= co_branch_op;
               o_shift_op         <= co_shift_op;
               o_rd_op            <= co_rd_op;
               o_mdu_op           <= co_mdu_op;
               o_amo              <= co_amo;
               o_amo_op           <= co_amo_op;
               o_ext_funct3       <= co_ext_funct3;
               o_bufreg_rs1_en    <= co_bufreg_rs1_en;
               o_bufreg_imm_en    <= co_bufreg_imm_en;
               o_bufreg_clr_lsb   <= co_bufreg_clr_lsb;
               o_bufreg_sh_signed <= co_bufreg_sh_signed;
               o_ctrl_jal_or_jalr <= co_ctrl_jal_or_jalr;
               o_ctrl_utype       <= co_ctrl_utype;
               o_ctrl_pc_rel      <= co_ctrl_pc_rel;
               o_ctrl_mret        <= co_ctrl_mret;
               o_ctrl_sret        <= co_ctrl_sret;
               o_sfence           <= co_sfence;
               o_alu_sub          <= co_alu_sub;
               o_alu_bool_op      <= co_alu_bool_op;
               o_alu_cmp_eq       <= co_alu_cmp_eq;
               o_alu_cmp_sig      <= co_alu_cmp_sig;
               o_alu_rd_sel       <= co_alu_rd_sel;
               o_mem_signed       <= co_mem_signed;
               o_mem_word         <= co_mem_word;
               o_mem_half         <= co_mem_half;
               o_mem_cmd          <= co_mem_cmd;
               o_csr_en           <= co_csr_en;
               o_csr_addr         <= co_csr_addr;
               o_csr_s            <= co_csr_s;
               o_csr_sel          <= co_csr_sel;
               o_csr_mstatus_en   <= co_csr_mstatus_en;
               o_csr_mie_en       <= co_csr_mie_en;
               o_csr_mcause_en    <= co_csr_mcause_en;
               o_csr_mip_en       <= co_csr_mip_en;
               o_csr_source       <= co_csr_source;
               o_csr_d_sel        <= co_csr_d_sel;
               o_csr_imm_en       <= co_csr_imm_en;
               o_immdec_ctrl      <= co_immdec_ctrl;
               o_immdec_en        <= co_immdec_en;
               o_op_b_source      <= co_op_b_source;
               o_rd_csr_en        <= co_rd_csr_en;
               o_rd_alu_en        <= co_rd_alu_en;
               o_rd_mem_en        <= co_rd_mem_en;
            end
         end

      end
   endgenerate

endmodule
