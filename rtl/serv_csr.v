/*
 * serv_csr.v : SERV module for handling CSR registers
 *
 * SPDX-FileCopyrightText: 2018 Olof Kindgren <olof@award-winning.me>
 * SPDX-License-Identifier: ISC
 */
`default_nettype none
module serv_csr
  #(
    parameter RESET_STRATEGY = "MINI",
    parameter W = 1,
    parameter B = W-1,
    /* LINUX=1: U-mode (one privilege bit; mstatus.MPP reads 00/11 and is writable),
       mstatus.MPIE readable/writable, mie and mip readable, ecall from U gives mcause 8,
       M-mode interrupts always enabled while in U-mode. LINUX=0 is upstream behaviour. */
    parameter [0:0] LINUX = 0
  )
  (
   input wire 	    i_clk,
   input wire 	    i_rst,
   //State
   input wire 	    i_trig_irq,
   input wire 	    i_en,
   input wire 	    i_cnt0to3,
   input wire 	    i_cnt3,
   input wire 	    i_cnt7,
   input wire 	    i_cnt11,
   input wire 	    i_cnt12,
   input wire 	    i_cnt_done,
   input wire 	    i_mem_op,
   input wire 	    i_mtip,
   input wire 	    i_trap,
   output reg 	    o_new_irq,
   //Control
   input wire 	    i_e_op,
   input wire 	    i_ebreak,
   input wire 	    i_mem_cmd,
   input wire 	    i_mstatus_en,
   input wire 	    i_mie_en,
   input wire 	    i_mcause_en,
   input wire 	    i_mip_en,
   input wire [1:0] i_csr_source,
   input wire 	    i_mret,
   input wire 	    i_csr_d_sel,
   //Data
   input wire 	[B:0]    i_rf_csr_out,
   output wire 	[B:0]    o_csr_in,
   input wire 	[B:0]    i_csr_imm,
   input wire 	[B:0]    i_rs1,
   output wire 	[B:0]    o_q);

   localparam [1:0]
     CSR_SOURCE_CSR = 2'b00,
     CSR_SOURCE_EXT = 2'b01,
     CSR_SOURCE_SET = 2'b10,
     CSR_SOURCE_CLR = 2'b11;

   reg 		    mstatus_mie;
   reg 		    mstatus_mpie;
   reg 		    mie_mtie;
   //LINUX: current privilege (1 = M, 0 = U) and mstatus.MPP (only 00 and 11 are legal)
   reg 		    mode_m = 1'b1;
   reg 		    mstatus_mpp = 1'b1;

   reg 		mcause31;
   reg [3:0] 	mcause3_0;
   wire [B:0]	mcause;

   wire [B:0]	csr_in;
   wire [B:0]	csr_out;

   reg 		timer_irq_r;

   wire [B:0]	d = i_csr_d_sel ? i_csr_imm : i_rs1;

   assign csr_in = (i_csr_source == CSR_SOURCE_EXT) ? d :
		   (i_csr_source == CSR_SOURCE_SET) ? csr_out | d :
		   (i_csr_source == CSR_SOURCE_CLR) ? csr_out & ~d :
		   (i_csr_source == CSR_SOURCE_CSR) ? csr_out :
		   {W{1'bx}};

   wire [B:0]	mstatus;

   generate
      //MPP is bits 12:11 (reads 11 without LINUX), MPIE bit 7, MIE bit 3
      wire mpp  = !LINUX | mstatus_mpp;
      wire mpie = LINUX & mstatus_mpie;
      if (W==1) begin : gen_mstatus_w1
	 assign mstatus = ((mstatus_mie & i_cnt3) | (mpie & i_cnt7) | (mpp & (i_cnt11 | i_cnt12)));
      end else if (W==4) begin : gen_mstatus_w4
	 assign mstatus = {(mpp & i_cnt11) | (mstatus_mie & i_cnt3) | (mpie & i_cnt7), 2'b00, mpp & i_cnt12};
      end
   endgenerate

   //LINUX: mie.MTIE and mip.MTIP (bit 7) are readable, so csrs/csrc on mie keep MTIE
   wire 	mie_q = LINUX & i_mie_en & i_en & i_cnt7 & mie_mtie;
   wire 	mip_q = LINUX & i_mip_en & i_en & i_cnt7 & i_mtip;

   assign csr_out = ({W{i_mstatus_en & i_en}} & mstatus) |
		    i_rf_csr_out |
		    ({W{i_mcause_en & i_en}} & mcause) |
		    {mie_q | mip_q, {B{1'b0}}};

   assign o_q = csr_out;

   //M-mode interrupts are globally enabled whenever the hart runs in U-mode
   wire 	timer_irq = i_mtip & (mstatus_mie | (LINUX & !mode_m)) & mie_mtie;

   //Exactly one cycle per trap entry / mret (mret is a one-stage instruction)
   wire 	trap_done = i_trap & i_cnt_done;
   wire 	mret_done = LINUX & i_mret & !i_trap & i_cnt_done;

   /* Upstream restores MIE from MPIE on every cycle mret is decoded, which is idempotent
      only while MPIE never changes. With LINUX, mret also sets MPIE, so restore once. */
   wire 	mret_mie = LINUX ? mret_done : i_mret;

   /* An interrupt is taken by running the fetched instruction's cycle with i_trap set. If
      that instruction is a CSR access to mstatus or mie, upstream still performs its write:
      e.g. an interrupt on "csrr a5, mstatus" clears MIE at bit 3 before the trap saves MIE
      into MPIE, so mret later restores MIE=0 (interrupts silently left disabled). The
      instruction is re-executed after mret anyway, so with LINUX these writes are dropped
      during a trap. (mcause and the RF-stored CSRs already let the trap win.) */
   wire 	csr_wr_ok = !(LINUX & i_trap);

   //ecall raises cause 11 from M-mode and 8 from U-mode (bits 1:0 cleared); ebreak is 3
   wire 	e_op_lsbs = i_e_op & (!LINUX | i_ebreak | mode_m);

   assign mcause = i_cnt0to3 ? mcause3_0[B:0] : //[3:0]
		   i_cnt_done ? {mcause31,{B{1'b0}}} //[31]
		   : {W{1'b0}};

   assign o_csr_in = csr_in;

   always @(posedge i_clk) begin
      if (i_trig_irq) begin
	 timer_irq_r <= timer_irq;
	 o_new_irq   <= timer_irq & !timer_irq_r;
      end

      if (i_mie_en & i_cnt7 & csr_wr_ok)
	mie_mtie <= csr_in[B];

      /*
       The mie bit in mstatus gets updated under three conditions

       When a trap is taken, the bit is cleared
       During an mret instruction, the bit is restored from mpie
       During a mstatus CSR access instruction it's assigned when
        bit 3 gets updated

       These conditions are all mutually exclusive
       */
      if ((i_trap & i_cnt_done) | i_mstatus_en & i_cnt3 & i_en & csr_wr_ok | mret_mie)
	mstatus_mie <= !i_trap & (mret_mie ?  mstatus_mpie : csr_in[B]);

      /*
       Note: To save resources mstatus_mpie (mstatus bit 7) is not
       readable or writable from sw unless LINUX=1 (Linux writes MPIE
       before mret, e.g. to start a task with interrupts enabled)
       */
      if (trap_done)
	mstatus_mpie <= mstatus_mie;
      else if (mret_done)
	mstatus_mpie <= 1'b1;
      else if (LINUX & i_mstatus_en & i_cnt7 & i_en & csr_wr_ok)
	mstatus_mpie <= csr_in[B];

      if (LINUX) begin
	 if (trap_done) begin
	    mstatus_mpp <= mode_m;
	    mode_m      <= 1'b1;
	 end else if (mret_done) begin
	    mstatus_mpp <= 1'b0;
	    mode_m      <= mstatus_mpp;
	 end else if (i_mstatus_en & i_cnt11 & i_en & csr_wr_ok)
	   mstatus_mpp <= csr_in[B];
      end

      /*
       The four lowest bits in mcause hold the exception code

       These bits get updated under three conditions

       During an mcause CSR access function, they are assigned when
       bits 0 to 3 gets updated

       During an external interrupt the exception code is set to
       7, since SERV only support timer interrupts

       During an exception, the exception code is assigned to indicate
       if it was caused by an ebreak instruction (3),
       ecall instruction (11), misaligned load (4), misaligned store (6)
       or misaligned jump (0)

       The expressions below are derived from the following truth table
       irq  => 0111 (timer=7)
       e_op => x011 (ebreak=3, ecall=11)
       mem  => 01x0 (store=6, load=4)
       ctrl => 0000 (jump=0)
       */
      if (i_mcause_en & i_en & i_cnt0to3 | (i_trap & i_cnt_done)) begin
	 /* An interrupt replaces the fetched instruction; if that was an ecall, upstream still
	    sets bit 3 and reports cause 15. LINUX masks it (a tick on a syscall = cause 7). */
	 mcause3_0[3] <= (i_e_op & !i_ebreak & !(LINUX & o_new_irq)) | (!i_trap & csr_in[B]);
	 mcause3_0[2] <= o_new_irq | i_mem_op | (!i_trap & ((W == 1) ? mcause3_0[3] : csr_in[(W == 1) ? 0 : 2]));
	 mcause3_0[1] <= o_new_irq | e_op_lsbs | (i_mem_op & i_mem_cmd) | (!i_trap & ((W == 1) ? mcause3_0[2] : csr_in[(W == 1) ? 0 : 1]));
	 mcause3_0[0] <= o_new_irq | e_op_lsbs | (!i_trap & ((W == 1) ? mcause3_0[1] : csr_in[0]));
      end
      if (i_mcause_en & i_cnt_done | i_trap)
	mcause31 <= i_trap ? o_new_irq : csr_in[B];
      if (i_rst)
	if (RESET_STRATEGY != "NONE") begin
	   o_new_irq <= 1'b0;
	   mie_mtie <= 1'b0;
	   mode_m <= 1'b1;
	   mstatus_mpp <= 1'b1;
	end
   end

endmodule
