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
    parameter [0:0] LINUX = 0,
    /* SMODE=1 (needs LINUX=1): S-mode for an SV32 Linux, see gen_smode below. */
    parameter [0:0] SMODE = 0
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
   input wire 	    i_seip,         //SMODE: supervisor external interrupt (PLIC), level
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
   output wire 	[B:0]    o_q,
   //SMODE
   input wire [2:0]  i_cnt_hi,       //serv_state o_cnt[4:2]
   input wire [3:0]  i_cnt_lsb,      //serv_state cnt_r (one-hot for W=1, 1111 for W=4)
   input wire        i_sret,
   input wire [12:0] i_sel,          //serv_decode o_csr_sel (one-hot, see gen_smode)
   input wire [63:0] i_mtime,
   input wire        i_ibus_fault,   //instruction page fault, with the fetch ack
   input wire        i_dbus_ack,
   input wire        i_dbus_fault_we,//data page fault was a store/AMO, with the dbus ack
   input wire        i_dfault,       //serv_state: this trap is a data page fault
   output wire       o_trap_s,       //this trap goes to S-mode (stvec/sepc/stval)
   output wire       o_tval_pc,      //this trap is an instruction page fault: tval = pc
   output wire [1:0] o_priv,
   output wire       o_sum,
   output wire       o_mxr,
   output wire       o_satp_mode,
   output wire       o_xlate,        //SMODE: satp.MODE & priv < M, as a register
   output wire [19:0] o_satp_ppn);

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

   wire [B:0]	csr_out_classic = ({W{i_mstatus_en & i_en}} & mstatus) |
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

   generate
   if (!SMODE) begin : gen_classic
   assign csr_out = csr_out_classic;
   assign o_trap_s = 1'b0;
   assign o_tval_pc = 1'b0;
   assign o_priv = {2{mode_m}};
   assign o_sum = 1'b0;
   assign o_mxr = 1'b0;
   assign o_satp_mode = 1'b0;
   assign o_xlate = 1'b0;
   assign o_satp_ppn = 20'd0;

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

   end else begin : gen_smode
   /*
    SMODE: privilege U/S/M for an SV32 Linux (doc/stage2_plan.md in the QERV-Linux project).

    CSRs here (one-hot i_sel from serv_decode):
      0 mstatus 0x300: SIE 1, MIE 3, SPIE 5, MPIE 7, SPP 8, MPP 12:11, SUM 18, MXR 19
                       (MPP = 10 is stored as 11)
      1 sstatus 0x100: SIE, SPIE, SPP, SUM, MXR
      2 mie   0x304: STIE 5, MTIE 7, SEIE 9     3 sie 0x104: STIE, SEIE (if delegated)
      4 mip   0x344: STIP 5 (writable), MTIP 7 (CLINT), SEIP 9 (i_seip, read-only)
      5 sip 0x144: STIP, SEIP (read-only, if delegated)
      6 mcause 0x342, 7 scause 0x142: bit 31 and bits 3:0
      8 medeleg 0x302: bits 0,3,4,6,8,12,13,15     9 mideleg 0x303: bits 5 (STI), 9 (SEI)
      10 satp 0x180: MODE 31, PPN 19:0 (ASID reads 0: no ASIDs)
      11 time 0xC01, 12 timeh 0xC81: i_mtime, sampled at every fetch so that a read that
         takes several cycles is not torn by a carry
    stvec, sscratch, sepc and stval are in the RF (serv_rf_if), next to the M ones.

    Every CSR is a parallel 32-bit view; at[i] marks the cycle in which bit i is on the
    serial CSR bus (lane i%W). Synthesis keeps only the bits that are not constant.

    Traps: interrupts are level-sensitive (MTI, then SEI, then STI) and sampled at each fetch; an
    instruction page fault with the fetch is taken the same way (the fetched word is
    replaced by a trap, tval = pc). A data page fault comes from serv_state (i_dfault).
    The trap goes to S when the hart is below M and the cause is delegated.
    */
   wire [31:0] at;
   genvar      gi;
   for (gi = 0; gi < 32; gi = gi + 1) begin : gen_at
      assign at[gi] = i_en & (i_cnt_hi == (gi / 4)) & ((W == 4) | i_cnt_lsb[gi % 4]);
   end
   wire [31:0] wr_at = at & {32{csr_wr_ok}};

   reg [1:0]  priv;
   reg 	      m_ie, m_pie, mpp1, mpp0, s_ie, s_pie, s_pp, sum, mxr;
   reg 	      mtie, stie, stip, mideleg5, seie, mideleg9;
   reg [15:0] medeleg;
   reg 	      m_int, s_int;
   reg [3:0]  m_code, s_code;
   reg 	      satp_mode;
   reg [19:0] satp_ppn;
   reg [63:0] mtime_s;
   reg 	      new_int, int_m, int_e, new_ifault, dfault_we;

   wire       s_mstatus = i_sel[0],  s_sstatus = i_sel[1],  s_mie   = i_sel[2];
   wire       s_sie     = i_sel[3],  s_mip     = i_sel[4],  s_sip   = i_sel[5];
   wire       s_mcause  = i_sel[6],  s_scause  = i_sel[7],  s_medeleg = i_sel[8];
   wire       s_mideleg = i_sel[9],  s_satp    = i_sel[10], s_time  = i_sel[11];
   wire       s_timeh   = i_sel[12];

   wire [15:0] medeleg_m = medeleg & 16'hB159;

   wire [31:0] v_status  = {12'd0, mxr, sum, 5'd0, mpp1, mpp1 | mpp0, 2'd0, s_pp, m_pie, 1'b0,
			    s_pie, 1'b0, m_ie, 1'b0, s_ie, 1'b0};
   wire [31:0] v_mie     = {22'd0, seie, 1'b0, mtie, 1'b0, stie, 5'd0};
   wire [31:0] v_mip     = {22'd0, i_seip, 1'b0, i_mtip, 1'b0, stip, 5'd0};
   wire [31:0] v_sie     = {22'd0, seie & mideleg9, 3'd0, stie & mideleg5, 5'd0};
   wire [31:0] v_sip     = {22'd0, i_seip & mideleg9, 3'd0, stip & mideleg5, 5'd0};
   wire [31:0] v_mcause  = {m_int, 27'd0, m_code};
   wire [31:0] v_scause  = {s_int, 27'd0, s_code};
   wire [31:0] v_medeleg = {16'd0, medeleg_m};
   wire [31:0] v_mideleg = {22'd0, mideleg9, 3'd0, mideleg5, 5'd0};
   wire [31:0] v_satp    = {satp_mode, 11'd0, satp_ppn};

   wire [31:0] v = ({32{s_mstatus}} & v_status) |
		   ({32{s_sstatus}} & v_status & 32'h000C0122) |
		   ({32{s_mie}}     & v_mie)     | ({32{s_sie}}     & v_sie) |
		   ({32{s_mip}}     & v_mip)     | ({32{s_sip}}     & v_sip) |
		   ({32{s_mcause}}  & v_mcause)  | ({32{s_scause}}  & v_scause) |
		   ({32{s_medeleg}} & v_medeleg) | ({32{s_mideleg}} & v_mideleg) |
		   ({32{s_satp}}    & v_satp)    |
		   ({32{s_time}}    & mtime_s[31:0]) | ({32{s_timeh}} & mtime_s[63:32]);

   localparam [31:0] LANE0 = (W == 4) ? 32'h11111111 : 32'hFFFFFFFF;
   wire [B:0]  sv;
   genvar      gl;
   for (gl = 0; gl < W; gl = gl + 1) begin : gen_lane
      assign sv[gl] = |(v & at & (LANE0 << gl));
   end
   assign csr_out = sv | i_rf_csr_out;

   wire        m_mode = priv[1];
   wire        s_mode = !priv[1] & priv[0];
   wire        u_mode = !priv[1] & !priv[0];
   /* The privilege the trap was taken from. priv changes at trap_done, but the RF writes of
      the tval/epc CSRs trail the counter by a cycle and their address depends on trap_s
      (whose ecall cause and delegation depend on the privilege), and i_trap stays high
      until the next fetch. */
   reg [1:0]   tpriv;
   wire [1:0]  cur_priv = i_trap ? tpriv : priv;
   wire        mti = i_mtip & mtie & (!m_mode | m_ie);
   wire        sti = stip & stie & (mideleg5 ? (u_mode | (s_mode & s_ie)) : (!m_mode | m_ie));
   wire        sei = i_seip & seie & (mideleg9 ? (u_mode | (s_mode & s_ie)) : (!m_mode | m_ie));
   wire        irq = mti | sei | sti;

   wire [3:0]  code = new_int    ? (int_m ? 4'd7 : int_e ? 4'd9 : 4'd5) :
		      new_ifault ? 4'd12 :
		      i_e_op     ? (i_ebreak ? 4'd3 : {2'b10, cur_priv}) :
		      i_dfault   ? (dfault_we ? 4'd15 : 4'd13) :
		      i_mem_op   ? (i_mem_cmd ? 4'd6 : 4'd4) :
		      4'd0;
   wire        trap_s = !cur_priv[1] & (new_int ? (!int_m & (int_e ? mideleg9 : mideleg5)) :
					    medeleg_m[code]);

   wire        sret_done = i_sret & !i_trap & i_cnt_done;

   assign o_trap_s    = trap_s;
   assign o_tval_pc   = new_ifault;
   assign o_priv      = priv;
   assign o_sum       = sum;
   assign o_mxr       = mxr;
   assign o_satp_mode = satp_mode;
   /* Address translation on (satp.MODE and priv below M), kept as its own register,
      updated wherever priv or satp.MODE are: the MMU's address selects then start at a
      flop instead of priv/satp logic. */
   reg         xlate_r;
   assign o_xlate     = xlate_r;
   assign o_satp_ppn  = satp_ppn;

   integer     k;
   always @(posedge i_clk) begin
      if (i_trig_irq) begin
	 mtime_s    <= i_mtime;
	 new_int    <= irq;
	 int_m      <= mti;
	 int_e      <= !mti & sei;
	 new_ifault <= i_ibus_fault & !irq;
	 o_new_irq  <= irq | i_ibus_fault;
      end
      if (i_dbus_ack)
	dfault_we <= i_dbus_fault_we;
      if (!i_trap)
	tpriv <= priv;

      //CSR instructions (never in the same cycle as a trap entry or xret completion)
      if ((s_mstatus | s_sstatus) & wr_at[1])  s_ie  <= csr_in[1 % W];
      if (s_mstatus & wr_at[3])                m_ie  <= csr_in[3 % W];
      if ((s_mstatus | s_sstatus) & wr_at[5])  s_pie <= csr_in[5 % W];
      if (s_mstatus & wr_at[7])                m_pie <= csr_in[7 % W];
      if ((s_mstatus | s_sstatus) & wr_at[8])  s_pp  <= csr_in[8 % W];
      if (s_mstatus & wr_at[11])               mpp0  <= csr_in[11 % W];
      if (s_mstatus & wr_at[12])               mpp1  <= csr_in[12 % W];
      if ((s_mstatus | s_sstatus) & wr_at[18]) sum   <= csr_in[18 % W];
      if ((s_mstatus | s_sstatus) & wr_at[19]) mxr   <= csr_in[19 % W];
      if (s_mie & wr_at[7])                    mtie  <= csr_in[7 % W];
      if ((s_mie | (s_sie & mideleg5)) & wr_at[5]) stie <= csr_in[5 % W];
      if (s_mip & wr_at[5])                    stip  <= csr_in[5 % W];
      if (s_mideleg & wr_at[5])                mideleg5 <= csr_in[5 % W];
      if ((s_mie | (s_sie & mideleg9)) & wr_at[9]) seie <= csr_in[9 % W];
      if (s_mideleg & wr_at[9])                mideleg9 <= csr_in[9 % W];
      if (s_mcause & wr_at[31])                m_int <= csr_in[31 % W];
      if (s_scause & wr_at[31])                s_int <= csr_in[31 % W];
      if (s_satp & wr_at[31]) begin
	 satp_mode <= csr_in[31 % W];
	 xlate_r   <= csr_in[31 % W] & !priv[1];
      end
      for (k = 0; k < 4; k = k + 1) begin
	 if (s_mcause & wr_at[k]) m_code[k] <= csr_in[k % W];
	 if (s_scause & wr_at[k]) s_code[k] <= csr_in[k % W];
      end
      for (k = 0; k < 16; k = k + 1)
	if (s_medeleg & wr_at[k]) medeleg[k] <= csr_in[k % W];
      for (k = 0; k < 20; k = k + 1)
	if (s_satp & wr_at[k]) satp_ppn[k] <= csr_in[k % W];

      if (trap_done) begin
	 if (trap_s) begin
	    s_int <= new_int;
	    s_code <= code;
	    s_pie <= s_ie;
	    s_ie <= 1'b0;
	    s_pp <= priv[0];
	    priv <= 2'b01;
	    xlate_r <= satp_mode;
	 end else begin
	    m_int <= new_int;
	    m_code <= code;
	    m_pie <= m_ie;
	    m_ie <= 1'b0;
	    {mpp1, mpp0} <= priv;
	    priv <= 2'b11;
	    xlate_r <= 1'b0;
	 end
      end
      if (mret_done) begin
	 priv <= {mpp1, mpp1 | mpp0};
	 xlate_r <= satp_mode & !mpp1;
	 m_ie <= m_pie;
	 m_pie <= 1'b1;
	 {mpp1, mpp0} <= 2'b00;
      end
      if (sret_done) begin
	 priv <= {1'b0, s_pp};
	 xlate_r <= satp_mode;
	 s_ie <= s_pie;
	 s_pie <= 1'b1;
	 s_pp <= 1'b0;
      end

      if (i_rst) begin
	 priv <= 2'b11;
	 {m_ie, m_pie, mpp1, mpp0, s_ie, s_pie, s_pp, sum, mxr} <= 9'd0;
	 {mtie, stie, stip, mideleg5, seie, mideleg9} <= 6'd0;
	 medeleg <= 16'd0;
	 {m_int, s_int, m_code, s_code} <= 10'd0;
	 satp_mode <= 1'b0;
	 xlate_r <= 1'b0;
	 satp_ppn <= 20'd0;
	 {new_int, int_m, int_e, new_ifault, dfault_we} <= 5'd0;
	 o_new_irq <= 1'b0;
      end
   end

   end
   endgenerate

endmodule
