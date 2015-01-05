/*-
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Brini.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BRINI ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL BRINI OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/proc.h>
#include <sys/kdb.h>
#include <machine/pcb.h>
#include <ddb/ddb.h>
#include <ddb/db_sym.h>

struct unwind_state {
	uint64_t fp;
	uint64_t sp;
	uint64_t pc;
};

void
db_md_list_watchpoints()
{
}

int
db_md_clr_watchpoint(db_expr_t addr, db_expr_t size)
{
	return (0);
}

int
db_md_set_watchpoint(db_expr_t addr, db_expr_t size)
{
	return (0);
}

static int
db_unwind_frame(struct unwind_state *frame)
{
	uint64_t fp = frame->fp;

	if (fp == 0)
		return -1;

	frame->sp = fp + 0x10;
	/* FP to previous frame (X29) */
	frame->fp = *(uint64_t *)(fp);
	/* LR (X30) */
	frame->pc = *(uint64_t *)(fp + 8) - 4;
	return (0);
}

static void
db_stack_trace_cmd(struct unwind_state *frame)
{
	c_db_sym_t sym;
	const char *name;
	db_expr_t value;
	db_expr_t offset;

	while (1) {
		uint64_t pc = frame->pc;
		int ret;

		ret = db_unwind_frame(frame);
		if (ret < 0)
			break;

		sym = db_search_symbol(pc, DB_STGY_ANY, &offset);
		if (sym == C_DB_SYM_NULL) {
			value = 0;
			name = "(null)";
		} else
			db_symbol_values(sym, &name, &value);

		db_printf("%s() at ", name);
		db_printsym(frame->pc, DB_STGY_PROC);
		db_printf("\n");

		db_printf("\t pc = 0x%08llx  lr = 0x%08llx\n", pc,
		    frame->pc);
		db_printf("\t sp = 0x%08llx  fp = 0x%08llx\n", frame->sp,
		    frame->fp);
		/* TODO: Show some more registers */
		db_printf("\n");
	}
}

int
db_trace_thread(struct thread *thr, int count)
{
	struct unwind_state frame;
	struct pcb *ctx;

	if (thr != curthread) {
		ctx = kdb_thr_ctx(thr);

		frame.sp = (uint64_t)ctx->pcb_sp;
		frame.fp = (uint64_t)ctx->pcb_x[29];
		frame.pc = (uint64_t)ctx->pcb_x[30];
		db_stack_trace_cmd(&frame);
	} else
		db_trace_self();
	return (0);
}

void
db_trace_self(void)
{
	struct unwind_state frame;
	uint64_t sp;

	__asm __volatile("mov %0, sp" : "=&r" (sp));

	frame.sp = sp;
	frame.fp = (uint64_t)__builtin_frame_address(0);
	frame.pc = (uint64_t)db_trace_self;
	db_stack_trace_cmd(&frame);
}
