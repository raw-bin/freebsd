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
#include <ddb/ddb.h>
#include <ddb/db_variables.h>

struct db_variable db_regs[] = {};
struct db_variable *db_eregs = NULL;

void
db_show_mdpcpu(struct pcpu *pc)
{
}

static int
db_validate_address(vm_offset_t addr)
{
#if 0
	struct proc *p = curproc;
	struct pmap *pmap;

	if (!p || !p->p_vmspace || !p->p_vmspace->vm_map.pmap ||
	    addr >= VM_MAXUSER_ADDRESS)
		pmap = pmap_kernel();
	else
		pmap = p->p_vmspace->vm_map.pmap;

	return (pmap_extract(pmap, addr) == FALSE);
#endif
	return 0;
}

/*
 * Read bytes from kernel address space for debugger.
 */
int
db_read_bytes(vm_offset_t addr, size_t size, char *data)
{
	char	*src = (char *)addr;

	while (size-- > 0) {
		if (db_validate_address((u_int)src)) {
			db_printf("address %p is invalid\n", src);
			return (-1);
		}
		*data++ = *src++;
	}
	return (0);
}

/*
 * Write bytes to kernel address space for debugger.
 */
int
db_write_bytes(vm_offset_t addr, size_t size, char *data)
{
	char *dst;

	dst = (char *)addr;
	while (size-- > 0) {
		if (db_validate_address((u_int)dst)) {
			db_printf("address %p is invalid\n", dst);
			return (-1);
		}
		*dst++ = *data++;
	}

#if 0
	/* make sure the caches and memory are in sync */
	cpu_icache_sync_range(addr, size);

	/* In case the current page tables have been modified ... */
	cpu_tlb_flushID();
#endif
	return (0);
}

u_int
branch_taken (u_int insn, u_int pc)
{

	return (0);
}
