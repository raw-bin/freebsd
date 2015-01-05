/*-
 * Copyright (c) 2014 The FreeBSD Foundation
 * All rights reserved.
 *
 * This software was developed by Semihalf under
 * the sponsorship of the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/cpuset.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#include "gic_v3_reg.h"
#include "gic_v3_var.h"

/*
 * FDT glue.
 */
struct gic_v3_ofw_devinfo {
	struct ofw_bus_devinfo	di_dinfo;
	struct resource_list	di_rl;
};

static int arm_gic_v3_fdt_probe(device_t);
static int arm_gic_v3_fdt_attach(device_t);
static int arm_gic_v3_fdt_detach(device_t);

static struct resource * arm_gic_v3_bus_alloc_res(device_t,
    device_t, int, int *, u_long, u_long, u_long, u_int);

static const struct ofw_bus_devinfo *
arm_gic_v3_ofw_get_devinfo(device_t __unused, device_t);

static device_method_t arm_gic_v3_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		arm_gic_v3_fdt_probe),
	DEVMETHOD(device_attach,	arm_gic_v3_fdt_attach),
	DEVMETHOD(device_detach,	arm_gic_v3_fdt_detach),

	/* PIC interface */
	DEVMETHOD(pic_dispatch,		arm_gic_v3_dispatch),
	DEVMETHOD(pic_eoi,		arm_gic_v3_eoi),
	DEVMETHOD(pic_mask,		arm_gic_v3_mask_irq),
	DEVMETHOD(pic_unmask,		arm_gic_v3_unmask_irq),

	/* Bus interface */
	DEVMETHOD(bus_alloc_resource,		arm_gic_v3_bus_alloc_res),
	DEVMETHOD(bus_activate_resource,	bus_generic_activate_resource),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_devinfo,	arm_gic_v3_ofw_get_devinfo),
	DEVMETHOD(ofw_bus_get_compat,	ofw_bus_gen_get_compat),
	DEVMETHOD(ofw_bus_get_model,	ofw_bus_gen_get_model),
	DEVMETHOD(ofw_bus_get_name,	ofw_bus_gen_get_name),
	DEVMETHOD(ofw_bus_get_node,	ofw_bus_gen_get_node),
	DEVMETHOD(ofw_bus_get_type,	ofw_bus_gen_get_type),

	/* End */
	DEVMETHOD_END
};

static driver_t arm_gic_v3_driver = {
	"gic",
	arm_gic_v3_methods,
	sizeof(struct gic_v3_softc),
};

EARLY_DRIVER_MODULE(gic_v3, simplebus, arm_gic_v3_driver, arm_gic_v3_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
EARLY_DRIVER_MODULE(gic_v3, ofwbus, arm_gic_v3_driver, arm_gic_v3_devclass, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

/*
 * Helper functions declarations.
 */
static int gic_v3_fdt_bus_attach(device_t);

/*
 * Device interface.
 */
static int
arm_gic_v3_fdt_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "arm,gic-v3"))
		return (ENXIO);

	device_set_desc(dev, GIC_V3_DEVSTR);
	return (BUS_PROBE_DEFAULT);
}

static int
arm_gic_v3_fdt_attach(device_t dev)
{
	struct gic_v3_softc *sc;
	pcell_t redist_regions;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;

	/*
	 * Recover number of the Re-Distributor regions.
	 */
	if (OF_getprop(ofw_bus_get_node(dev), "#redistributor-regions",
	    &redist_regions, sizeof(redist_regions)) <= 0)
		sc->gic_r_nregions = 1;
	else
		sc->gic_r_nregions = fdt32_to_cpu(redist_regions);

	err = arm_gic_v3_attach(dev);
	if (err)
		goto error;
	/*
	 * Try to register ITS to this GIC.
	 * GIC will act as a bus in that case.
	 * Failure here will not affect main GIC functionality.
	 */
	if (gic_v3_fdt_bus_attach(dev)) {
		if (bootverbose) {
			device_printf(dev,
			    "Failed to attach ITS to this GIC\n");
		}
	}

	return (err);

error:
	if (bootverbose) {
		device_printf(dev,
		    "Failed to attach. Error %d\n", err);
	}
	/* Failure so free resources */
	arm_gic_v3_fdt_detach(dev);

	return (err);
}

static int
arm_gic_v3_fdt_detach(device_t dev)
{

	return (arm_gic_v3_detach(dev));
}

/* ofw_bus interface */
static const struct ofw_bus_devinfo *
arm_gic_v3_ofw_get_devinfo(device_t bus __unused, device_t child)
{
	struct gic_v3_ofw_devinfo *di;

	di = device_get_ivars(child);
	return (&di->di_dinfo);
}

/* Bus interface */
static struct resource *
arm_gic_v3_bus_alloc_res(device_t bus, device_t child, int type, int *rid,
    u_long start, u_long end, u_long count, u_int flags)
{
	struct gic_v3_ofw_devinfo *di;
	struct resource_list_entry *rle;
	int ranges_len;

	if ((start == 0UL) && (end == ~0UL)) {
		if ((di = device_get_ivars(child)) == NULL)
			return (NULL);
		if (type != SYS_RES_MEMORY)
			return (NULL);

		/* Find defaults for this rid */
		rle = resource_list_find(&di->di_rl, type, *rid);
		if (rle == NULL)
			return (NULL);

		start = rle->start;
		end = rle->end;
		count = rle->count;
	}
	/*
	 * XXX: No ranges remap!
	 *	Absolute address is expected.
	 */
	if (ofw_bus_has_prop(bus, "ranges")) {
		ranges_len = OF_getproplen(ofw_bus_get_node(bus), "ranges");
		if (ranges_len != 0) {
			if (bootverbose) {
				device_printf(child,
				    "Ranges remap not supported\n");
			}
			return (NULL);
		}
	}
	return (bus_generic_alloc_resource(bus, child, type, rid, start, end,
	    count, flags));
}

/* Helper functions */

/*
 * Bus capability support for GICv3.
 * Collects and configures device informations and finally
 * adds ITS device as a child of GICv3 in Newbus hierarchy.
 */
static int
gic_v3_fdt_bus_attach(device_t dev)
{
	struct gic_v3_ofw_devinfo *di;
	device_t child;
	phandle_t parent, node;
	pcell_t addr_cells, size_cells;
	uint64_t phys, size;
	uint32_t *reg;
	ssize_t nreg;
	size_t i, j, k;

	parent = ofw_bus_get_node(dev);
	if ((parent > 0) &&
	    (fdt_addrsize_cells(parent, &addr_cells, &size_cells) == 0)) {
		/* Iterate through all GIC subordinates */
		for (node = OF_child(parent); node > 0; node = OF_peer(node)) {
			/*
			 * Look out! This is a special case where we
			 * want to accept only ITS drivers and no other.
			 */
			if (!fdt_is_compatible(node, GIC_V3_ITS_COMPSTR))
				continue;

			/* Allocate and populate devinfo. */
			di = malloc(sizeof(*di), M_GIC_V3, M_WAITOK | M_ZERO);
			if (ofw_bus_gen_setup_devinfo(&di->di_dinfo, node)) {
				if (bootverbose) {
					device_printf(dev,
					    "Could not set up devinfo for ITS\n");
				}
				free(di, M_GIC_V3);
				continue;
			}

			/* Initialize and populate resource list. */
			resource_list_init(&di->di_rl);
			/*
			 * Get reg property.
			 * This assumes that the address is an absolute value.
			 */
			nreg = OF_getencprop_alloc(node,
			    "reg", sizeof(*reg), (void **)&reg);
			if ((nreg <= 0) ||
			    (nreg % (addr_cells + size_cells) != 0)) {
				if (bootverbose) {
					device_printf(dev,
					    "Could not process reg property for ITS\n");
				}
				ofw_bus_gen_destroy_devinfo(&di->di_dinfo);
				free(di, M_GIC_V3);
				free(reg, M_OFWPROP);
				continue;
			}
			for (i = 0, k = 0; i < nreg;
			    i += addr_cells + size_cells, k++) {
				phys = size = 0;
				for (j = 0; j < addr_cells; j++) {
					phys <<= 32;
					phys |= reg[i + j];
				}
				for (j = 0; j < size_cells; j++) {
					size <<= 32;
					size |= reg[i + addr_cells + j];
				}
				/* Ready to add resource to the list. */
				resource_list_add(&di->di_rl,
				    SYS_RES_MEMORY, k, phys, (phys + size - 1),
				    size);
			}
			free(reg, M_OFWPROP);

			/* Should not have any interrupts, so don't add any */

			/* Add newbus device for this FDT node */
			child = device_add_child(dev, NULL, -1);
			if (!child) {
				if (bootverbose) {
					device_printf(dev,
					    "Could not add child: %s\n",
					    di->di_dinfo.obd_name);
				}
				resource_list_free(&di->di_rl);
				ofw_bus_gen_destroy_devinfo(&di->di_dinfo);
				free(di, M_GIC_V3);
				continue;
			}

			device_set_ivars(child, di);
		}
	}

	return (bus_generic_attach(dev));
}
