#include <sys/bus.h>

INTERFACE mtk_gdma;

HEADER {
#include <machine/bus.h>

	struct gdma_config {
		bool		dst_noincr;	/* Fix/Incremental mode */
		bool		src_noincr;	/* Fix/Incremental mode */
		unsigned int	burst_len;
		bool		soft_mode;	/* Software/Hardware mode */
		unsigned int	dst_dma_req;
		unsigned int	src_dma_req;
		bool		coherent;
	};

	typedef void (*gdma_callback)(void *);
}

#
# Allocate DMA channel
#
METHOD void * alloc {
	device_t dev;
	bool dedicated;
	gdma_callback callback;
	void *callback_arg;
};

#
# Free DMA channel
#
METHOD void free {
	device_t dev;
	void *dmachan;
};

#
# Set DMA channel configuration
#
METHOD int set_config {
	device_t dev;
	void *dmachan;
	const struct gdma_config *cfg;
};

#
# Start DMA channel transfer
#
METHOD int transfer {
	device_t dev;
	void *dmachan;
	bus_addr_t src;
	bus_addr_t dst;
	size_t nbytes;
};

#
# Halt DMA channel transfer
#
METHOD void halt {
	device_t dev;
	void *dmachan;
};
