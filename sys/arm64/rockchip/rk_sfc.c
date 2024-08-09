/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <machine/bus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include "spibus_if.h"

#define SFC_CTRL			0x0
#define  SFC_CTRL_PHASE_SEL_NEGATIVE	 (1 << 1)
#define  SFC_CTRL_CMD_BITS_SHIFT	8
#define  SFC_CTRL_ADDR_BITS_SHIFT	10
#define  SFC_CTRL_DATA_BITS_SHIFT	12

/* Interrupt mask */
#define SFC_IMR				0x4
#define  SFC_IMR_RX_FULL		 (1 << 0)
#define  SFC_IMR_RX_UFLOW		 (1 << 1)
#define  SFC_IMR_TX_OFLOW		 (1 << 2)
#define  SFC_IMR_TX_EMPTY		 (1 << 3)
#define  SFC_IMR_TRAN_FINISH		 (1 << 4)
#define  SFC_IMR_BUS_ERR		 (1 << 5)
#define  SFC_IMR_NSPI_ERR		 (1 << 6)
#define  SFC_IMR_DMA			 (1 << 7)

/* Interrupt clear */
#define SFC_ICLR			0x8
#define  SFC_ICLR_RX_FULL		 (1 << 0)
#define  SFC_ICLR_RX_UFLOW		 (1 << 1)
#define  SFC_ICLR_TX_OFLOW		 (1 << 2)
#define  SFC_ICLR_TX_EMPTY		 (1 << 3)
#define  SFC_ICLR_TRAN_FINISH		 (1 << 4)
#define  SFC_ICLR_BUS_ERR		 (1 << 5)
#define  SFC_ICLR_NSPI_ERR		 (1 << 6)
#define  SFC_ICLR_DMA			 (1 << 7)

/* FIFO threshold level */
#define SFC_FTLR			0xc
#define  SFC_FTLR_TX_SHIFT		0
#define  SFC_FTLR_TX_MASK		0x1f
#define  SFC_FTLR_RX_SHIFT		8
#define  SFC_FTLR_RX_MASK		0x1f

/* Reset FSM and FIFO */
#define SFC_RCVR			0x10
#define  SFC_RCVR_RESET			 (1 << 0)

/* Enhanced mode */
#define SFC_AX				0x14

/* Address Bit number */
#define SFC_ABIT			0x18

/* Interrupt status */
#define SFC_ISR				0x1c
#define  SFC_ISR_RX_FULL_SHIFT		 (1 << 0)
#define  SFC_ISR_RX_UFLOW_SHIFT		 (1 << 1)
#define  SFC_ISR_TX_OFLOW_SHIFT		 (1 << 2)
#define  SFC_ISR_TX_EMPTY_SHIFT		 (1 << 3)
#define  SFC_ISR_TX_FINISH_SHIFT	 (1 << 4)
#define  SFC_ISR_BUS_ERR_SHIFT		 (1 << 5)
#define  SFC_ISR_NSPI_ERR_SHIFT		 (1 << 6)
#define  SFC_ISR_DMA_SHIFT		 (1 << 7)

/* FIFO status */
#define SFC_FSR				0x20
#define  SFC_FSR_TX_IS_FULL		 (1 << 0)
#define  SFC_FSR_TX_IS_EMPTY		 (1 << 1)
#define  SFC_FSR_RX_IS_EMPTY		 (1 << 2)
#define  SFC_FSR_RX_IS_FULL		 (1 << 3)
#define  SFC_FSR_TXLV_MASK		(0x1f << 8)
#define  SFC_FSR_TXLV_SHIFT		8
#define  SFC_FSR_RXLV_MASK		(0x1f << 16)
#define  SFC_FSR_RXLV_SHIFT		16

/* FSM status */
#define SFC_SR				0x24
#define  SFC_SR_IS_IDLE			0x0
#define  SFC_SR_IS_BUSY			0x1

/* Raw interrupt status */
#define SFC_RISR			0x28
#define  SFC_RISR_RX_FULL		 (1 << 0)
#define  SFC_RISR_RX_UNDERFLOW		 (1 << 1)
#define  SFC_RISR_TX_OVERFLOW		 (1 << 2)
#define  SFC_RISR_TX_EMPTY		 (1 << 3)
#define  SFC_RISR_TRAN_FINISH		 (1 << 4)
#define  SFC_RISR_BUS_ERR		 (1 << 5)
#define  SFC_RISR_NSPI_ERR		 (1 << 6)
#define  SFC_RISR_DMA			 (1 << 7)

/* Version */
#define SFC_VER				0x2C
#define  SFC_VER_3			0x3
#define  SFC_VER_4			0x4
#define  SFC_VER_5			0x5

/* Delay line controller resiter */
#define SFC_DLL_CTRL0			0x3C
#define SFC_DLL_CTRL0_SCLK_SMP_DLL	 (1 << 15)
#define SFC_DLL_CTRL0_DLL_MAX_VER4	0xFFU
#define SFC_DLL_CTRL0_DLL_MAX_VER5	0x1FFU

/* Master trigger */
#define SFC_DMA_TRIGGER			0x80
#define SFC_DMA_TRIGGER_START		1

/* Src or Dst addr for master */
#define SFC_DMA_ADDR			0x84

/* Length control register extension 32GB */
#define SFC_LEN_CTRL			0x88
#define SFC_LEN_CTRL_TRB_SEL		1
#define SFC_LEN_EXT			0x8C

/* Command */
#define SFC_CMD				0x100
#define  SFC_CMD_IDX_SHIFT		0
#define  SFC_CMD_DUMMY_SHIFT		8
#define  SFC_CMD_DIR_SHIFT		12
#define  SFC_CMD_DIR_RD			0
#define  SFC_CMD_DIR_WR			1
#define  SFC_CMD_ADDR_SHIFT		14
#define  SFC_CMD_ADDR_0BITS		0
#define  SFC_CMD_ADDR_24BITS		1
#define  SFC_CMD_ADDR_32BITS		2
#define  SFC_CMD_ADDR_XBITS		3
#define  SFC_CMD_TRAN_BYTES_SHIFT	16
#define  SFC_CMD_CS_SHIFT		30

/* Address */
#define SFC_ADDR			0x104

/* Data */
#define SFC_DATA			0x108

/* The controller and documentation reports that it supports up to 4 CS
 * devices (0-3), however I have only been able to test a single CS (CS 0)
 * due to the configuration of my device.
 */
#define SFC_MAX_CHIPSELECT_NUM		4

/* The SFC can transfer max 16KB - 1 at one time
 * we set it to 15.5KB here for alignment.
 */
#define SFC_MAX_IOSIZE_VER3		(512 * 31)

/* DMA is only enabled for large data transmission */
#define SFC_DMA_TRANS_THRETHOLD		(0x40)

/* Maximum clock values from datasheet suggest keeping clock value under
 * 150MHz. No minimum or average value is suggested.
 */
#define SFC_MAX_SPEED		(150 * 1000 * 1000)

#define RK_SFC_XFER_OUT 0
#define RK_SFC_XFER_IN  1

#define CMD_WRITE_ENABLE        0x06
#define CMD_WRITE_DISABLE       0x04
#define CMD_READ_IDENT          0x9F
#define CMD_READ_STATUS         0x05
#define CMD_WRITE_STATUS        0x01
#define CMD_READ                0x03
#define CMD_FAST_READ           0x0B
#define CMD_READ_DUAL_IO        0xBB
#define CMD_READ_QUAD_OUTPUT    0x6B
#define CMD_PAGE_PROGRAM        0x02
#define CMD_SECTOR_ERASE        0xD8
#define CMD_BULK_ERASE          0xC7
#define CMD_BLOCK_4K_ERASE      0x20
#define CMD_BLOCK_32K_ERASE     0x52
#define CMD_ENTER_4B_MODE       0xB7
#define CMD_EXIT_4B_MODE        0xE9

struct rk_sfc_conf {
 uint8_t cmd,  dir, addr_len, dummy_bytes;
 uint32_t xfer_len, addr;
 };

static struct ofw_compat_data compat_data[] = {
	{ "rockchip,sfc",		1 },
	{ NULL,					0 }
};

static struct resource_spec rk_sfc_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

struct rk_sfc_softc {
	device_t	dev;
	device_t	spibus;
	struct resource	*res[2];
	struct mtx	mtx;
	clk_t		clk_ahb;
	clk_t		clk_sfc;
	uint32_t	version;
	void *		intrhand;
	int		transfer;

	uint32_t	intreg;
	uint8_t		*rxbuf;
	uint32_t	rxidx;
	uint8_t		*txbuf;
	uint32_t	txidx;
	uint32_t	txlen;
	uint32_t	rxlen;
};

#define	RK_SFC_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	RK_SFC_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	RK_SFC_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	RK_SFC_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

#if 0
 #define dprintf(dev, format, arg...)     device_printf(dev, "%s: " format, __func__, arg)
#else
 #define dprintf(dev, format, arg...)
#endif

static int rk_sfc_probe(device_t dev);
static int rk_sfc_attach(device_t dev);
static int rk_sfc_detach(device_t dev);
static void rk_sfc_intr(void *arg);


static int
rk_sfc_reset(struct rk_sfc_softc *sc)
{
	int error, i;
	uint32_t status;

	RK_SFC_WRITE_4(sc, SFC_RCVR, SFC_RCVR_RESET);

	for(i = 0; i < 10000;i++) {
		status = RK_SFC_READ_4(sc, SFC_RCVR);
		if(!(status & SFC_RCVR_RESET)) break;
		DELAY(20);
	}

	error = status & SFC_RCVR_RESET;
	if (error)
		device_printf(sc->dev, "SFC reset did not finish\n");

	/* Still need to clear the masked interrupt from RISR */
	RK_SFC_WRITE_4(sc, SFC_ICLR, 0xFFFFFFFF);

	dprintf(sc->dev, "reset %s\n", "");

	return error;
}

static uint32_t
rk_sfc_get_version(struct rk_sfc_softc *sc)
{
	return  (RK_SFC_READ_4(sc, SFC_VER) & 0xffff);
}

static uint32_t
rk_sfc_get_max_iosize(struct rk_sfc_softc *sc)
{
	return SFC_MAX_IOSIZE_VER3;
}


static void
rk_sfc_irq_mask(struct rk_sfc_softc *sc, uint32_t mask)
{
	uint32_t reg;

	/* Disable transfer finish interrupt */
	reg = RK_SFC_READ_4(sc, SFC_IMR);
	reg |= mask;
	RK_SFC_WRITE_4(sc, SFC_IMR, reg);
}

static void
rk_sfc_irq_unmask(struct rk_sfc_softc *sc, uint32_t mask)
{
	uint32_t reg;

	/* Enable transfer complete interrupt */
	reg = RK_SFC_READ_4(sc, SFC_IMR);
	reg &= ~mask;
	RK_SFC_WRITE_4(sc, SFC_IMR, reg);
}
static int
rk_sfc_init(struct rk_sfc_softc *sc)
{
	RK_SFC_WRITE_4(sc, SFC_CTRL, 0);
	RK_SFC_WRITE_4(sc, SFC_ICLR, 0xFFFFFFFF);
	rk_sfc_irq_mask(sc, 0xFFFFFFFF);
	if (rk_sfc_get_version(sc) >= SFC_VER_4)
		RK_SFC_WRITE_4(sc, SFC_LEN_CTRL, SFC_LEN_CTRL_TRB_SEL);

	return 0;
}

static int
rk_sfc_xfer_done(struct rk_sfc_softc *sc, uint32_t timeout_us)
{
	int ret = 0, cycles =0;
	uint32_t status;
	sbintime_t sbt_end = sbinuptime() + 4294 * timeout_us;

	while(1) {
		cycles++;
		status = RK_SFC_READ_4(sc, SFC_SR);
		if(!(status & SFC_SR_IS_BUSY)) break;
		if(getsbinuptime() > sbt_end) {
			ret = 1;
			break;
		}
	}
	if (ret) {
		device_printf(sc->dev, "wait sfc idle timeout, cycles=%d\n", cycles);
		rk_sfc_reset(sc);

		ret = EIO;
	}

	return (ret);
}
static int
rk_sfc_wait_txfifo_ready(struct rk_sfc_softc *sc, uint32_t timeout_us, uint32_t *level)
{
	int ret = 0, cycles = 0;
	uint32_t status;
	sbintime_t sbt_end = sbinuptime() + 4294 * timeout_us;
	while(1) {
		cycles++;
		status = RK_SFC_READ_4(sc, SFC_FSR);
		if(status & SFC_FSR_TXLV_MASK) break;
		if(getsbinuptime() > sbt_end) {
			ret = 1;
			break;
		}
	}

	if (ret) {
		device_printf(sc->dev, "sfc wait tx fifo timeout, cycles=%d\n", cycles);

		return (ETIMEDOUT);
	}

	*level = (status & SFC_FSR_TXLV_MASK) >> SFC_FSR_TXLV_SHIFT;
	return (0);
}

static int
rk_sfc_wait_rxfifo_ready(struct rk_sfc_softc *sc, uint32_t timeout_us, uint32_t *level)
{
	int ret = 0, cycles = 0;
	uint32_t status;
        sbintime_t sbt_end = sbinuptime() + 4294 * timeout_us;

        while(1) {
		cycles++;
		status = RK_SFC_READ_4(sc, SFC_FSR);
		if(status & SFC_FSR_RXLV_MASK) break;
		if(getsbinuptime() > sbt_end) {
                        ret = 1;
                        break;
		}
        }
	if (ret) {
		device_printf(sc->dev, "sfc wait rx fifo timeout, cycles=%d\n", cycles);

		return (ETIMEDOUT);
	}

	*level = (status & SFC_FSR_RXLV_MASK) >> SFC_FSR_RXLV_SHIFT;
	return (0);
}

static int
rk_sfc_write_fifo(struct rk_sfc_softc *sc, const uint8_t *buf, int len)
{
	uint8_t bytes = len & 0x3;
	uint32_t dwords;
	int tx_level, error;
	uint32_t write_words;
	uint32_t tmp = 0;

	dwords = len >> 2;
	while (dwords) {
		error = rk_sfc_wait_txfifo_ready(sc, 1000, &tx_level);
		if (error)
			return (error);
		write_words =tx_level < dwords ? tx_level : dwords;
		while(write_words) {
			RK_SFC_WRITE_4(sc, SFC_DATA, *((const uint32_t *)buf));
			buf += 4;
			write_words--;
			dwords--;
			}
	}

	/* write the rest non word aligned bytes */
	if (bytes) {
		error = rk_sfc_wait_txfifo_ready(sc, 1000, &tx_level);
		if (error)
			return (error);
		memcpy(&tmp, buf, bytes);
		RK_SFC_WRITE_4(sc, SFC_DATA, tmp);
	}

	return (len);
}

static int
rk_sfc_read_fifo(struct rk_sfc_softc *sc, uint8_t *buf, int len)
{
	uint8_t bytes = len & 0x3;
	uint32_t dwords;
	uint8_t read_words;
	int rx_level;
	int tmp, error;

	/* word aligned access only */
	dwords = len >> 2;
	while (dwords) {
		error = rk_sfc_wait_rxfifo_ready(sc, 10000, &rx_level);
		if (error)
			return (error);
		read_words = rx_level < dwords ? rx_level : dwords;
		while(read_words) {
		 *((uint32_t *)buf) = RK_SFC_READ_4(sc, SFC_DATA);
		 buf += 4;
		 read_words--;
		 dwords--;
		 }
	}

	/* read the rest non word aligned bytes */
	if (bytes) {
		error = rk_sfc_wait_rxfifo_ready(sc, 10000, &rx_level);
		if (error)
			return (error);
		tmp = RK_SFC_READ_4(sc, SFC_DATA);
		memcpy(buf, &tmp, bytes);
	}

	return (len);
}


static int
rk_sfc_xfer_data_poll(struct rk_sfc_softc *sc, void *buffer, uint8_t dir, uint32_t len)
{

	dprintf(sc->dev, "sfc xfer_poll dir=%d len=%x\n", dir, len);

	if (dir == RK_SFC_XFER_OUT)
		return rk_sfc_write_fifo(sc, buffer, len);
	else
		return rk_sfc_read_fifo(sc, buffer,  len);
}
static int
rk_sfc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Rockchip Serial Flash Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
rk_sfc_attach(device_t dev)
{
	struct rk_sfc_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, rk_sfc_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}

	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, rk_sfc_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, rk_sfc_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	/* Activate the module clock. */
	error = clk_get_by_ofw_name(dev, 0, "hclk_sfc", &sc->clk_ahb);
	if (error != 0) {
		device_printf(dev, "cannot get sfc ahb clock\n");
		goto fail;
	}
	error = clk_get_by_ofw_name(dev, 0, "clk_sfc", &sc->clk_sfc);
	if (error != 0) {
		device_printf(dev, "cannot get clk_sfc clock\n");
		goto fail;
	}
	error = clk_enable(sc->clk_ahb);
	if (error != 0) {
		device_printf(dev, "cannot enable ahb clock\n");
		goto fail;
	}
	error = clk_enable(sc->clk_sfc);
	if (error != 0) {
		device_printf(dev, "cannot enable sfc clock\n");
		goto fail;
	}

	rk_sfc_init(sc);
	sc->version = rk_sfc_get_version(sc);
	device_printf(dev, "Rockchip SFC version %d\n", sc->version);
	sc->spibus = device_add_child(dev, "spibus", -1);

	return (bus_generic_attach(dev));

fail:
	rk_sfc_detach(dev);
	return (error);
}

static int
rk_sfc_detach(device_t dev)
{
	struct rk_sfc_softc *sc;

	sc = device_get_softc(dev);

	bus_generic_detach(sc->dev);
	if (sc->spibus != NULL)
		device_delete_child(dev, sc->spibus);

	if (sc->clk_sfc != NULL)
		clk_release(sc->clk_sfc);
	if (sc->clk_ahb)
		clk_release(sc->clk_ahb);

	if (sc->intrhand != NULL)
		bus_teardown_intr(sc->dev, sc->res[1], sc->intrhand);

	bus_release_resources(dev, rk_sfc_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static void
rk_sfc_intr(void *arg)
{
	struct rk_sfc_softc *sc;
	uint32_t reg;

	sc = arg;
	RK_SFC_LOCK(sc);
	reg = RK_SFC_READ_4(sc, SFC_RISR);
	RK_SFC_WRITE_4(sc, SFC_ICLR, reg);
	RK_SFC_UNLOCK(sc);
}

static phandle_t
rk_sfc_get_node(device_t bus, device_t dev)
{

	return ofw_bus_get_node(bus);
}


static int
rk_sfc_xfer_setup(struct rk_sfc_softc *sc, struct rk_sfc_conf *op, uint32_t cs)
{
	uint32_t ctrl = 0, cmd = 0;

	/* set CMD */
	cmd = op->cmd;

	/* set ADDR */
	if (op->addr_len) {
		if (op->addr_len == 4) {
			cmd |= SFC_CMD_ADDR_32BITS << SFC_CMD_ADDR_SHIFT;
		} else if (op->addr_len == 3) {
			cmd |= SFC_CMD_ADDR_24BITS << SFC_CMD_ADDR_SHIFT;
		} else {
			cmd |= SFC_CMD_ADDR_XBITS << SFC_CMD_ADDR_SHIFT;
			RK_SFC_WRITE_4(sc, SFC_ABIT, op->addr_len * 8 - 1);
		}

	}

	/* set DUMMY */
	if (op->dummy_bytes) {
		cmd |= op->dummy_bytes * 8 << SFC_CMD_DUMMY_SHIFT;
	}

	/* set DATA */
	if (sc->version >= SFC_VER_4) /* Clear it if no data to transfer */
		RK_SFC_WRITE_4(sc, SFC_LEN_EXT, op->xfer_len);
	else
		cmd |= op->xfer_len << SFC_CMD_TRAN_BYTES_SHIFT;

	if (op->xfer_len) {
		if (op->dir == RK_SFC_XFER_OUT)
			cmd |= SFC_CMD_DIR_WR << SFC_CMD_DIR_SHIFT;

	}
	if (!op->xfer_len && op->addr_len)
		cmd |= SFC_CMD_DIR_WR << SFC_CMD_DIR_SHIFT;

	/* set the Controller */
	ctrl |= SFC_CTRL_PHASE_SEL_NEGATIVE;
	cmd |= cs << SFC_CMD_CS_SHIFT;

	dprintf(sc->dev, "sfc addr.nbytes=%x(x%d) dummy.nbytes=%x(x%d)\n",
		op->addr_len, 1,
		op->dummy_bytes, 1);
	dprintf(sc->dev, "sfc ctrl=%x cmd=%x addr=%x len=%x\n",
		ctrl, cmd, op->addr, op->xfer_len);

	RK_SFC_WRITE_4(sc, SFC_CTRL, ctrl);
	RK_SFC_WRITE_4(sc, SFC_CMD, cmd);

	if (op->addr_len)
		RK_SFC_WRITE_4(sc, SFC_ADDR, op->addr);

	return (0);
}
static uint32_t
mk_addr(uint8_t *b, uint8_t len)
{
KASSERT((len == 3) || (len == 4), ("Invalid sfc address length"));
  if(len == 3)
   return (b[0] << 16) | (b[1] << 8) | b[2];
  else /* len == 4*/
   return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
}

static int
rk_sfc_transfer(device_t dev, device_t child, struct spi_command *cmd)
{
	struct rk_sfc_softc *sc;
	uint32_t cs, mode, clock, ret;
	void *tbuf;
	uint8_t *tx_cmd = (uint8_t *) cmd->tx_cmd;
	struct rk_sfc_conf op;

	memset(&op, 0, sizeof(op));

	op.cmd = tx_cmd[0];
	op.dir =  RK_SFC_XFER_OUT;

	int err = 0;

	sc = device_get_softc(dev);
	spibus_get_cs(child, &cs);
	spibus_get_clock(child, &clock);
	spibus_get_mode(child, &mode);

	op.xfer_len = cmd->tx_data_sz;
	tbuf = cmd->tx_data;
	RK_SFC_LOCK(sc);
	err = 0;
	dprintf(sc->dev, "CMDXX %d %d %d\n", tx_cmd[0], cmd->tx_cmd_sz, cmd->tx_data_sz);
	if(cmd->tx_cmd_sz != 1) {
	 switch (op.cmd) {

	  case CMD_READ_STATUS:
	   tbuf = (char *)cmd->rx_cmd + 1;
	   op.dir = RK_SFC_XFER_IN;
	   op.xfer_len = 1;
	   break;

	  case CMD_READ_IDENT:
	   op.xfer_len = 3;
	   op.dir = RK_SFC_XFER_IN;
	   tbuf = ((char *)cmd->rx_cmd) + 1;
	   break;

	  case CMD_BLOCK_4K_ERASE:
	  case CMD_BLOCK_32K_ERASE:
	  case CMD_SECTOR_ERASE:
	   op.xfer_len = 0;
	   op.addr_len = cmd->tx_cmd_sz - 1;
	   op.addr = mk_addr(tx_cmd + 1, op.addr_len);
	    dprintf(dev, "ADDR DUMP cmd=%d %d %d %d %d\n", tx_cmd[0], tx_cmd[1], tx_cmd[2], tx_cmd[3], tx_cmd[4]);
	    dprintf(dev, "ADDR FMT a=%d l=%d\n",op.addr, op.addr_len);
	   break;

	  case CMD_FAST_READ:
	    op.dummy_bytes = 1;
	    op.dir = RK_SFC_XFER_IN;
	    op.addr_len = cmd->tx_cmd_sz - 2;
	    op.addr = mk_addr(tx_cmd + 1, op.addr_len);
	    dprintf(dev, "ADDR DUMP cmd=%d %d %d %d %d\n", tx_cmd[0], tx_cmd[1], tx_cmd[2], tx_cmd[3], tx_cmd[4]);
	    dprintf(dev, "ADDR FMT a=%d l=%d\n",op.addr, op.addr_len);
	    break;
	  case CMD_PAGE_PROGRAM:
           op.addr_len = cmd->tx_cmd_sz - 1;
           op.addr = mk_addr(tx_cmd + 1, op.addr_len);
	    dprintf(dev, "ADDR DUMP cmd=%d %d %d %d %d\n", tx_cmd[0], tx_cmd[1], tx_cmd[2], tx_cmd[3], tx_cmd[4]);
	    dprintf(dev, "ADDR FMT a=%d l=%d\n",op.addr, op.addr_len);
           break;
	  default:
	   device_printf(dev, "unknown cmd=%d tx_cmd_sz=%d\n", op.cmd, cmd->tx_cmd_sz);
	   break;
	  }
	 }
	 rk_sfc_xfer_setup(sc, &op, cs);
	 if(op.xfer_len) {
	  ret = rk_sfc_xfer_data_poll(sc, tbuf, op.dir, op.xfer_len);
	  if(ret != op.xfer_len) {
	   device_printf(sc->dev,"Short transfer want=%u done=%u\n", op.xfer_len, ret);
	   err = EIO;
	   }
	  }
	RK_SFC_UNLOCK(sc);
	if(!err)
	 err = rk_sfc_xfer_done(sc, 100000);
	return (err);

}

static device_method_t rk_sfc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rk_sfc_probe),
	DEVMETHOD(device_attach,	rk_sfc_attach),
	DEVMETHOD(device_detach,	rk_sfc_detach),

        /* spibus_if  */
	DEVMETHOD(spibus_transfer,	rk_sfc_transfer),

        /* ofw_bus_if */
	DEVMETHOD(ofw_bus_get_node,	rk_sfc_get_node),

	DEVMETHOD_END
};

static driver_t rk_sfc_driver = {
	"spi",
	rk_sfc_methods,
	sizeof(struct rk_sfc_softc),
};

DRIVER_MODULE(rk_sfc, simplebus, rk_sfc_driver, 0, 0);
DRIVER_MODULE(ofw_spibus, rk_sfc, ofw_spibus_driver, 0, 0);
MODULE_DEPEND(rk_sfc, ofw_spibus, 1, 1, 1);
OFWBUS_PNP_INFO(compat_data);
