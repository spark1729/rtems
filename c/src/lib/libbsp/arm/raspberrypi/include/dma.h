/**
 * @file
 *
 * @ingroup raspberrypi_dma
 *
 * @brief Direct memory access (DMA) support.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_DMA_H
#define LIBBSP_ARM_RASPBERRYPI_DMA_H

#define BCM_DMA_BLOCK_SIZE 512

/* DMA0-DMA15 but DMA15 is special */
#define BCM_DMA_CH_MAX 12

/* request CH for any nubmer */
#define BCM_DMA_CH_INVALID ( -1 )
#define BCM_DMA_CH_ANY ( -1 )

/* Peripheral DREQ Signals (4.2.1.3) */
#define BCM_DMA_DREQ_NONE 0
#define BCM_DMA_DREQ_EMMC 11
#define BCM_DMA_DREQ_SDHOST 13

#define BCM_DMA_SAME_ADDR 0
#define BCM_DMA_INC_ADDR 1

#define BCM_DMA_32BIT 0
#define BCM_DMA_128BIT 1

/*
 * Defines for converting physical address to VideoCore bus address and back
 */
#define BCM2835_VCBUS_SDRAM_CACHED 0x40000000
#define BCM2835_VCBUS_IO_BASE 0x7E000000
#define BCM2835_VCBUS_SDRAM_UNCACHED 0xC0000000

#ifdef SOC_BCM2836
#define BCM2835_VCBUS_SDRAM_BASE BCM2835_VCBUS_SDRAM_UNCACHED
#else
#define BCM2835_VCBUS_SDRAM_BASE BCM2835_VCBUS_SDRAM_CACHED
#endif
#define BCM2835_ARM_IO_SIZE 0x01000000

/*
 * Convert physical address to VC bus address. Should be used
 * when submitting address over mailbox interface
 */
#define PHYS_TO_VCBUS( pa ) ( ( pa ) + BCM2835_VCBUS_SDRAM_BASE )

/* Check whether pa bellong top IO window */
#define BCM2835_ARM_IS_IO( pa ) ( ( ( pa ) >= RPI_PERIPHERAL_BASE ) && \
                                  ( ( pa ) < RPI_PERIPHERAL_BASE + \
                                    BCM2835_ARM_IO_SIZE ) )

/*
 * Convert physical address in IO space to VC bus address.
 */
#define IO_TO_VCBUS( pa ) ( ( ( pa ) - RPI_PERIPHERAL_BASE ) + \
                            BCM2835_VCBUS_IO_BASE )

/*
 * Convert address from VC bus space to physical. Should be used
 * when address is returned by VC over mailbox interface. e.g.
 * framebuffer base
 */
#define VCBUS_TO_PHYS( vca ) ( ( vca ) & ~( BCM2835_VCBUS_SDRAM_BASE ) )

struct bus_dmamap {
  void *buffer_begin;
  uint32_t buffer_size;
};

typedef struct bus_dmamap *bus_dmamap_t;

/*
 *	bus_dma_segment_t
 *
 *	Describes a single contiguous DMA transaction.  Values
 *	are suitable for programming into DMA registers.
 */
typedef struct bus_dma_segment {
  unsigned int ds_addr;         /* DMA address */
  unsigned int ds_len;          /* length of transfer */
} bus_dma_segment_t;

/* DMA Control Block - 256bit aligned (p.40) */
struct bcm_dma_cb {
  uint32_t info;                /* Transfer Information */
  uint32_t src;                 /* Source Address */
  uint32_t dst;                 /* Destination Address */
  uint32_t len;                 /* Transfer Length */
  uint32_t stride;              /* 2D Mode Stride */
  uint32_t next;                /* Next Control Block Address */
  uint32_t rsvd1;               /* Reserved */
  uint32_t rsvd2;               /* Reserved */
};

/* DMA Channel Structure */
struct bcm_dma_ch {
  int ch;
  uint32_t flags;
  struct bcm_dma_cb *cb;
  uint32_t vc_cb;
  bus_dmamap_t dma_map;
  void ( *intr_func )(
    int,
    void *
  );
  void *intr_arg;
  Atomic_Flag dma_lock;
};

/* API */

/*
 * Allocate DMA channel for further use.
 */
rtems_status_code rpi_dma_allocate( int req_ch );

/*
 * Frees allocated channel.
 */
rtems_status_code rpi_dma_free( int ch );

/*
 * Assign handler function for channel interrupt
 */
rtems_status_code rpi_dma_setup_intr( int ch, void ( *func )(
    int,
    void *
  ), void *arg );

/*
 * Setup DMA source parameters
 *     ch - channel number
 *     dreq - hardware DREQ # or BCM_DMA_DREQ_NONE if
 *         source is physical memory
 *     inc_addr - BCM_DMA_INC_ADDR if source address
 *         should be increased after each access or
 *         BCM_DMA_SAME_ADDR if address should remain
 *         the same
 *     width - size of read operation, BCM_DMA_32BIT
 *         for 32bit bursts, BCM_DMA_128BIT for 128 bits
 */
rtems_status_code rpi_dma_setup_src(
  int ch,
  int dreq,
  int inc_addr,
  int width
);

/*
 * Setup DMA destination parameters
 *     ch - channel number
 *     dreq - hardware DREQ # or BCM_DMA_DREQ_NONE if
 *         destination is physical memory
 *     inc_addr - BCM_DMA_INC_ADDR if source address
 *         should be increased after each access or
 *         BCM_DMA_SAME_ADDR if address should remain
 *         the same
 *     width - size of write operation, BCM_DMA_32BIT
 *         for 32bit bursts, BCM_DMA_128BIT for 128 bits
 */
rtems_status_code rpi_dma_setup_dst(
  int ch,
  int dreq,
  int inc_addr,
  int width
);

/*
 * Start DMA transaction
 *     ch - channel number
 *     src, dst - source and destination address in
 *         ARM physical memory address space.
 *     len - amount of bytes to be transferred
 */
rtems_status_code rpi_dma_start(
  int        ch,
  vm_paddr_t src,
  vm_paddr_t dst,
  int        len
);

/*
 * Initializes the DMA
 */
int rpi_dma_init( int ch );

/*
 * Get length requested for DMA transaction
 *     ch - channel number
 */
uint32_t rpi_dma_length( int ch );

static void rpi_dma_intr( void *arg );

#endif /* LIBBSP_ARM_RASPBERRYPI_DMA_H */
