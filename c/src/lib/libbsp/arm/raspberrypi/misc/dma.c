/**
 * @file
 *
 * @ingroup rpi_dma
 *
 * @brief Direct memory access (DMA) support.
 */

#include <bsp.h>
#include <bsp/raspberrypi.h>
#include <bsp/dma.h>
#include <stdlib.h>
#include <errno.h>

/* private flags */
#define BCM_DMA_CH_USED 0x00000001
#define BCM_DMA_CH_FREE 0x40000000
#define BCM_DMA_CH_UNMAP 0x80000000

/* Register Map (4.2.1.2) */
#define BCM_DMA_CS( n ) ( 0x100 * ( n ) + 0x00 )
/* Different States */
#define         CS_ACTIVE ( 1 << 0 )
#define         CS_END ( 1 << 1 )
#define         CS_INT ( 1 << 2 )
#define         CS_DREQ ( 1 << 3 )
#define         CS_ISPAUSED ( 1 << 4 )
#define         CS_ISHELD ( 1 << 5 )
#define         CS_ISWAIT ( 1 << 6 )
#define         CS_ERR ( 1 << 8 )
#define         CS_WAITWRT ( 1 << 28 )
#define         CS_DISDBG ( 1 << 29 )
#define         CS_ABORT ( 1 << 30 )
#define         CS_RESET ( 1U << 31 )
#define BCM_DMA_CBADDR( n ) ( 0x100 * ( n ) + 0x04 )
#define BCM_DMA_INFO( n ) ( 0x100 * ( n ) + 0x08 )
#define         INFO_INT_EN ( 1 << 0 )
#define         INFO_TDMODE ( 1 << 1 )
#define         INFO_WAIT_RESP ( 1 << 3 )
#define         INFO_D_INC ( 1 << 4 )
#define         INFO_D_WIDTH ( 1 << 5 )
#define         INFO_D_DREQ ( 1 << 6 )
#define         INFO_S_INC ( 1 << 8 )
#define         INFO_S_WIDTH ( 1 << 9 )
#define         INFO_S_DREQ ( 1 << 10 )
#define         INFO_WAITS_SHIFT ( 21 )
#define         INFO_PERMAP_SHIFT ( 16 )
#define         INFO_PERMAP_MASK ( 0x1f << INFO_PERMAP_SHIFT )

#define BCM_DMA_SRC( n ) ( 0x100 * ( n ) + 0x0C )
#define BCM_DMA_DST( n ) ( 0x100 * ( n ) + 0x10 )
#define BCM_DMA_LEN( n ) ( 0x100 * ( n ) + 0x14 )
#define BCM_DMA_STRIDE( n ) ( 0x100 * ( n ) + 0x18 )
#define BCM_DMA_CBNEXT( n ) ( 0x100 * ( n ) + 0x1C )
#define BCM_DMA_DEBUG( n ) ( 0x100 * ( n ) + 0x20 )
#define         DEBUG_ERROR_MASK ( 7 )

#define BCM_DMA_INT_STATUS 0xfe0
#define BCM_DMA_ENABLE 0xff0

#define BCM_PAGE_SIZE 4096
#define BCM_PAGE_MASK ( BCM_PAGE_SIZE - 1 )
/**
 * @brief Table that indicates if a channel is currently occupied.
 */
static struct bcm_dma_ch bcm_dma_ch[ BCM_DMA_CH_MAX ];
static Atomic_Flag       dma_mutex;

static void rpi_dmamap_cb(
  void              *arg,
  bus_dma_segment_t *segs,
  int                nseg,
  int                err
)
{
  unsigned int *addr;

  if ( err )
    return;

  addr = (unsigned int *) arg;
  *addr = PHYS_TO_VCBUS( segs[ 0 ].ds_addr );
}

rtems_status_code rpi_dma_reset( int ch )
{
  struct bcm_dma_cb *cb;
  uint32_t           cs; /* Current State */
  int                count;

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  /* Current State of the channel */
  cs = BCM2835_REG( BCM_DMA_CS( ch ) );

  if ( cs & CS_ACTIVE ) {
    /* Pause current task */
    BCM2835_REG( BCM_DMA_CS( ch ) ) = 0;

    /* Wait */
    count = 1000;

    do {
      cs = BCM2835_REG( BCM_DMA_CS( ch ) );
    } while ( !( cs & CS_ISPAUSED ) && ( count-- > 0 ) );

    if ( !( cs & CS_ISPAUSED ) ) {
      return ( RTEMS_RESOURCE_IN_USE );
    }

    /* Clear the next control block address */
    BCM2835_REG( BCM_DMA_CBNEXT( ch ) ) = 0;

    /* Complete everything, clear interrupt */
    BCM2835_REG( BCM_DMA_CS( ch ) ) = CS_ABORT | CS_INT | CS_END | CS_ACTIVE;
  }

  /* Clear control blocks */
  BCM2835_REG( BCM_DMA_CBADDR( ch ) ) = 0;
  BCM2835_REG( BCM_DMA_CBNEXT( ch ) ) = 0;

  /* Reset control block */
  cb = bcm_dma_ch[ ch ].cb;
  bzero( cb, sizeof( *cb ) );
  cb->info - INFO_WAIT_RESP;

  return RTEMS_SUCCESSFUL;
}

/*
 * Allocate DMA channel for further use
 */
rtems_status_code rpi_dma_allocate( int req_ch )
{
  /* Check the channel number provided */
  if ( req_ch >= BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  if ( _Atomic_Flag_test_and_set( &dma_mutex,
         ATOMIC_ORDER_ACQUIRE ) != 0 ) {
    printk( "Could not lock the DMA mutex\n" );
    return RTEMS_UNSATISFIED;
  }

  /* Check whether a channel is in use or not and allocate accordingly */
  if ( bcm_dma_ch[ req_ch ].flags & BCM_DMA_CH_FREE ) {
    bcm_dma_ch[ req_ch ].flags &= ~BCM_DMA_CH_FREE;
    bcm_dma_ch[ req_ch ].flags |= BCM_DMA_CH_USED;
  } else {
    return ( RTEMS_RESOURCE_IN_USE );
  }

  _Atomic_Flag_clear( &dma_mutex, ATOMIC_ORDER_RELEASE );

  return ( RTEMS_SUCCESSFUL );
}

/*
 * Frees allocated channel. Returns 0 on success, -1 otherwise
 */
rtems_status_code rpi_dma_free( int ch )
{
  rtems_status_code status_code;

  if ( _Atomic_Flag_test_and_set( &dma_mutex,
         ATOMIC_ORDER_ACQUIRE ) != 0 ) {
    printk( "Could not lock the DMA mutex\n" );
    return RTEMS_UNSATISFIED;
  }

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  /* Check whether the channel is in use or not and free accordingly */
  if ( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) {
    bcm_dma_ch[ ch ].flags |= BCM_DMA_CH_FREE;
    bcm_dma_ch[ ch ].flags &= ~BCM_DMA_CH_USED;
    bcm_dma_ch[ ch ].intr_func = NULL;
    bcm_dma_ch[ ch ].intr_arg = NULL;

    /* Reset DMA */
    status_code = rpi_dma_reset( ch );
  }

  _Atomic_Flag_clear( &dma_mutex, ATOMIC_ORDER_RELEASE );

  return ( status_code );
}

/*
 * Assign handler function for channel interrupt
 */
rtems_status_code rpi_dma_setup_intr(
  int   ch,
  void  ( *func )(
    int,
    void *
  ),
  void *arg
)
{
  struct bcm_dma_cb *cb;

  /* Check the channel number provided */
  if ( ch < 0 || ch > -BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  /* Check whether the channel is in use */
  if ( !( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) )
    return ( RTEMS_RESOURCE_IN_USE );

  /* Update the software context */
  bcm_dma_ch[ ch ].intr_func = func;
  bcm_dma_ch[ ch ].intr_arg = arg;
  cb = bcm_dma_ch[ ch ].cb;
  cb->info |= INFO_INT_EN;

  return ( RTEMS_SUCCESSFUL );
}

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
 *
 */
rtems_status_code rpi_dma_setup_src(
  int ch,
  int dreq,
  int inc_addr,
  int width
)
{
  uint32_t info;

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  /* Check whether the channel is in use */
  if ( !( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) )
    return ( RTEMS_RESOURCE_IN_USE );

  /* Configure the info field */
  info = bcm_dma_ch[ ch ].cb->info;
  info &= ~INFO_PERMAP_MASK;
  info |= ( dreq << INFO_PERMAP_SHIFT ) & INFO_PERMAP_MASK;

  if ( dreq )
    info |= INFO_S_DREQ;
  else
    info &= ~INFO_S_DREQ;

  if ( width == BCM_DMA_128BIT )
    info |= INFO_S_WIDTH;
  else
    info &= ~INFO_S_WIDTH;

  if ( inc_addr == BCM_DMA_INC_ADDR )
    info |= INFO_S_INC;
  else
    info &= ~INFO_S_INC;

  /* Update the info field in the CB struture of the given channel */
  bcm_dma_ch[ ch ].cb->info = info;

  return ( RTEMS_SUCCESSFUL );
}

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
)
{
  uint32_t info;

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return ( RTEMS_INVALID_ID );

  /* Check whether the channel is in use */
  if ( !( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) )
    return ( RTEMS_RESOURCE_IN_USE );

  /* Configure the info field */
  info = bcm_dma_ch[ ch ].cb->info;
  info &= ~INFO_PERMAP_MASK;
  info |= ( dreq << INFO_PERMAP_SHIFT ) & INFO_PERMAP_MASK;

  if ( dreq )
    info |= INFO_D_DREQ;
  else
    info &= ~INFO_D_DREQ;

  if ( width == BCM_DMA_128BIT )
    info |= INFO_D_WIDTH;
  else
    info &= ~INFO_D_WIDTH;

  if ( inc_addr == BCM_DMA_INC_ADDR )
    info |= INFO_D_INC;
  else
    info &= ~INFO_D_INC;

  bcm_dma_ch[ ch ].cb->info = info;

  return RTEMS_SUCCESSFUL;
}

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
)
{
  struct bcm_dma_cb *cb;

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return RTEMS_INVALID_ID;

  /* Check whether the channel is in use */
  if ( !( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) )
    return RTEMS_RESOURCE_IN_USE;

  cb = bcm_dma_ch[ ch ].cb;

  /*
   *  Checking whether the pointer address belongs to the
   *  top IO window. If it does converting physical address
   *  in IO space to VC bus address. If it does not then
   *  convert physical address to VC bus address.
   */
  if ( BCM2835_ARM_IS_IO( src ) )
    cb->src = IO_TO_VCBUS( src );
  else
    /* While submitting address over mailbox interface */
    cb->src = PHYS_TO_VCBUS( src );

  if ( BCM2835_ARM_IS_IO( dst ) )
    cb->dst = IO_TO_VCBUS( src );
  else
    cb->dst = PHYS_TO_VCBUS( dst );

  cb->len = len;

  /* Cache coherency */
  rtems_cache_flush_multiple_data_lines(
    (void *) bcm_dma_ch[ ch ].dma_map->buffer_begin,
    bcm_dma_ch[ ch ].dma_map->buffer_size );

  /* Write the physical address of the control block into the register */
  BCM2835_REG( BCM_DMA_CBADDR( ch ) ) = bcm_dma_ch[ ch ].vc_cb;

  /* Change the state of the channel */
  BCM2835_REG( BCM_DMA_CS( ch ) ) = CS_ACTIVE;

  return RTEMS_SUCCESSFUL;
}

/*
 * Get length requested for DMA transaction
 *     ch - channel number
 *
 * Returns size of transaction, 0 if channel is invalid
 */
uint32_t rpi_dma_length( int ch )
{
  struct bcm_dma_cb *cb;

  /* Check the channel number provided */
  if ( ch < 0 || ch >= BCM_DMA_CH_MAX )
    return RTEMS_INVALID_ID;

  /* Check whether the channel is in use */
  if ( !( bcm_dma_ch[ ch ].flags & BCM_DMA_CH_USED ) )
    return RTEMS_RESOURCE_IN_USE;

  cb = bcm_dma_ch[ ch ].cb;

  return ( cb->len );
}

/* Utility function for rpi_dma_init */
int bus_dmamap_load_buffer(
  bus_dma_segment_t segs[],
  void             *buf,
  unsigned int      buflen,
  int               flags,
  vm_offset_t      *lastaddrp,
  int              *segp,
  int               first
)
{
  unsigned int sgsize;
  unsigned int curaddr, lastaddr, baddr, bmask;
  vm_offset_t  vaddr = (vm_offset_t) buf;
  int          seg;

  lastaddr = *lastaddrp;
  bmask = ~( -1 );

  for ( seg = *segp; buflen > 0; ) {
    /*
     * Get the physical address for this segment.
     */
    curaddr = vaddr;

    /*
     * Compute the segment size, and adjust counts.
     */
    sgsize = BCM_PAGE_SIZE - ( (u_long) curaddr & BCM_PAGE_MASK );

    if ( sgsize > sizeof( struct bcm_dma_cb ) )
      sgsize = sizeof( struct bcm_dma_cb );

    if ( buflen < sgsize )
      sgsize = buflen;

    /*
     * Insert chunk into a segment, coalescing with
     * the previous segment if possible.
     */
    if ( first ) {
      segs[ seg ].ds_addr = curaddr;
      segs[ seg ].ds_len = sgsize;
      first = 0;
    } else {
      if ( curaddr == lastaddr &&
           ( segs[ seg ].ds_len + sgsize ) <= sizeof( struct bcm_dma_cb ) &&
           ( segs[ seg ].ds_addr & bmask ) == ( curaddr & bmask ) )
        segs[ seg ].ds_len += sgsize;
      else {
        if ( ++seg >= 1 )
          break;

        segs[ seg ].ds_addr = curaddr;
        segs[ seg ].ds_len = sgsize;
      }
    }

    lastaddr = curaddr + sgsize;
    vaddr += sgsize;
    buflen -= sgsize;
  }

  *segp = seg;
  *lastaddrp = lastaddr;

  return ( buflen != 0 ? 27 : 0 );
}

static int rpi_dma_init()
{
  int                i;
  struct bcm_dma_ch *ch;
  void              *cb_virt;
  vm_paddr_t         cb_phys;
  int                error, nsegs;
  vm_offset_t        lastaddr;

  /* Setup Initial Setting */
  for ( i = 0; i < BCM_DMA_CH_MAX; i++ ) {
    bus_dma_segment_t dm_segments[ 1 ];

    ch = &bcm_dma_ch[ i ];

    bzero( ch, sizeof( struct bcm_dma_ch ) );
    ch->ch = i;
    ch->flags = BCM_DMA_CH_UNMAP;

    ch->dma_map = malloc( sizeof( struct bus_dmamap ) );

    if ( ch->dma_map == NULL ) {
      return ENOMEM;
    }

    /* Alignment = 1 , Boundary = 0 */
    cb_virt = rtems_cache_coherent_allocate(
      sizeof( struct bcm_dma_cb ), 1, 0 );

    if ( cb_virt == NULL ) {
      free( ch->dma_map );

      return ENOMEM;
    }

    ch->dma_map->buffer_begin = cb_virt;
    ch->dma_map->buffer_size = sizeof( struct bcm_dma_cb );

    memset( cb_virt, 0, sizeof( struct bcm_dma_cb ) );

    /*
     * Least alignment for busdma-allocated stuff is cache
     * line size, so just make sure nothing stupid happened
     * and we got properly aligned address
     */
    if ( (unsigned long int) cb_virt & 0x1f )
      break;

//FIXME : Verify mapping buffer into bus space using the dmamap
    lastaddr = (vm_offset_t) 0;
    nsegs = 0;

    error = bus_dmamap_load_buffer( dm_segments, cb_virt,
      sizeof( struct bcm_dma_cb ), 0x00, &lastaddr, &nsegs, 1 );

    if ( error == 0 )
      rpi_dmamap_cb( &cb_phys, dm_segments, nsegs + 1, 0 );
    else
      rpi_dmamap_cb( &cb_phys, NULL, 0, error );

    ch->cb = cb_virt;
    ch->vc_cb = cb_phys;
    ch->flags = BCM_DMA_CH_FREE;
    ch->cb->info = INFO_WAIT_RESP;

    /* reset DMA */
    BCM2835_REG( BCM_DMA_CS( i ) ) = CS_RESET;
  }

  return 0;
}

static void rpi_dma_intr( void *arg )
{
  struct bcm_dma_ch *ch = (struct bcm_dma_ch *) arg;
  uint32_t           cs, debug;

  cs = BCM2835_REG( BCM_DMA_CS( ch->ch ) );

  if ( !( cs & ( CS_INT | CS_ERR ) ) ) {
    return;
  }

  /* Check whether the channel is in use */
  if ( !( ch->flags & BCM_DMA_CH_USED ) ) {
    return;
  }

  if ( cs & CS_ERR ) {
    debug = BCM2835_REG( BCM_DMA_DEBUG( ch->ch ) );
    BCM2835_REG( BCM_DMA_DEBUG( ch->ch ) ) = debug & DEBUG_ERROR_MASK;
    rpi_dma_reset( ch->ch );
  }

  if ( cs & CS_INT ) {
    /* acknowledge interrupt */
    BCM2835_REG( BCM_DMA_CS( ch->ch ) ) = CS_INT | CS_END;

    /* Note : BUS DMA sync : POST write so cache coherency not required. */

    /* save callback function and argument */
    if ( ch->intr_func )
      ch->intr_func( ch->ch, ch->intr_arg );
  }
}

