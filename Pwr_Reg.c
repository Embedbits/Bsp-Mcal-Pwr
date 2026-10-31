/**
 * \author Mr.Nobody
 * \file Pwr_Reg.c
 * \ingroup Pwr
 * \brief Pwr module Reg component functionality.
 *
 * Register access of the module: generic accessors (bounds checked register ID) and the register
 * protocols of the power controller. The registers are written only here, the masks come from the
 * CMSIS device header.
 *
 * Properties of the registers, which the protocols respect (RM0481, RM0492, RM0517 - PWR chapter):
 * - PWR_VOSCR.VOS is changed one scale at a time, the new scale is reached when PWR_VOSSR.ACTVOS
 *   shows it and ACTVOSRDY is set (PWR_VOSSR.VOSRDY is used only when the performance is raised).
 * - PWR_SCCR.BYPASS is written once after the power-on reset, a later write is ignored. SMPSEN and
 *   LDOEN are read-only and tell the regulator selected by the package.
 * - PWR_PMCR.CSSF is write-only (reads as 0), PWR_PMCR.LPMS selects Stop (0) or Standby (1) for the
 *   deep sleep of the core, the SRAM shut-off bits (xxSO) keep the content in Stop mode when 0.
 * - PWR_WUSR.WUFx is cleared only through PWR_WUSCR.CWUFx (write-1-to-clear), PWR_PMSR.SBF and
 *   PWR_PMSR.STOPF only through PWR_PMCR.CSSF.
 * - PWR_VMCR.PVDE / PLS can be locked by the SBS module, a write that does not take effect is
 *   reported by the read-back of every protocol below.
 * - PWR_BDCR.BREN / MONEN are write protected until PWR_DBPCR.DBP is set, PWR_BDSR.BRRDY tells that
 *   the backup regulator is ready.
 * The non-secure access to a register that is secured by PWR_SECCFGR is ignored by the hardware
 * (the writes are dropped, the reads return 0), the read-back reports it as an error.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Pwr_Reg.h"                        /* Self include                   */
#include "Pwr_Types.h"                      /* Module types definitions       */
#include "Stm32.h"                          /* PWR registers definition       */
/* ======================= DEVICE LINE CONSISTENCY ========================== */

/* The feature macros of Pwr_Types.h are derived from the device line, check them against CMSIS */
#if defined(PWR_SCCR_SMPSEN) != defined(PWR_LINE_HAS_SMPS)
#error "Pwr_Reg: PWR_LINE_HAS_SMPS does not match PWR_SCCR_SMPSEN."
#endif

#if defined(PWR_USBSCR_USB33SV) != defined(PWR_LINE_HAS_USB_SUPPLY)
#error "Pwr_Reg: PWR_LINE_HAS_USB_SUPPLY does not match PWR_USBSCR_USB33SV."
#endif

#if defined(PWR_WUCR_WUPEN6) != defined(PWR_LINE_HAS_WKUP_6_8)
#error "Pwr_Reg: PWR_LINE_HAS_WKUP_6_8 does not match PWR_WUCR_WUPEN6."
#endif

#if defined(PWR_PMCR_SRAM2SO) != defined(PWR_LINE_SRAM2_SINGLE)
#error "Pwr_Reg: PWR_LINE_SRAM2_SINGLE does not match PWR_PMCR_SRAM2SO."
#endif

#if defined(PWR_PMCR_SRAM2_48SO) != defined(PWR_LINE_SRAM2_48)
#error "Pwr_Reg: PWR_LINE_SRAM2_48 does not match PWR_PMCR_SRAM2_48SO."
#endif

#if defined(PWR_PMCR_SRAM2_48LSO) != defined(PWR_LINE_SRAM2_48_SPLIT)
#error "Pwr_Reg: PWR_LINE_SRAM2_48_SPLIT does not match PWR_PMCR_SRAM2_48LSO."
#endif

#if defined(PWR_PMCR_SRAM2_16SO) != defined(PWR_LINE_SRAM2_16)
#error "Pwr_Reg: PWR_LINE_SRAM2_16 does not match PWR_PMCR_SRAM2_16SO."
#endif

#if defined(PWR_PMCR_SRAM2_16LSO) != defined(PWR_LINE_SRAM2_16_SPLIT)
#error "Pwr_Reg: PWR_LINE_SRAM2_16_SPLIT does not match PWR_PMCR_SRAM2_16LSO."
#endif

#if defined(PWR_PMCR_SRAM3SO) != defined(PWR_LINE_HAS_SRAM3)
#error "Pwr_Reg: PWR_LINE_HAS_SRAM3 does not match PWR_PMCR_SRAM3SO."
#endif

#if defined(PWR_PMCR_SRAM4SO) != defined(PWR_LINE_HAS_SRAM4_5)
#error "Pwr_Reg: PWR_LINE_HAS_SRAM4_5 does not match PWR_PMCR_SRAM4SO."
#endif

#if defined(PWR_PMCR_ETHERNETSO) != defined(PWR_LINE_HAS_ETH_RAM)
#error "Pwr_Reg: PWR_LINE_HAS_ETH_RAM does not match PWR_PMCR_ETHERNETSO."
#endif

#if defined(PWR_PMCR_LTDCSO) != defined(PWR_LINE_HAS_LTDC_RAM)
#error "Pwr_Reg: PWR_LINE_HAS_LTDC_RAM does not match PWR_PMCR_LTDCSO."
#endif
/* ============================== TYPEDEFS ================================== */

/** \brief Structure type used to store physical address of PWR registers.
 * This array is used to reduce size of configuration array. With this style
 * the registers are referenced through enumeration.
 *
 */
typedef struct
{
    pwr_RegId_t         RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   pwr_RegList_t;


/** \brief One row of the SRAM shut-off table: the retention block and its shut-off bit */
typedef struct
{
    pwr_SramId_t SramId;      /**< The retention block this row describes                */
    uint32_t     ShutOffMask; /**< Bit of PWR_PMCR that shuts the block off in Stop mode */
}   pwr_SramInfo_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static pwr_RequestState_t   Pwr_Reg_Wait_RegVal         ( pwr_RegId_t regId, uint32_t regMask, uint32_t regValue );
static pwr_FunctionState_t  Pwr_Reg_Get_BitState        ( pwr_RegId_t regId, uint32_t bitMask );
static pwr_RequestState_t   Pwr_Reg_Set_BitVerified     ( pwr_RegId_t regId,
                                                          uint32_t bitMask,
                                                          pwr_FunctionState_t bitState );
static pwr_RequestState_t   Pwr_Reg_Set_VosStep         ( pwr_VoltageScale_t voltageScale, pwr_FunctionState_t raise );
static pwr_RequestState_t   Pwr_Reg_Get_SramMask        ( pwr_SramId_t sramId, uint32_t * const shutOffMask );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Count of read-back iterations of a written register value */
#define PWR_REG_READBACK_TIMEOUT_RAW    ( 0x1000u )

/** \brief Count of iterations of the wait for the voltage scale and for the regulators */
#define PWR_REG_READY_TIMEOUT_RAW       ( 0x10000u )

/** \brief Register value without any bit set (masked register value of an inactive bit field) */
#define PWR_REG_BITS_CLEAR              ( 0u )

/** \brief Width of the pull configuration field of a wake-up pin in PWR_WUCR [bit] */
#define PWR_REG_WUPPUPD_WIDTH           ( 2u )

/** \brief Values of the VOS field (unshifted), the scale 3 is the lowest performance */
#define PWR_REG_VOS_SCALE_3             ( 0x0u )
#define PWR_REG_VOS_SCALE_2             ( 0x1u )
#define PWR_REG_VOS_SCALE_1             ( 0x2u )
#define PWR_REG_VOS_SCALE_0             ( 0x3u )

/** \brief Values of the SVOS field (unshifted), the SVOS3 is the highest Stop voltage */
#define PWR_REG_SVOS_3                  ( 0x3u )
#define PWR_REG_SVOS_4                  ( 0x2u )
#define PWR_REG_SVOS_5                  ( 0x1u )

/** \brief Values of the WUPPUPDx fields (unshifted) */
#define PWR_REG_WUPPUPD_NONE            ( 0x0u )
#define PWR_REG_WUPPUPD_UP              ( 0x1u )
#define PWR_REG_WUPPUPD_DOWN            ( 0x2u )

/** \brief Index of the wake-up inputs in the bit fields of PWR_WUCR (WKUP1 is the first field) */
#define PWR_REG_WKUP_1                  ( 0u )
#define PWR_REG_WKUP_2                  ( 1u )
#define PWR_REG_WKUP_3                  ( 2u )
#define PWR_REG_WKUP_4                  ( 3u )
#define PWR_REG_WKUP_5                  ( 4u )
#define PWR_REG_WKUP_6                  ( 5u )
#define PWR_REG_WKUP_7                  ( 6u )
#define PWR_REG_WKUP_8                  ( 7u )

/** \brief Wake-up flags (WUFx) of the device line that have a pin */
#if defined(PWR_LINE_HAS_WKUP_6_8)
#define PWR_REG_WUF_HIGH_MASK           ( PWR_WUSR_WUF6 | PWR_WUSR_WUF7 | PWR_WUSR_WUF8 )
#else
#define PWR_REG_WUF_HIGH_MASK           ( 0u )
#endif

#if defined(PWR_LINE_HAS_WKUP_3)
#define PWR_REG_WUF_3_MASK              ( PWR_WUSR_WUF3 )
#else
#define PWR_REG_WUF_3_MASK              ( 0u )
#endif

#define PWR_REG_WUF_MASK                ( PWR_WUSR_WUF1 | PWR_WUSR_WUF2 | PWR_REG_WUF_3_MASK | \
                                          PWR_WUSR_WUF4 | PWR_WUSR_WUF5 | PWR_REG_WUF_HIGH_MASK )

/** \brief The address of the PWR registers, the non-secure or the secure alias of the build */
#define PWR_REG_PERIPH                  ( PWR )

/** \brief Fields of the supervisor registers that are checked as a group */
#define PWR_REG_VBAT_FLAG_MASK          ( PWR_BDSR_VBATL | PWR_BDSR_VBATH )
#define PWR_REG_TEMP_FLAG_MASK          ( PWR_BDSR_TEMPL | PWR_BDSR_TEMPH )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief PWR registers configuration structure
 *
 * This structure is used to store addresses of PWR registers.
 * It is used to reduce code size while referring to PWR registers.
 */
static const pwr_RegList_t              pwr_RegList[] =
{
    { .RegId = PWR_REG_PMCR  , .RegAddr = &( PWR_REG_PERIPH->PMCR   ) }, /**< Power mode control register     */
    { .RegId = PWR_REG_PMSR  , .RegAddr = &( PWR_REG_PERIPH->PMSR   ) }, /**< Power mode status register      */
    { .RegId = PWR_REG_VOSCR , .RegAddr = &( PWR_REG_PERIPH->VOSCR  ) }, /**< Voltage scaling control         */
    { .RegId = PWR_REG_VOSSR , .RegAddr = &( PWR_REG_PERIPH->VOSSR  ) }, /**< Voltage scaling status          */
    { .RegId = PWR_REG_BDCR  , .RegAddr = &( PWR_REG_PERIPH->BDCR   ) }, /**< Backup domain control           */
    { .RegId = PWR_REG_DBPCR , .RegAddr = &( PWR_REG_PERIPH->DBPCR  ) }, /**< Backup protection control       */
    { .RegId = PWR_REG_BDSR  , .RegAddr = &( PWR_REG_PERIPH->BDSR   ) }, /**< Backup domain status            */
    { .RegId = PWR_REG_SCCR  , .RegAddr = &( PWR_REG_PERIPH->SCCR   ) }, /**< Supply configuration control   */
    { .RegId = PWR_REG_VMCR  , .RegAddr = &( PWR_REG_PERIPH->VMCR   ) }, /**< Voltage monitor control         */
#if defined(PWR_LINE_HAS_USB_SUPPLY)
    { .RegId = PWR_REG_USBSCR, .RegAddr = &( PWR_REG_PERIPH->USBSCR ) }, /**< USB supply control              */
#endif
    { .RegId = PWR_REG_VMSR  , .RegAddr = &( PWR_REG_PERIPH->VMSR   ) }, /**< Voltage monitor status          */
    { .RegId = PWR_REG_WUSCR , .RegAddr = &( PWR_REG_PERIPH->WUSCR  ) }, /**< Wake-up status clear            */
    { .RegId = PWR_REG_WUSR  , .RegAddr = &( PWR_REG_PERIPH->WUSR   ) }, /**< Wake-up status                  */
    { .RegId = PWR_REG_WUCR  , .RegAddr = &( PWR_REG_PERIPH->WUCR   ) }, /**< Wake-up configuration           */
    { .RegId = PWR_REG_IORETR, .RegAddr = &( PWR_REG_PERIPH->IORETR ) }, /**< I/O retention                   */
};

_Static_assert( (sizeof(pwr_RegList) / sizeof(pwr_RegList_t)) == PWR_REG_CNT,
                "Pwr_Reg: pwr_RegList has incorrect size." );

/** \brief VOS field value of every voltage scale, indexed by \ref pwr_VoltageScale_t */
static const uint32_t pwr_VosFieldValue[ PWR_VOLTAGE_SCALE_CNT ] =
{
    PWR_REG_VOS_SCALE_0, /**< PWR_VOLTAGE_SCALE_0 */
    PWR_REG_VOS_SCALE_1, /**< PWR_VOLTAGE_SCALE_1 */
    PWR_REG_VOS_SCALE_2, /**< PWR_VOLTAGE_SCALE_2 */
    PWR_REG_VOS_SCALE_3, /**< PWR_VOLTAGE_SCALE_3 */
};

/** \brief SVOS field value of every Stop sub-mode, indexed by \ref pwr_StopMode_t */
static const uint32_t pwr_SvosFieldValue[ PWR_STOP_CNT ] =
{
    PWR_REG_SVOS_3, /**< PWR_STOP_0 */
    PWR_REG_SVOS_4, /**< PWR_STOP_1 */
    PWR_REG_SVOS_5, /**< PWR_STOP_2 */
};

/** \brief The wake-up inputs of the device line and their fixed pins */
static const pwr_WakeUpPinInfo_t pwr_WakeUpPinList[] =
{
    { .PinItem = PWR_WAKEUP_PIN_1, .WkupIndex = PWR_REG_WKUP_1, .GpioPort = GPIO_PORT_A, .GpioPin = GPIO_PIN_ID_0  },
    { .PinItem = PWR_WAKEUP_PIN_2, .WkupIndex = PWR_REG_WKUP_2, .GpioPort = GPIO_PORT_A, .GpioPin = GPIO_PIN_ID_2  },
#if defined(STM32H503xx)
    { .PinItem = PWR_WAKEUP_PIN_3, .WkupIndex = PWR_REG_WKUP_3, .GpioPort = GPIO_PORT_C, .GpioPin = GPIO_PIN_ID_1  },
#elif defined(PWR_LINE_HAS_WKUP_3)
    { .PinItem = PWR_WAKEUP_PIN_3, .WkupIndex = PWR_REG_WKUP_3, .GpioPort = GPIO_PORT_I, .GpioPin = GPIO_PIN_ID_8  },
#endif
    { .PinItem = PWR_WAKEUP_PIN_4, .WkupIndex = PWR_REG_WKUP_4, .GpioPort = GPIO_PORT_C, .GpioPin = GPIO_PIN_ID_13 },
    { .PinItem = PWR_WAKEUP_PIN_5, .WkupIndex = PWR_REG_WKUP_5, .GpioPort = GPIO_PORT_B, .GpioPin = GPIO_PIN_ID_7  },
#if defined(PWR_LINE_HAS_WKUP_6_8)
    { .PinItem = PWR_WAKEUP_PIN_6, .WkupIndex = PWR_REG_WKUP_6, .GpioPort = GPIO_PORT_C, .GpioPin = GPIO_PIN_ID_1  },
    { .PinItem = PWR_WAKEUP_PIN_7, .WkupIndex = PWR_REG_WKUP_7, .GpioPort = GPIO_PORT_D, .GpioPin = GPIO_PIN_ID_2  },
    { .PinItem = PWR_WAKEUP_PIN_8, .WkupIndex = PWR_REG_WKUP_8, .GpioPort = GPIO_PORT_D, .GpioPin = GPIO_PIN_ID_3  },
#endif
};

_Static_assert( (sizeof(pwr_WakeUpPinList) / sizeof(pwr_WakeUpPinInfo_t)) == PWR_WAKEUP_PIN_UNUSED,
                "Pwr_Reg: pwr_WakeUpPinList has incorrect size." );

/** \brief The shut-off bits of the retention blocks of the device line (the backup SRAM has none) */
static const pwr_SramInfo_t pwr_SramList[] =
{
    { .SramId = PWR_SRAM_1       , .ShutOffMask = PWR_PMCR_SRAM1SO      },
#if defined(PWR_LINE_SRAM2_SINGLE)
    { .SramId = PWR_SRAM_2       , .ShutOffMask = PWR_PMCR_SRAM2SO      },
#endif
#if defined(PWR_LINE_SRAM2_48)
    { .SramId = PWR_SRAM_2_48    , .ShutOffMask = PWR_PMCR_SRAM2_48SO   },
#endif
#if defined(PWR_LINE_SRAM2_48_SPLIT)
    { .SramId = PWR_SRAM_2_48_LOW, .ShutOffMask = PWR_PMCR_SRAM2_48LSO  },
    { .SramId = PWR_SRAM_2_48_HIGH, .ShutOffMask = PWR_PMCR_SRAM2_48HSO },
#endif
#if defined(PWR_LINE_SRAM2_16)
    { .SramId = PWR_SRAM_2_16    , .ShutOffMask = PWR_PMCR_SRAM2_16SO   },
#endif
#if defined(PWR_LINE_SRAM2_16_SPLIT)
    { .SramId = PWR_SRAM_2_16_LOW, .ShutOffMask = PWR_PMCR_SRAM2_16LSO  },
    { .SramId = PWR_SRAM_2_16_HIGH, .ShutOffMask = PWR_PMCR_SRAM2_16HSO },
#endif
#if defined(PWR_LINE_HAS_SRAM3)
    { .SramId = PWR_SRAM_3       , .ShutOffMask = PWR_PMCR_SRAM3SO      },
#endif
#if defined(PWR_LINE_HAS_SRAM4_5)
    { .SramId = PWR_SRAM_4       , .ShutOffMask = PWR_PMCR_SRAM4SO      },
    { .SramId = PWR_SRAM_5       , .ShutOffMask = PWR_PMCR_SRAM5SO      },
#endif
#if defined(PWR_LINE_HAS_ETH_RAM)
    { .SramId = PWR_SRAM_ETHERNET, .ShutOffMask = PWR_PMCR_ETHERNETSO   },
#endif
#if defined(PWR_LINE_HAS_LTDC_RAM)
    { .SramId = PWR_SRAM_LTDC    , .ShutOffMask = PWR_PMCR_LTDCSO       },
#endif
};

_Static_assert( (sizeof(pwr_SramList) / sizeof(pwr_SramInfo_t)) == PWR_SRAM_BACKUP,
                "Pwr_Reg: pwr_SramList has incorrect size." );

/* The level items are the values of the PLS / ALS fields */
_Static_assert( PWR_PVD_LEVEL_PIN == 7, "Pwr_Reg: PWR_PVD_LEVEL_PIN is not the PLS value of the PVD_IN pin." );
_Static_assert( PWR_AVD_LEVEL_3 == 3, "Pwr_Reg: PWR_AVD_LEVEL_3 is not the highest ALS value." );

/* The bits of the module wake-up source mask are the WUFx bits of PWR_WUSR */
_Static_assert( PWR_WAKEUP_SOURCE_PIN_1 == PWR_WUSR_WUF1, "Pwr_Reg: wake-up source bit 1 is not WUF1." );
_Static_assert( PWR_WAKEUP_SOURCE_PIN_5 == PWR_WUSR_WUF5, "Pwr_Reg: wake-up source bit 5 is not WUF5." );

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Sets the register bit with given mask.
 *
 * \param regId   [in]: PWR register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Pwr_Set_RegBit( pwr_RegId_t regId, uint32_t bitMask )
{
    if( PWR_REG_CNT > regId )
    {
        *pwr_RegList[regId].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit with given mask.
 *
 * \param regId   [in]: PWR register ID
 * \param bitMask [in]: Mask to apply to the register
 */
void Pwr_Reset_RegBit( pwr_RegId_t regId, uint32_t bitMask )
{
    if( PWR_REG_CNT > regId )
    {
        *pwr_RegList[regId].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit value with given mask.
 *
 * \param regId   [in]: PWR register ID
 * \param bitMask [in]: Mask to apply to the register
 * \return Register bit value with applied mask, no bit set for invalid register ID
 */
uint32_t Pwr_Get_RegBit( pwr_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = PWR_REG_BITS_CLEAR;

    if( PWR_REG_CNT > regId )
    {
        retValue = ( *pwr_RegList[regId].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = PWR_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with given mask and value.
 *
 * \param regId    [in]: PWR register ID
 * \param regMask  [in]: Mask to apply to the register
 * \param regValue [in]: Value to set in the register
 */
void Pwr_Set_RegVal( pwr_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( PWR_REG_CNT > regId )
    {
        /* Set the register value with mask */
        *pwr_RegList[regId].RegAddr = ( ( *pwr_RegList[regId].RegAddr & ~regMask ) |
                                        ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with given mask.
 *
 * \param regId   [in]: PWR register ID.
 * \param regMask [in]: Mask to apply to the register
 * \return Register value with applied mask, no bit set for invalid register ID
 */
uint32_t Pwr_Get_RegVal( pwr_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = PWR_REG_BITS_CLEAR;

    if( PWR_REG_CNT > regId )
    {
        /* Get the register value with mask */
        retValue = ( *pwr_RegList[regId].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = PWR_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Reads the voltage scale that is applied to VCORE (PWR_VOSSR.ACTVOS).
 *
 * \param voltageScale [out]: Voltage scale applied to VCORE
 * \return \ref PWR_REQUEST_OK if the field holds a known scale, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Get_VoltageScale( pwr_VoltageScale_t * const voltageScale )
{
    pwr_RequestState_t retState   = PWR_REQUEST_ERROR;
    const uint32_t     fieldValue = ( Pwr_Get_RegVal( PWR_REG_VOSSR, PWR_VOSSR_ACTVOS_Msk ) >> PWR_VOSSR_ACTVOS_Pos );

    if( PWR_NULL_PTR != voltageScale )
    {
        for( uint32_t scaleIdx = 0u; (uint32_t)PWR_VOLTAGE_SCALE_CNT > scaleIdx; scaleIdx++ )
        {
            if( pwr_VosFieldValue[scaleIdx] == fieldValue )
            {
                *voltageScale = (pwr_VoltageScale_t)scaleIdx;
                retState      = PWR_REQUEST_OK;
            }
            else
            {
                /* The field value belongs to another scale */
            }
        }
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Changes the voltage scale one scale at a time up to the required one.
 *
 * Every step writes PWR_VOSCR.VOS and waits (bounded) for ACTVOS and ACTVOSRDY, when the
 * performance is raised also for VOSRDY. The sequence stops at the first step that fails, the
 * scale reached so far stays applied.
 *
 * \param voltageScale [in]: Required voltage scale
 * \return \ref PWR_REQUEST_OK if the required scale is applied, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_VoltageScale( pwr_VoltageScale_t voltageScale )
{
    pwr_RequestState_t retState     = PWR_REQUEST_ERROR;
    pwr_VoltageScale_t actualScale  = PWR_VOLTAGE_SCALE_CNT;
    pwr_RequestState_t readState    = Pwr_Reg_Get_VoltageScale( &actualScale );

    if( ( PWR_VOLTAGE_SCALE_CNT > voltageScale ) &&
        ( PWR_REQUEST_OK        == readState   )    )
    {
        pwr_VoltageScale_t stepScale = actualScale;

        retState = PWR_REQUEST_OK;

        for( uint32_t stepCnt = 0u;
             ( (uint32_t)PWR_VOLTAGE_SCALE_CNT > stepCnt ) &&
             ( PWR_REQUEST_OK == retState                ) &&
             ( voltageScale   != stepScale               );
             stepCnt++ )
        {
            pwr_FunctionState_t raise = PWR_FUNCTION_INACTIVE;

            if( voltageScale < stepScale )
            {
                /* Higher performance, the scale with the lower index */
                stepScale = (pwr_VoltageScale_t)( (uint32_t)stepScale - 1u );
                raise     = PWR_FUNCTION_ACTIVE;
            }
            else
            {
                /* Lower performance, the scale with the higher index */
                stepScale = (pwr_VoltageScale_t)( (uint32_t)stepScale + 1u );
                raise     = PWR_FUNCTION_INACTIVE;
            }

            retState = Pwr_Reg_Set_VosStep( stepScale, raise );
        }
    }
    else
    {
        /* Invalid scale or the actual scale is not readable, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the supply of VCORE from PWR_SCCR.
 *
 * \param regulatorMode [out]: Bypass when BYPASS is set, otherwise the regulator the package uses
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer or when the
 *         register does not tell any regulator
 */
pwr_RequestState_t Pwr_Reg_Get_RegulatorMode( pwr_RegulatorMode_t * const regulatorMode )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != regulatorMode )
    {
        const uint32_t bypassBit = Pwr_Get_RegBit( PWR_REG_SCCR, PWR_SCCR_BYPASS );
        const uint32_t ldoBit    = Pwr_Get_RegBit( PWR_REG_SCCR, PWR_SCCR_LDOEN );
#if defined(PWR_LINE_HAS_SMPS)
        const uint32_t smpsBit   = Pwr_Get_RegBit( PWR_REG_SCCR, PWR_SCCR_SMPSEN );
#endif

        retState = PWR_REQUEST_OK;

        if( PWR_REG_BITS_CLEAR != bypassBit )
        {
            *regulatorMode = PWR_REGULATOR_BYPASS;
        }
#if defined(PWR_LINE_HAS_SMPS)
        else if( PWR_REG_BITS_CLEAR != smpsBit )
        {
            *regulatorMode = PWR_REGULATOR_SMPS;
        }
#endif
        else if( PWR_REG_BITS_CLEAR != ldoBit )
        {
            *regulatorMode = PWR_REGULATOR_LDO;
        }
        else
        {
            /* The register does not tell any regulator */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Bypasses the internal regulator (external VCORE supply).
 *
 * PWR_SCCR.BYPASS is written once after the power-on reset, a write after the register is locked
 * is ignored and reported by the read-back.
 *
 * \return \ref PWR_REQUEST_OK if BYPASS reads back as set, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_Bypass( void )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_SCCR, PWR_SCCR_BYPASS, PWR_FUNCTION_ACTIVE ) );
}


/**
 * \brief Selects the flash memory low-power mode in Stop mode (PWR_PMCR.FLPS).
 *
 * \param flashLowPower [in]: \ref PWR_FUNCTION_ACTIVE puts the flash memory to low-power mode in Stop mode
 * \return \ref PWR_REQUEST_OK if FLPS reads back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_StopFlashLowPower( pwr_FunctionState_t flashLowPower )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_PMCR, PWR_PMCR_FLPS, flashLowPower ) );
}


/**
 * \brief Reads the flash memory low-power mode selection of Stop mode (PWR_PMCR.FLPS).
 *
 * \return \ref PWR_FUNCTION_ACTIVE if the flash memory goes to low-power mode in Stop mode
 */
pwr_FunctionState_t Pwr_Reg_Get_StopFlashLowPower( void )
{
    return ( Pwr_Reg_Get_BitState( PWR_REG_PMCR, PWR_PMCR_FLPS ) );
}


/**
 * \brief Selects the Stop sub-mode for the next deep sleep (PWR_PMCR.SVOS, LPMS = 0).
 *
 * \param stopMode [in]: Stop sub-mode
 * \return \ref PWR_REQUEST_OK if the fields read back as written, otherwise \ref PWR_REQUEST_ERROR
 *         (also for an invalid sub-mode, no register is written then)
 */
pwr_RequestState_t Pwr_Reg_Set_StopMode( pwr_StopMode_t stopMode )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_STOP_CNT > stopMode )
    {
        const uint32_t regMask  = ( PWR_PMCR_SVOS_Msk | PWR_PMCR_LPMS_Msk );
        const uint32_t regValue = ( pwr_SvosFieldValue[stopMode] << PWR_PMCR_SVOS_Pos );

        Pwr_Set_RegVal( PWR_REG_PMCR, regMask, regValue );
        retState = Pwr_Reg_Wait_RegVal( PWR_REG_PMCR, regMask, regValue );
    }
    else
    {
        /* Invalid Stop sub-mode, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects Standby mode for the next deep sleep (PWR_PMCR.LPMS = 1).
 *
 * \return \ref PWR_REQUEST_OK if LPMS reads back as set, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_StandbyMode( void )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_PMCR, PWR_PMCR_LPMS, PWR_FUNCTION_ACTIVE ) );
}


/**
 * \brief Selects Stop mode for the deep sleep (PWR_PMCR.LPMS = 0), the SVOS selection is kept.
 */
void Pwr_Reg_Set_StopDefault( void )
{
    Pwr_Reset_RegBit( PWR_REG_PMCR, PWR_PMCR_LPMS );
}


/**
 * \brief Clears the Standby and Stop flags (PWR_PMCR.CSSF, the bit reads as 0).
 */
void Pwr_Reg_Set_ModeFlagClear( void )
{
    Pwr_Set_RegBit( PWR_REG_PMCR, PWR_PMCR_CSSF );
}


/**
 * \brief Finds the row of the wake-up pin table.
 *
 * \param wakeUpPin [in]: Wake-up pin item
 * \param info      [out]: Row of the table
 * \return \ref PWR_REQUEST_OK if the item is in the table, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Get_WakeUpPinInfo( pwr_WakeUpPin_t wakeUpPin, const pwr_WakeUpPinInfo_t * * const info )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_WAKEUP_PIN_UNUSED > wakeUpPin ) &&
        ( PWR_NULL_PTR         != info        )    )
    {
        for( uint32_t rowIdx = 0u; PWR_WAKEUP_PIN_UNUSED > rowIdx; rowIdx++ )
        {
            if( wakeUpPin == pwr_WakeUpPinList[rowIdx].PinItem )
            {
                *info    = &pwr_WakeUpPinList[rowIdx];
                retState = PWR_REQUEST_OK;
            }
            else
            {
                /* The row describes another pin */
            }
        }
    }
    else
    {
        /* Invalid pin item or pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Arms a wake-up pin: the polarity and the pull are written first, the pin is enabled after them.
 *
 * \param info      [in]: Row of the wake-up pin table
 * \param pinConfig [in]: Polarity and pull of the pin
 * \return \ref PWR_REQUEST_OK if the three fields read back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_WakeUpPin( const pwr_WakeUpPinInfo_t * const info,
                                          const pwr_WakeUpPinConfig_t * const pinConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_NULL_PTR != info      ) &&
        ( PWR_NULL_PTR != pinConfig )    )
    {
        const uint32_t pinEnable   = ( PWR_WUCR_WUPEN1 << info->WkupIndex );
        const uint32_t pinPolarity = ( PWR_WUCR_WUPP1 << info->WkupIndex );
        const uint32_t pinPull     = ( PWR_WUCR_WUPPUPD1_Msk << ( info->WkupIndex * PWR_REG_WUPPUPD_WIDTH ) );
        uint32_t       pullValue   = PWR_REG_WUPPUPD_NONE;
        uint32_t       polValue    = PWR_REG_BITS_CLEAR;

        if( PWR_PULL_UP == pinConfig->Pull )
        {
            pullValue = PWR_REG_WUPPUPD_UP;
        }
        else if( PWR_PULL_DOWN == pinConfig->Pull )
        {
            pullValue = PWR_REG_WUPPUPD_DOWN;
        }
        else
        {
            /* No pull resistor */
            pullValue = PWR_REG_WUPPUPD_NONE;
        }

        if( PWR_WAKEUP_FALLING == pinConfig->Polarity )
        {
            polValue = pinPolarity;
        }
        else
        {
            /* Detection on the high level (rising edge) */
            polValue = PWR_REG_BITS_CLEAR;
        }

        pullValue = ( pullValue << ( PWR_WUCR_WUPPUPD1_Pos + ( info->WkupIndex * PWR_REG_WUPPUPD_WIDTH ) ) );

        Pwr_Set_RegVal( PWR_REG_WUCR, ( pinPolarity | pinPull ), ( polValue | pullValue ) );
        Pwr_Set_RegBit( PWR_REG_WUCR, pinEnable );

        retState = Pwr_Reg_Wait_RegVal( PWR_REG_WUCR,
                                        ( pinEnable | pinPolarity | pinPull ),
                                        ( pinEnable | polValue | pullValue ) );
    }
    else
    {
        /* Invalid pin row or configuration, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disarms a wake-up pin (the enable bit is cleared, the polarity and the pull stay).
 *
 * \param info [in]: Row of the wake-up pin table
 * \return \ref PWR_REQUEST_OK if the enable bit reads back as cleared, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_WakeUpPinInactive( const pwr_WakeUpPinInfo_t * const info )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != info )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_WUCR,
                                            ( PWR_WUCR_WUPEN1 << info->WkupIndex ),
                                            PWR_FUNCTION_INACTIVE );
    }
    else
    {
        /* Invalid pin row, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the configuration of a wake-up pin from PWR_WUCR.
 *
 * \param info      [in]: Row of the wake-up pin table
 * \param pinActive [out]: Enable state of the pin
 * \param pinConfig [out]: Polarity and pull of the pin
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Reg_Get_WakeUpPinConfig( const pwr_WakeUpPinInfo_t * const info,
                                                pwr_FunctionState_t * const pinActive,
                                                pwr_WakeUpPinConfig_t * const pinConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_NULL_PTR != info      ) &&
        ( PWR_NULL_PTR != pinActive ) &&
        ( PWR_NULL_PTR != pinConfig )    )
    {
        const uint32_t pullShift = ( PWR_WUCR_WUPPUPD1_Pos + ( info->WkupIndex * PWR_REG_WUPPUPD_WIDTH ) );
        const uint32_t pullMask  = ( PWR_WUCR_WUPPUPD1_Msk << ( info->WkupIndex * PWR_REG_WUPPUPD_WIDTH ) );
        const uint32_t pinPull   = ( Pwr_Get_RegVal( PWR_REG_WUCR, pullMask ) >> pullShift );
        const uint32_t pinPolBit = Pwr_Get_RegBit( PWR_REG_WUCR, ( PWR_WUCR_WUPP1 << info->WkupIndex ) );

        *pinActive = Pwr_Reg_Get_BitState( PWR_REG_WUCR, ( PWR_WUCR_WUPEN1 << info->WkupIndex ) );

        if( PWR_REG_BITS_CLEAR != pinPolBit )
        {
            pinConfig->Polarity = PWR_WAKEUP_FALLING;
        }
        else
        {
            pinConfig->Polarity = PWR_WAKEUP_RISING;
        }

        if( PWR_REG_WUPPUPD_UP == pinPull )
        {
            pinConfig->Pull = PWR_PULL_UP;
        }
        else if( PWR_REG_WUPPUPD_DOWN == pinPull )
        {
            pinConfig->Pull = PWR_PULL_DOWN;
        }
        else
        {
            /* No pull resistor (the value 3 is reserved) */
            pinConfig->Pull = PWR_PULL_NONE;
        }

        retState = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the wake-up flags of the pins and the Standby and Stop flags as the source mask.
 *
 * \return Mask of \ref pwr_WakeUpSourceBit_t items
 */
pwr_WakeUpSource_t Pwr_Reg_Get_WakeUpSource( void )
{
    pwr_WakeUpSource_t wakeUpSource = (pwr_WakeUpSource_t)PWR_WAKEUP_SOURCE_NONE;
    const uint32_t     pinFlags     = Pwr_Get_RegVal( PWR_REG_WUSR, PWR_REG_WUF_MASK );
    const uint32_t     standbyFlag  = Pwr_Get_RegBit( PWR_REG_PMSR, PWR_PMSR_SBF );
    const uint32_t     stopFlag     = Pwr_Get_RegBit( PWR_REG_PMSR, PWR_PMSR_STOPF );

    wakeUpSource = pinFlags;

    if( PWR_REG_BITS_CLEAR != standbyFlag )
    {
        wakeUpSource |= (pwr_WakeUpSource_t)PWR_WAKEUP_SOURCE_STANDBY;
    }
    else
    {
        /* The system has not been in Standby mode */
    }

    if( PWR_REG_BITS_CLEAR != stopFlag )
    {
        wakeUpSource |= (pwr_WakeUpSource_t)PWR_WAKEUP_SOURCE_STOP;
    }
    else
    {
        /* The system has not been in Stop mode */
    }

    return ( wakeUpSource );
}


/**
 * \brief Clears the wake-up flags of the pins (PWR_WUSCR) and the Standby and Stop flags (PWR_PMCR.CSSF).
 */
void Pwr_Reg_Set_WakeUpFlagClear( void )
{
    Pwr_Set_RegBit( PWR_REG_WUSCR, PWR_WUSCR_CWUF_Msk );
    Pwr_Reg_Set_ModeFlagClear();
}


/**
 * \brief Selects whether the content of a retention block is kept in Stop mode.
 *
 * The shut-off bit of the SRAM blocks is cleared to keep the content. The backup SRAM is kept in
 * Standby and VBAT modes by the backup regulator (PWR_BDCR.BREN) which has to be ready (BRRDY).
 * BREN is write protected until the write access to the backup domain is enabled.
 *
 * \param sramId    [in]: Retention block
 * \param retention [in]: \ref PWR_FUNCTION_ACTIVE keeps the content
 * \return \ref PWR_REQUEST_OK if the bit reads back as written (the regulator is ready), otherwise
 *         \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_SramRetention( pwr_SramId_t sramId, pwr_FunctionState_t retention )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;
    uint32_t           shutOffMask = PWR_REG_BITS_CLEAR;

    if( PWR_SRAM_BACKUP == sramId )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_BREN, retention );

        if( ( PWR_REQUEST_OK == retState ) &&
            ( PWR_FUNCTION_ACTIVE == retention ) )
        {
            retState = Pwr_Reg_Wait_RegVal( PWR_REG_BDSR, PWR_BDSR_BRRDY, PWR_BDSR_BRRDY );
        }
        else
        {
            /* The regulator is switched off or the bit was not written, no wait is needed */
        }
    }
    else
    {
        const pwr_RequestState_t maskState = Pwr_Reg_Get_SramMask( sramId, &shutOffMask );

        if( PWR_REQUEST_OK == maskState )
        {
            pwr_FunctionState_t shutOff = PWR_FUNCTION_ACTIVE;

            if( PWR_FUNCTION_ACTIVE == retention )
            {
                shutOff = PWR_FUNCTION_INACTIVE;
            }
            else
            {
                /* The content is lost in Stop mode, the shut-off bit is set */
                shutOff = PWR_FUNCTION_ACTIVE;
            }

            retState = Pwr_Reg_Set_BitVerified( PWR_REG_PMCR, shutOffMask, shutOff );
        }
        else
        {
            /* Invalid retention block, no register is written */
            retState = PWR_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Reads whether the content of a retention block is kept.
 *
 * \param sramId    [in]: Retention block
 * \param retention [out]: \ref PWR_FUNCTION_ACTIVE if the content is kept
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for an invalid block or pointer
 */
pwr_RequestState_t Pwr_Reg_Get_SramRetention( pwr_SramId_t sramId, pwr_FunctionState_t * const retention )
{
    pwr_RequestState_t retState    = PWR_REQUEST_ERROR;
    uint32_t           shutOffMask = PWR_REG_BITS_CLEAR;

    if( PWR_NULL_PTR != retention )
    {
        if( PWR_SRAM_BACKUP == sramId )
        {
            *retention = Pwr_Reg_Get_BitState( PWR_REG_BDCR, PWR_BDCR_BREN );
            retState   = PWR_REQUEST_OK;
        }
        else
        {
            const pwr_RequestState_t maskState = Pwr_Reg_Get_SramMask( sramId, &shutOffMask );

            if( PWR_REQUEST_OK == maskState )
            {
                const pwr_FunctionState_t shutOff = Pwr_Reg_Get_BitState( PWR_REG_PMCR, shutOffMask );

                if( PWR_FUNCTION_ACTIVE == shutOff )
                {
                    *retention = PWR_FUNCTION_INACTIVE;
                }
                else
                {
                    /* The shut-off bit is cleared, the content is kept */
                    *retention = PWR_FUNCTION_ACTIVE;
                }

                retState = PWR_REQUEST_OK;
            }
            else
            {
                /* Invalid retention block, no action required */
                retState = PWR_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects whether the output state of a group of I/Os is retained in Standby mode.
 *
 * \param ioGroup   [in]: Group of I/Os
 * \param retention [in]: \ref PWR_FUNCTION_ACTIVE retains the output state
 * \return \ref PWR_REQUEST_OK if the bit reads back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_IoRetention( pwr_IoGroup_t ioGroup, pwr_FunctionState_t retention )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_IO_GROUP_GENERAL == ioGroup )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_IORETR, PWR_IORETR_IORETEN, retention );
    }
    else if( PWR_IO_GROUP_JTAG == ioGroup )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_IORETR, PWR_IORETR_JTAGIORETEN, retention );
    }
    else
    {
        /* Invalid I/O group, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the threshold of the programmable voltage detector (PWR_VMCR.PLS).
 *
 * \param pvdLevel [in]: Threshold, the field value is the item value
 * \return \ref PWR_REQUEST_OK if the field reads back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_PvdLevel( pwr_PvdLevel_t pvdLevel )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_PVD_LEVEL_CNT > pvdLevel )
    {
        const uint32_t regValue = ( (uint32_t)pvdLevel << PWR_VMCR_PLS_Pos );

        Pwr_Set_RegVal( PWR_REG_VMCR, PWR_VMCR_PLS_Msk, regValue );
        retState = Pwr_Reg_Wait_RegVal( PWR_REG_VMCR, PWR_VMCR_PLS_Msk, regValue );
    }
    else
    {
        /* Invalid threshold, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the threshold of the programmable voltage detector (PWR_VMCR.PLS).
 *
 * \param pvdLevel [out]: Threshold
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Reg_Get_PvdLevel( pwr_PvdLevel_t * const pvdLevel )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != pvdLevel )
    {
        *pvdLevel = (pwr_PvdLevel_t)( Pwr_Get_RegVal( PWR_REG_VMCR, PWR_VMCR_PLS_Msk ) >> PWR_VMCR_PLS_Pos );
        retState  = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the threshold of the analog voltage detector (PWR_VMCR.ALS).
 *
 * \param avdLevel [in]: Threshold, the field value is the item value
 * \return \ref PWR_REQUEST_OK if the field reads back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_AvdLevel( pwr_AvdLevel_t avdLevel )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_AVD_LEVEL_CNT > avdLevel )
    {
        const uint32_t regValue = ( (uint32_t)avdLevel << PWR_VMCR_ALS_Pos );

        Pwr_Set_RegVal( PWR_REG_VMCR, PWR_VMCR_ALS_Msk, regValue );
        retState = Pwr_Reg_Wait_RegVal( PWR_REG_VMCR, PWR_VMCR_ALS_Msk, regValue );
    }
    else
    {
        /* Invalid threshold, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the threshold of the analog voltage detector (PWR_VMCR.ALS).
 *
 * \param avdLevel [out]: Threshold
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Reg_Get_AvdLevel( pwr_AvdLevel_t * const avdLevel )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != avdLevel )
    {
        *avdLevel = (pwr_AvdLevel_t)( Pwr_Get_RegVal( PWR_REG_VMCR, PWR_VMCR_ALS_Msk ) >> PWR_VMCR_ALS_Pos );
        retState  = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the programmable voltage detector (PWR_VMCR.PVDE).
 *
 * \param pvdState [in]: \ref PWR_FUNCTION_ACTIVE enables the detector
 * \return \ref PWR_REQUEST_OK if the bit reads back as written (it can be locked by the SBS module)
 */
pwr_RequestState_t Pwr_Reg_Set_PvdState( pwr_FunctionState_t pvdState )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_VMCR, PWR_VMCR_PVDEN, pvdState ) );
}


/**
 * \brief Enables or disables the analog voltage detector (PWR_VMCR.AVDEN).
 *
 * \param avdState [in]: \ref PWR_FUNCTION_ACTIVE enables the detector
 * \return \ref PWR_REQUEST_OK if the bit reads back as written
 */
pwr_RequestState_t Pwr_Reg_Set_AvdState( pwr_FunctionState_t avdState )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_VMCR, PWR_VMCR_AVDEN, avdState ) );
}


/**
 * \brief Reads the enable state of the programmable voltage detector.
 *
 * \return \ref PWR_FUNCTION_ACTIVE if the detector is enabled
 */
pwr_FunctionState_t Pwr_Reg_Get_PvdState( void )
{
    return ( Pwr_Reg_Get_BitState( PWR_REG_VMCR, PWR_VMCR_PVDEN ) );
}


/**
 * \brief Reads the enable state of the analog voltage detector.
 *
 * \return \ref PWR_FUNCTION_ACTIVE if the detector is enabled
 */
pwr_FunctionState_t Pwr_Reg_Get_AvdState( void )
{
    return ( Pwr_Reg_Get_BitState( PWR_REG_VMCR, PWR_VMCR_AVDEN ) );
}


/**
 * \brief Enables or disables the monitoring of the VBAT voltage and of the temperature (PWR_BDCR.MONEN).
 *
 * MONEN is write protected until the write access to the backup domain is enabled.
 *
 * \param monitorState [in]: \ref PWR_FUNCTION_ACTIVE enables the monitoring
 * \return \ref PWR_REQUEST_OK if the bit reads back as written
 */
pwr_RequestState_t Pwr_Reg_Set_BackupMonitor( pwr_FunctionState_t monitorState )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_MONEN, monitorState ) );
}


/**
 * \brief Reads whether the monitoring of the VBAT voltage and of the temperature is enabled (PWR_BDCR.MONEN).
 *
 * \return \ref PWR_FUNCTION_ACTIVE if the monitoring is enabled
 */
pwr_FunctionState_t Pwr_Reg_Get_BackupMonitor( void )
{
    return ( Pwr_Reg_Get_BitState( PWR_REG_BDCR, PWR_BDCR_MONEN ) );
}


/**
 * \brief Reads the state of a supply from the status registers.
 *
 * The supply is valid when the flag of its monitor does not report a violation. The monitor of the
 * supply has to be enabled, otherwise the flag has no meaning and the request is rejected.
 *
 * \param supply      [in]: Supply
 * \param supplyState [out]: State of the supply
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for an invalid supply or pointer
 *         or when the monitor of the supply is not enabled
 */
pwr_RequestState_t Pwr_Reg_Get_SupplyState( pwr_Supply_t supply, pwr_SupplyState_t * const supplyState )
{
    pwr_RequestState_t  retState     = PWR_REQUEST_ERROR;
    pwr_FunctionState_t monitorState = PWR_FUNCTION_INACTIVE;
    pwr_SupplyState_t   state        = PWR_SUPPLY_STATE_VALID;
    uint32_t            flagValue    = PWR_REG_BITS_CLEAR;

    if( ( PWR_SUPPLY_CNT > supply ) &&
        ( PWR_NULL_PTR  != supplyState ) )
    {
        retState = PWR_REQUEST_OK;

        if( PWR_SUPPLY_VDD == supply )
        {
            monitorState = Pwr_Reg_Get_BitState( PWR_REG_VMCR, PWR_VMCR_PVDEN );
            flagValue    = Pwr_Get_RegBit( PWR_REG_VMSR, PWR_VMSR_PVDO );
        }
        else if( PWR_SUPPLY_ANALOG == supply )
        {
            monitorState = Pwr_Reg_Get_BitState( PWR_REG_VMCR, PWR_VMCR_AVDEN );
            flagValue    = Pwr_Get_RegBit( PWR_REG_VMSR, PWR_VMSR_AVDO );
        }
#if defined(PWR_LINE_HAS_USB_SUPPLY)
        else if( PWR_SUPPLY_USB == supply )
        {
            monitorState = Pwr_Reg_Get_BitState( PWR_REG_USBSCR, PWR_USBSCR_USB33DEN );
            /* The ready flag is set when the supply is valid, the other flags report a violation */
            flagValue    = ( Pwr_Get_RegBit( PWR_REG_VMSR, PWR_VMSR_USB33RDY ) ^ PWR_VMSR_USB33RDY );
        }
#endif
        else if( PWR_SUPPLY_IO2 == supply )
        {
            monitorState = PWR_FUNCTION_ACTIVE;
            flagValue    = ( Pwr_Get_RegBit( PWR_REG_VMSR, PWR_VMSR_VDDIO2RDY ) ^ PWR_VMSR_VDDIO2RDY );
        }
        else if( PWR_SUPPLY_BACKUP == supply )
        {
            monitorState = Pwr_Reg_Get_BitState( PWR_REG_BDCR, PWR_BDCR_MONEN );
            flagValue    = Pwr_Get_RegBit( PWR_REG_BDSR, PWR_REG_VBAT_FLAG_MASK );
        }
        else
        {
            /* The temperature, monitored together with the VBAT voltage */
            monitorState = Pwr_Reg_Get_BitState( PWR_REG_BDCR, PWR_BDCR_MONEN );
            flagValue    = Pwr_Get_RegBit( PWR_REG_BDSR, PWR_REG_TEMP_FLAG_MASK );
        }

        if( PWR_FUNCTION_ACTIVE == monitorState )
        {
            if( PWR_REG_BITS_CLEAR != flagValue )
            {
                state = PWR_SUPPLY_STATE_NOT_VALID;
            }
            else
            {
                state = PWR_SUPPLY_STATE_VALID;
            }

            *supplyState = state;
        }
        else
        {
            /* The monitor of the supply is disabled, the flag has no meaning */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid supply or pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the write access to the backup domain (PWR_DBPCR.DBP).
 *
 * \param accessState [in]: \ref PWR_FUNCTION_ACTIVE enables the write access
 * \return \ref PWR_REQUEST_OK if the bit reads back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_BackupAccess( pwr_FunctionState_t accessState )
{
    return ( Pwr_Reg_Set_BitVerified( PWR_REG_DBPCR, PWR_DBPCR_DBP, accessState ) );
}


/**
 * \brief Reads whether the write access to the backup domain is enabled (PWR_DBPCR.DBP).
 *
 * \return \ref PWR_FUNCTION_ACTIVE if the write access is enabled
 */
pwr_FunctionState_t Pwr_Reg_Get_BackupAccess( void )
{
    return ( Pwr_Reg_Get_BitState( PWR_REG_DBPCR, PWR_DBPCR_DBP ) );
}


/**
 * \brief Selects the charging of the VBAT battery (PWR_BDCR.VBE and VBRS).
 *
 * The resistor is selected before the charging is enabled.
 *
 * \param vbatCharging [in]: Charging resistor or charging disabled
 * \return \ref PWR_REQUEST_OK if the bits read back as written, otherwise \ref PWR_REQUEST_ERROR
 */
pwr_RequestState_t Pwr_Reg_Set_VbatCharging( pwr_VbatCharging_t vbatCharging )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_VBAT_CHARGING_OFF == vbatCharging )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_VBE, PWR_FUNCTION_INACTIVE );
    }
    else if( PWR_VBAT_CHARGING_5K == vbatCharging )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_VBRS, PWR_FUNCTION_INACTIVE );

        if( PWR_REQUEST_OK == retState )
        {
            retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_VBE, PWR_FUNCTION_ACTIVE );
        }
        else
        {
            /* The resistor was not selected, the charging stays as it was */
        }
    }
    else if( PWR_VBAT_CHARGING_1K5 == vbatCharging )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_VBRS, PWR_FUNCTION_ACTIVE );

        if( PWR_REQUEST_OK == retState )
        {
            retState = Pwr_Reg_Set_BitVerified( PWR_REG_BDCR, PWR_BDCR_VBE, PWR_FUNCTION_ACTIVE );
        }
        else
        {
            /* The resistor was not selected, the charging stays as it was */
        }
    }
    else
    {
        /* Invalid charging selection, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Validates or invalidates the VDDUSB supply (PWR_USBSCR).
 *
 * Activation: the VDDUSB monitor is enabled (USB33DEN), the supply has to become ready
 * (VMSR.USB33RDY) within the timeout, then it is declared valid (USB33SV). When it does not become
 * ready the monitor is disabled again. Deactivation clears USB33SV and USB33DEN.
 *
 * \param supplyState [in]: \ref PWR_FUNCTION_ACTIVE validates the supply
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the supply is not ready, a bit does
 *         not read back as written or the device line has no VDDUSB supply
 */
pwr_RequestState_t Pwr_Reg_Set_UsbSupply( pwr_FunctionState_t supplyState )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

#if defined(PWR_LINE_HAS_USB_SUPPLY)
    if( PWR_FUNCTION_ACTIVE == supplyState )
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_USBSCR, PWR_USBSCR_USB33DEN, PWR_FUNCTION_ACTIVE );

        if( PWR_REQUEST_OK == retState )
        {
            retState = Pwr_Reg_Wait_RegVal( PWR_REG_VMSR, PWR_VMSR_USB33RDY, PWR_VMSR_USB33RDY );
        }
        else
        {
            /* The monitor could not be enabled, no wait is needed */
        }

        if( PWR_REQUEST_OK == retState )
        {
            retState = Pwr_Reg_Set_BitVerified( PWR_REG_USBSCR, PWR_USBSCR_USB33SV, PWR_FUNCTION_ACTIVE );
        }
        else
        {
            /* The supply is not ready, the monitor is disabled again */
            Pwr_Reset_RegBit( PWR_REG_USBSCR, PWR_USBSCR_USB33DEN );
        }
    }
    else
    {
        retState = Pwr_Reg_Set_BitVerified( PWR_REG_USBSCR, PWR_USBSCR_USB33SV, PWR_FUNCTION_INACTIVE );

        if( PWR_REQUEST_OK == retState )
        {
            retState = Pwr_Reg_Set_BitVerified( PWR_REG_USBSCR, PWR_USBSCR_USB33DEN, PWR_FUNCTION_INACTIVE );
        }
        else
        {
            /* The supply could not be invalidated, the monitor stays enabled */
        }
    }
#else
    (void)supplyState;
    retState = PWR_REQUEST_ERROR;
#endif

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Waits until the register (bit field) reads back the required value.
 *
 * \param regId    [in]: PWR register ID
 * \param regMask  [in]: Mask of the checked bits
 * \param regValue [in]: Required value of the masked bits
 *
 * \return Returns \ref PWR_REQUEST_OK if the value was read back within the
 *         timeout. Otherwise returns \ref PWR_REQUEST_ERROR.
 */
static pwr_RequestState_t Pwr_Reg_Wait_RegVal( pwr_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; PWR_REG_READY_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
    {
        const uint32_t actualValue = Pwr_Get_RegVal( regId, regMask );

        if( ( regValue & regMask ) == actualValue )
        {
            retState = PWR_REQUEST_OK;
            break;
        }
        else
        {
            /* Register has not been updated yet */
            retState = PWR_REQUEST_ERROR;
        }
    }

    return ( retState );
}


/**
 * \brief Returns the state of a register bit field as module enumeration.
 *
 * \param regId   [in]: PWR register ID
 * \param bitMask [in]: Mask of the checked bits
 *
 * \return \ref PWR_FUNCTION_ACTIVE if any bit of the mask is set, otherwise
 *         \ref PWR_FUNCTION_INACTIVE (also for an invalid register ID).
 */
static pwr_FunctionState_t Pwr_Reg_Get_BitState( pwr_RegId_t regId, uint32_t bitMask )
{
    pwr_FunctionState_t retState = PWR_FUNCTION_INACTIVE;
    const uint32_t      bitValue = Pwr_Get_RegBit( regId, bitMask );

    if( PWR_REG_BITS_CLEAR != bitValue )
    {
        retState = PWR_FUNCTION_ACTIVE;
    }
    else
    {
        retState = PWR_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Sets or clears a register bit and verifies it by read-back.
 *
 * \param regId    [in]: PWR register ID
 * \param bitMask  [in]: Mask of the bit field
 * \param bitState [in]: \ref PWR_FUNCTION_ACTIVE sets the bits, otherwise they are cleared
 *
 * \return \ref PWR_REQUEST_OK if the bits read back as written within the timeout
 */
static pwr_RequestState_t Pwr_Reg_Set_BitVerified( pwr_RegId_t regId,
                                                   uint32_t bitMask,
                                                   pwr_FunctionState_t bitState )
{
    uint32_t bitValue = PWR_REG_BITS_CLEAR;

    if( PWR_FUNCTION_ACTIVE == bitState )
    {
        bitValue = bitMask;
    }
    else
    {
        /* The bits are cleared */
        bitValue = PWR_REG_BITS_CLEAR;
    }

    Pwr_Set_RegVal( regId, bitMask, bitValue );

    return ( Pwr_Reg_Wait_RegVal( regId, bitMask, bitValue ) );
}


/**
 * \brief Writes one voltage scale and waits until VCORE has reached it.
 *
 * \param voltageScale [in]: Voltage scale of the step
 * \param raise        [in]: \ref PWR_FUNCTION_ACTIVE when the performance is raised (VOSRDY is checked then)
 *
 * \return \ref PWR_REQUEST_OK if the scale is applied and ready within the timeout
 */
static pwr_RequestState_t Pwr_Reg_Set_VosStep( pwr_VoltageScale_t voltageScale, pwr_FunctionState_t raise )
{
    pwr_RequestState_t retState      = PWR_REQUEST_ERROR;
    const uint32_t     fieldValue    = pwr_VosFieldValue[voltageScale];
    uint32_t           readyMask     = ( PWR_VOSSR_ACTVOS_Msk | PWR_VOSSR_ACTVOSRDY_Msk );
    uint32_t           readyValue    = ( ( fieldValue << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY_Msk );

    if( PWR_FUNCTION_ACTIVE == raise )
    {
        /* VOSRDY is valid only when switching from a low to a high-voltage scale */
        readyMask  |= PWR_VOSSR_VOSRDY_Msk;
        readyValue |= PWR_VOSSR_VOSRDY_Msk;
    }
    else
    {
        /* Lower performance, only ACTVOS and ACTVOSRDY are checked */
    }

    Pwr_Set_RegVal( PWR_REG_VOSCR, PWR_VOSCR_VOS_Msk, ( fieldValue << PWR_VOSCR_VOS_Pos ) );

    retState = Pwr_Reg_Wait_RegVal( PWR_REG_VOSSR, readyMask, readyValue );

    return ( retState );
}


/**
 * \brief Finds the shut-off bit of a retention block (the backup SRAM has none).
 *
 * \param sramId      [in]: Retention block
 * \param shutOffMask [out]: Bit of PWR_PMCR that shuts the block off in Stop mode
 * \return \ref PWR_REQUEST_OK if the block is in the table, otherwise \ref PWR_REQUEST_ERROR
 */
static pwr_RequestState_t Pwr_Reg_Get_SramMask( pwr_SramId_t sramId, uint32_t * const shutOffMask )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != shutOffMask )
    {
        for( uint32_t rowIdx = 0u; (uint32_t)PWR_SRAM_BACKUP > rowIdx; rowIdx++ )
        {
            if( sramId == pwr_SramList[rowIdx].SramId )
            {
                *shutOffMask = pwr_SramList[rowIdx].ShutOffMask;
                retState     = PWR_REQUEST_OK;
            }
            else
            {
                /* The row describes another block */
            }
        }
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
