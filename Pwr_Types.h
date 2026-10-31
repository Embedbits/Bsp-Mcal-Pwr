/**
 * \author Mr.Nobody
 * \file Pwr_Types.h
 * \ingroup Pwr
 * \brief Pwr module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file.
 *
 * The lists of this file contain only the items the device line has. The device lines are
 * told apart by the device line macros of the build (STM32H503xx ... STM32H5F5xx), because
 * this file does not include the CMSIS device header. The composite feature macros
 * (PWR_LINE_*) below are checked against the CMSIS bit definitions by Pwr_Reg.c.
 *
 * Wake-up pins: the WKUPx inputs are fixed pins (no alternate function), the pin of an input is
 * given in the description of its item. Source: CubeMX database of the device line
 * (PWR_WKUPx signals) and RM0481 / RM0492 / RM0517 chapter "Power control (PWR)".
 *
 */

/**
 * \defgroup Pwr Pwr
 * \brief Pwr module
 */

#ifndef PWR_PWR_TYPES_H
#define PWR_PWR_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define PWR_NULL_PTR                        ( ( void* ) 0u )

/**
 * \brief Composite device line feature macros, defined once here so that every guard below
 *        is a single condition.
 *
 * PWR_LINE_HAS_SMPS:        SMPS regulator status bit (PWR_SCCR.SMPSEN) in the PWR register set.
 * PWR_LINE_HAS_USB_SUPPLY:  VDDUSB supply (PWR_USBSCR, PWR_VMSR.USB33RDY), all lines except STM32H503.
 * PWR_LINE_HAS_WKUP_6_8:    Wake-up inputs WKUP6 .. WKUP8, all lines except STM32H503 (WKUP1 .. WKUP5).
 * PWR_LINE_HAS_WKUP_3:      Wake-up input WKUP3 with a pin (PC1 on STM32H503, PI8 on the lines with port I).
 * PWR_LINE_SRAM2_SINGLE:    SRAM2 shut-off in one bit (PWR_PMCR.SRAM2SO), STM32H503.
 * PWR_LINE_SRAM2_48:        SRAM2 48-Kbyte shut-off in one bit (PWR_PMCR.SRAM2_48SO).
 * PWR_LINE_SRAM2_48_SPLIT:  SRAM2 48-Kbyte shut-off in two bits (PWR_PMCR.SRAM2_48LSO / SRAM2_48HSO).
 * PWR_LINE_SRAM2_16:        SRAM2 16-Kbyte shut-off in one bit (PWR_PMCR.SRAM2_16SO).
 * PWR_LINE_SRAM2_16_SPLIT:  SRAM2 16-Kbyte shut-off in two bits (PWR_PMCR.SRAM2_16LSO / SRAM2_16HSO).
 * PWR_LINE_HAS_SRAM3:       AHB SRAM3 shut-off bit, all lines except STM32H503.
 * PWR_LINE_HAS_SRAM4_5:     AHB SRAM4 and SRAM5 shut-off bits (STM32H5E4 / H5E5 / H5F4 / H5F5).
 * PWR_LINE_HAS_ETH_RAM:     Ethernet RAM shut-off bit (lines with the Ethernet MAC).
 * PWR_LINE_HAS_LTDC_RAM:    LTDC FIFO RAM shut-off bit (STM32H5E4 / H5E5 / H5F4 / H5F5).
 */
#if defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_HAS_SMPS                   ( 1u )
#endif

#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_HAS_USB_SUPPLY             ( 1u )
#define PWR_LINE_HAS_WKUP_6_8               ( 1u )
#define PWR_LINE_HAS_SRAM3                  ( 1u )
#endif

#if defined(STM32H503xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_HAS_WKUP_3                 ( 1u )
#endif

#if defined(STM32H503xx)
#define PWR_LINE_SRAM2_SINGLE               ( 1u )
#endif

#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
#define PWR_LINE_SRAM2_48                   ( 1u )
#endif

#if defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_SRAM2_48_SPLIT             ( 1u )
#define PWR_LINE_HAS_SRAM4_5                ( 1u )
#define PWR_LINE_HAS_LTDC_RAM               ( 1u )
#endif

#if defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx)
#define PWR_LINE_SRAM2_16                   ( 1u )
#endif

#if defined(STM32H523xx) || \
    defined(STM32H533xx) || \
    defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_SRAM2_16_SPLIT             ( 1u )
#endif

#if defined(STM32H543xx) || \
    defined(STM32H553xx) || \
    defined(STM32H562xx) || \
    defined(STM32H563xx) || \
    defined(STM32H573xx) || \
    defined(STM32H5E4xx) || \
    defined(STM32H5E5xx) || \
    defined(STM32H5F4xx) || \
    defined(STM32H5F5xx)
#define PWR_LINE_HAS_ETH_RAM                ( 1u )
#endif

/* ========================== EXPORTED MACROS =============================== */

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t pwr_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t pwr_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t pwr_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    pwr_MajorVersion_t Major; /**< Major version */
    pwr_MinorVersion_t Minor; /**< Minor version */
    pwr_PatchVersion_t Patch; /**< Patch version */
}   pwr_ModuleVersion_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    PWR_REQUEST_ERROR = 0u, /**< Processing request failed  */
    PWR_REQUEST_OK          /**< Processing request succeed */
}   pwr_RequestState_t;


/** Function status enumeration */
typedef enum
{
    PWR_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    PWR_FUNCTION_ACTIVE         /**< Function status is active   */
}   pwr_FunctionState_t;


/** Flag states enumeration */
typedef enum
{
    PWR_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    PWR_FLAG_ACTIVE         /**< Active flag state   */
}   pwr_FlagState_t;


/** \brief Interrupt priority type definition */
typedef uint32_t pwr_IrqPrio_t;


/** \brief Voltage scale of the core (VCORE) in Run mode, 0 = the highest performance */
typedef enum
{
    PWR_VOLTAGE_SCALE_0 = 0u, /**< Scale 0 (VOS0), the highest performance, the highest consumption */
    PWR_VOLTAGE_SCALE_1,      /**< Scale 1 (VOS1)                                                  */
    PWR_VOLTAGE_SCALE_2,      /**< Scale 2 (VOS2)                                                  */
    PWR_VOLTAGE_SCALE_3,      /**< Scale 3 (VOS3), default after the reset, the lowest consumption  */
    PWR_VOLTAGE_SCALE_CNT     /**< Count of the voltage scales of the device                        */
}   pwr_VoltageScale_t;


/**
 * \brief Supply of VCORE.
 *
 * The internal regulator (LDO or SMPS) is selected by the hardware according to the package, the
 * software can only bypass it once after the power-on reset (the external VCORE supply).
 */
typedef enum
{
    PWR_REGULATOR_LDO = 0u,  /**< Internal LDO regulator (selected by the package)                  */
#if defined(PWR_LINE_HAS_SMPS)
    PWR_REGULATOR_SMPS,      /**< Internal SMPS step-down converter (selected by the package)      */
#endif
    PWR_REGULATOR_BYPASS,    /**< Regulator bypassed, external VCORE supply (written once after POR) */
    PWR_REGULATOR_CNT        /**< Count of the regulator modes of the device                        */
}   pwr_RegulatorMode_t;


/** \brief Stop sub-mode (voltage scale of the core in Stop mode, SVOS) */
typedef enum
{
    PWR_STOP_0 = 0u, /**< Stop with SVOS3: the peripherals can wake up, the fastest restart          */
    PWR_STOP_1,      /**< Stop with SVOS4: the regulator in low-power mode, only EXTI wake-up sources */
    PWR_STOP_2,      /**< Stop with SVOS5: the lowest consumption, the flash in low-power mode       */
    PWR_STOP_CNT     /**< Count of the Stop sub-modes of the device                                   */
}   pwr_StopMode_t;


/** \brief Instruction that puts the core to Sleep, which defines how it wakes up */
typedef enum
{
    PWR_WAKEUP_INTERRUPT = 0u, /**< Wait for interrupt (WFI): an interrupt wakes up the core */
    PWR_WAKEUP_EVENT           /**< Wait for event (WFE): an event wakes up the core         */
}   pwr_WakeUpEvent_t;


/**
 * \brief Wake-up input (WKUPx) of the Standby mode, every item is one fixed pin.
 *
 * Only the inputs of the device line that have a pin are in the list.
 */
typedef enum
{
    PWR_WAKEUP_PIN_1 = 0u,   /**< WKUP1 on PA0                                       */
    PWR_WAKEUP_PIN_2,        /**< WKUP2 on PA2                                       */
#if defined(PWR_LINE_HAS_WKUP_3)
    PWR_WAKEUP_PIN_3,        /**< WKUP3 on PC1 (STM32H503) or PI8 (the other lines)  */
#endif
    PWR_WAKEUP_PIN_4,        /**< WKUP4 on PC13                                      */
    PWR_WAKEUP_PIN_5,        /**< WKUP5 on PB7                                       */
#if defined(PWR_LINE_HAS_WKUP_6_8)
    PWR_WAKEUP_PIN_6,        /**< WKUP6 on PC1                                       */
    PWR_WAKEUP_PIN_7,        /**< WKUP7 on PD2                                       */
    PWR_WAKEUP_PIN_8,        /**< WKUP8 on PD3                                       */
#endif
    PWR_WAKEUP_PIN_UNUSED,   /**< No wake-up input selected                          */
    PWR_WAKEUP_PIN_CNT       /**< Count of the wake-up inputs of the device          */
}   pwr_WakeUpPin_t;


/** \brief Polarity of the wake-up pin event */
typedef enum
{
    PWR_WAKEUP_RISING = 0u, /**< Detection on the high level (rising edge) */
    PWR_WAKEUP_FALLING      /**< Detection on the low level (falling edge) */
}   pwr_WakeUpPolarity_t;


/** \brief Pull resistor of the wake-up pin (kept in Standby mode) */
typedef enum
{
    PWR_PULL_NONE = 0u, /**< No pull resistor */
    PWR_PULL_UP,        /**< Pull-up          */
    PWR_PULL_DOWN       /**< Pull-down        */
}   pwr_Pull_t;


/** \brief Configuration of a wake-up pin */
typedef struct
{
    pwr_WakeUpPolarity_t Polarity; /**< Polarity of the event that wakes up the MCU */
    pwr_Pull_t           Pull;     /**< Pull resistor of the pin                    */
}   pwr_WakeUpPinConfig_t;


/**
 * \brief Bit of the wake-up source mask (\ref pwr_WakeUpSource_t).
 *
 * The RTC and TAMP events have their own flags in the Rtc and Tamp modules, the PWR reports
 * only the wake-up pins and the flags of the low-power modes.
 */
typedef enum
{
    PWR_WAKEUP_SOURCE_NONE    = 0x00000000u, /**< No wake-up source flag is set            */
    PWR_WAKEUP_SOURCE_PIN_1   = 0x00000001u, /**< Wake-up flag of WKUP1                    */
    PWR_WAKEUP_SOURCE_PIN_2   = 0x00000002u, /**< Wake-up flag of WKUP2                    */
#if defined(PWR_LINE_HAS_WKUP_3)
    PWR_WAKEUP_SOURCE_PIN_3   = 0x00000004u, /**< Wake-up flag of WKUP3                    */
#endif
    PWR_WAKEUP_SOURCE_PIN_4   = 0x00000008u, /**< Wake-up flag of WKUP4                    */
    PWR_WAKEUP_SOURCE_PIN_5   = 0x00000010u, /**< Wake-up flag of WKUP5                    */
#if defined(PWR_LINE_HAS_WKUP_6_8)
    PWR_WAKEUP_SOURCE_PIN_6   = 0x00000020u, /**< Wake-up flag of WKUP6                    */
    PWR_WAKEUP_SOURCE_PIN_7   = 0x00000040u, /**< Wake-up flag of WKUP7                    */
    PWR_WAKEUP_SOURCE_PIN_8   = 0x00000080u, /**< Wake-up flag of WKUP8                    */
#endif
    PWR_WAKEUP_SOURCE_STANDBY = 0x00000100u, /**< The system has been in Standby mode (SBF) */
    PWR_WAKEUP_SOURCE_STOP    = 0x00000200u  /**< The system has been in Stop mode (STOPF)  */
}   pwr_WakeUpSourceBit_t;


/** \brief Mask of the wake-up sources, OR of the items of \ref pwr_WakeUpSourceBit_t */
typedef uint32_t pwr_WakeUpSource_t;


/**
 * \brief SRAM (or the RAM of a peripheral) whose content is kept in Stop mode, only the blocks of
 *        the device line are in the list.
 *
 * The backup SRAM is kept in Standby and VBAT modes by the backup regulator.
 */
typedef enum
{
    PWR_SRAM_1 = 0u,           /**< AHB SRAM1                                                     */
#if defined(PWR_LINE_SRAM2_SINGLE)
    PWR_SRAM_2,                /**< AHB SRAM2 (STM32H503)                                         */
#endif
#if defined(PWR_LINE_SRAM2_48)
    PWR_SRAM_2_48,             /**< AHB SRAM2 48-Kbyte                                            */
#endif
#if defined(PWR_LINE_SRAM2_48_SPLIT)
    PWR_SRAM_2_48_LOW,         /**< AHB SRAM2 48-Kbyte low part                                   */
    PWR_SRAM_2_48_HIGH,        /**< AHB SRAM2 48-Kbyte high part                                  */
#endif
#if defined(PWR_LINE_SRAM2_16)
    PWR_SRAM_2_16,             /**< AHB SRAM2 16-Kbyte                                            */
#endif
#if defined(PWR_LINE_SRAM2_16_SPLIT)
    PWR_SRAM_2_16_LOW,         /**< AHB SRAM2 16-Kbyte low part                                   */
    PWR_SRAM_2_16_HIGH,        /**< AHB SRAM2 16-Kbyte high part                                  */
#endif
#if defined(PWR_LINE_HAS_SRAM3)
    PWR_SRAM_3,                /**< AHB SRAM3                                                     */
#endif
#if defined(PWR_LINE_HAS_SRAM4_5)
    PWR_SRAM_4,                /**< AHB SRAM4                                                     */
    PWR_SRAM_5,                /**< AHB SRAM5                                                     */
#endif
#if defined(PWR_LINE_HAS_ETH_RAM)
    PWR_SRAM_ETHERNET,         /**< Ethernet RAM                                                  */
#endif
#if defined(PWR_LINE_HAS_LTDC_RAM)
    PWR_SRAM_LTDC,             /**< LTDC FIFO RAM                                                 */
#endif
    PWR_SRAM_BACKUP,           /**< Backup SRAM, kept in Standby and VBAT modes (backup regulator) */
    PWR_SRAM_CNT               /**< Count of the retention blocks of the device                   */
}   pwr_SramId_t;


/** \brief Group of I/Os whose output state is retained in Standby mode */
typedef enum
{
    PWR_IO_GROUP_GENERAL = 0u, /**< All I/Os except the ones with the Standby function and the JTAG I/Os */
    PWR_IO_GROUP_JTAG,         /**< JTAG I/Os: PA13, PA14, PA15 and PB4                                  */
    PWR_IO_GROUP_CNT           /**< Count of the I/O groups                                              */
}   pwr_IoGroup_t;


/** \brief Threshold of the programmable voltage detector (PVD), the typical values of the datasheet */
typedef enum
{
    PWR_PVD_LEVEL_0 = 0u, /**< Level 0, about 1.95 V                            */
    PWR_PVD_LEVEL_1,      /**< Level 1, about 2.10 V                            */
    PWR_PVD_LEVEL_2,      /**< Level 2, about 2.25 V                            */
    PWR_PVD_LEVEL_3,      /**< Level 3, about 2.40 V                            */
    PWR_PVD_LEVEL_4,      /**< Level 4, about 2.55 V                            */
    PWR_PVD_LEVEL_5,      /**< Level 5, about 2.70 V                            */
    PWR_PVD_LEVEL_6,      /**< Level 6, about 2.85 V                            */
    PWR_PVD_LEVEL_PIN,    /**< The voltage of the PVD_IN pin (PB15) is compared */
    PWR_PVD_LEVEL_CNT     /**< Count of the PVD thresholds                      */
}   pwr_PvdLevel_t;


/** \brief Threshold of the analog voltage detector (AVD), the typical values of the datasheet */
typedef enum
{
    PWR_AVD_LEVEL_0 = 0u, /**< Level 0, about 1.7 V */
    PWR_AVD_LEVEL_1,      /**< Level 1, about 2.1 V */
    PWR_AVD_LEVEL_2,      /**< Level 2, about 2.5 V */
    PWR_AVD_LEVEL_3,      /**< Level 3, about 2.8 V */
    PWR_AVD_LEVEL_CNT     /**< Count of the AVD thresholds */
}   pwr_AvdLevel_t;


/** \brief Change of the supervised supply that is reported to the callback */
typedef enum
{
    PWR_SUPPLY_EVENT_DROP = 0u, /**< The supply drops below the threshold   */
    PWR_SUPPLY_EVENT_RISE,      /**< The supply rises above the threshold   */
    PWR_SUPPLY_EVENT_BOTH       /**< Both changes are reported              */
}   pwr_SupplyEvent_t;


/** \brief Configuration of the programmable voltage detector */
typedef struct
{
    pwr_PvdLevel_t    Level; /**< Threshold of the detector                       */
    pwr_SupplyEvent_t Event; /**< Change of the supply that calls the callback    */
}   pwr_PvdConfig_t;


/** \brief Configuration of the analog voltage detector */
typedef struct
{
    pwr_AvdLevel_t    Level; /**< Threshold of the detector                       */
    pwr_SupplyEvent_t Event; /**< Change of the supply that calls the callback    */
}   pwr_AvdConfig_t;


/** \brief Supply or monitored value of the device */
typedef enum
{
    PWR_SUPPLY_VDD = 0u,      /**< VDD, monitored by the PVD (the PVD has to be active)                  */
    PWR_SUPPLY_ANALOG,        /**< VDDA, monitored by the AVD (the AVD has to be active)                 */
#if defined(PWR_LINE_HAS_USB_SUPPLY)
    PWR_SUPPLY_USB,           /**< VDDUSB, monitor enabled by \ref Pwr_Set_UsbSupplyActive               */
#endif
    PWR_SUPPLY_IO2,           /**< VDDIO2, monitored always                                              */
    PWR_SUPPLY_BACKUP,        /**< VBAT voltage inside the thresholds (\ref Pwr_Set_BackupMonitor)       */
    PWR_SUPPLY_TEMPERATURE,   /**< Junction temperature inside the thresholds (\ref Pwr_Set_BackupMonitor) */
    PWR_SUPPLY_CNT            /**< Count of the supplies of the device                                   */
}   pwr_Supply_t;


/** \brief State of a supply */
typedef enum
{
    PWR_SUPPLY_STATE_NOT_VALID = 0u, /**< The supply is below the threshold or outside the range */
    PWR_SUPPLY_STATE_VALID           /**< The supply is above the threshold and inside the range  */
}   pwr_SupplyState_t;


/** \brief Charging of the VBAT battery through a resistor */
typedef enum
{
    PWR_VBAT_CHARGING_OFF = 0u, /**< Charging disabled        */
    PWR_VBAT_CHARGING_5K,       /**< Charging through 5 kOhm   */
    PWR_VBAT_CHARGING_1K5       /**< Charging through 1.5 kOhm */
}   pwr_VbatCharging_t;


/** \brief Hook called before the entry to and after the exit from a low-power mode */
typedef void ( pwr_HookCallback_t )( void );


/**
 * \brief Supply supervisor callback, called from the PVD / AVD interrupt (or from \ref Pwr_Task)
 *        for every supply whose state changed and whose configured event matches the change.
 *
 * Called in the interrupt context, the flags are already cleared by the module.
 */
typedef void ( pwr_SupplyCallback_t )( pwr_Supply_t supply, pwr_SupplyState_t supplyState );


/** \brief Configuration of the module (\ref Pwr_Init) */
typedef struct
{
    pwr_VoltageScale_t    VoltageScale;                         /**< Run voltage scale applied by the init   */
    pwr_RegulatorMode_t   RegulatorMode;                        /**< Supply of VCORE                         */
    pwr_FunctionState_t   StopFlashLowPower;                    /**< Flash in low-power mode in Stop mode    */
    pwr_FunctionState_t   BackupMonitor;                        /**< VBAT and temperature monitoring         */
    pwr_FunctionState_t   WakeUpPinActive[ PWR_WAKEUP_PIN_CNT ];/**< Armed wake-up pins, indexed by the pin  */
    pwr_WakeUpPinConfig_t WakeUpPin[ PWR_WAKEUP_PIN_CNT ];      /**< Configuration of the wake-up pins       */
    pwr_FunctionState_t   PvdActive;                            /**< PVD enabled by the init                 */
    pwr_PvdConfig_t       Pvd;                                  /**< Configuration of the PVD                */
    pwr_FunctionState_t   AvdActive;                            /**< AVD enabled by the init                 */
    pwr_AvdConfig_t       Avd;                                  /**< Configuration of the AVD                */
    pwr_HookCallback_t   *PreEnterCallback;                     /**< Called before a low-power mode entry    */
    pwr_HookCallback_t   *PostExitCallback;                     /**< Called after a low-power mode exit      */
    pwr_SupplyCallback_t *SupplyCallback;                       /**< PVD / AVD callback, \ref PWR_NULL_PTR
                                                                     disables the supervisor events        */
    pwr_FunctionState_t   SupplyIsr;                            /**< Supervisor events from the interrupt
                                                                     (active) or from \ref Pwr_Task        */
    pwr_IrqPrio_t         IrqPriority;                          /**< PVD / AVD interrupt priority (0 - highest) */
}   pwr_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* PWR_PWR_TYPES_H */
