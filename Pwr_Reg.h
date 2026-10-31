/**
 * \author Mr.Nobody
 * \file Pwr_Reg.h
 * \ingroup Pwr
 * \brief Pwr module Reg component header file.
 *
 * This component contains functionality for register operations. It is private
 * to the module (the register access is not shared with other modules).
 *
 */

#ifndef PWR_REG_PWR_REG_H
#define PWR_REG_PWR_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Pwr_Types.h"                      /* Module types definition        */
#include "Gpio_Types.h"                     /* gpio_PortId_t / gpio_PinId_t   */
/* ============================= TYPEDEFS =================================== */

/** \brief List of PWR registers accessed by the module */
typedef enum
{
    PWR_REG_PMCR   = 0u, /**< PWR power mode control register           */
    PWR_REG_PMSR       , /**< PWR status register                       */
    PWR_REG_VOSCR      , /**< PWR voltage scaling control register      */
    PWR_REG_VOSSR      , /**< PWR voltage scaling status register       */
    PWR_REG_BDCR       , /**< PWR backup domain control register        */
    PWR_REG_DBPCR      , /**< PWR disable backup protection register    */
    PWR_REG_BDSR       , /**< PWR backup domain status register         */
    PWR_REG_SCCR       , /**< PWR supply configuration control register */
    PWR_REG_VMCR       , /**< PWR voltage monitor control register      */
#if defined(PWR_LINE_HAS_USB_SUPPLY)
    PWR_REG_USBSCR     , /**< PWR USB supply control register           */
#endif
    PWR_REG_VMSR       , /**< PWR voltage monitor status register       */
    PWR_REG_WUSCR      , /**< PWR wake-up status clear register         */
    PWR_REG_WUSR       , /**< PWR wake-up status register               */
    PWR_REG_WUCR       , /**< PWR wake-up configuration register        */
    PWR_REG_IORETR     , /**< PWR I/O retention register                */
    PWR_REG_CNT          /**< Count of available PWR registers          */
}   pwr_RegId_t;


/** \brief One row of the wake-up pin table: the wake-up input and its fixed pin */
typedef struct
{
    pwr_WakeUpPin_t PinItem;   /**< The wake-up pin item this row describes     */
    uint8_t         WkupIndex; /**< Index of the input in the WUxx register bit fields (WKUP1 is 0) */
    gpio_PortId_t   GpioPort;  /**< GPIO port of the pin                        */
    gpio_PinId_t    GpioPin;   /**< GPIO pin number                             */
}   pwr_WakeUpPinInfo_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                        Pwr_Set_RegBit              ( pwr_RegId_t regId, uint32_t bitMask );
void                        Pwr_Reset_RegBit            ( pwr_RegId_t regId, uint32_t bitMask );
uint32_t                    Pwr_Get_RegBit              ( pwr_RegId_t regId, uint32_t bitMask );

void                        Pwr_Set_RegVal              ( pwr_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t                    Pwr_Get_RegVal              ( pwr_RegId_t regId, uint32_t regMask );

/* ----------------------------- Voltage scaling ----------------------------- */

pwr_RequestState_t          Pwr_Reg_Get_VoltageScale    ( pwr_VoltageScale_t * const voltageScale );
pwr_RequestState_t          Pwr_Reg_Set_VoltageScale    ( pwr_VoltageScale_t voltageScale );
pwr_RequestState_t          Pwr_Reg_Get_RegulatorMode  ( pwr_RegulatorMode_t * const regulatorMode );
pwr_RequestState_t          Pwr_Reg_Set_Bypass          ( void );

/* ----------------------------- Low-power modes ----------------------------- */

pwr_RequestState_t          Pwr_Reg_Set_StopFlashLowPower ( pwr_FunctionState_t flashLowPower );
pwr_FunctionState_t         Pwr_Reg_Get_StopFlashLowPower ( void );
pwr_RequestState_t          Pwr_Reg_Set_StopMode        ( pwr_StopMode_t stopMode );
pwr_RequestState_t          Pwr_Reg_Set_StandbyMode     ( void );
void                        Pwr_Reg_Set_StopDefault     ( void );
void                        Pwr_Reg_Set_ModeFlagClear   ( void );

/* ----------------------------- Wake-up ----------------------------------- */

pwr_RequestState_t          Pwr_Reg_Get_WakeUpPinInfo   ( pwr_WakeUpPin_t wakeUpPin,
                                                          const pwr_WakeUpPinInfo_t * * const info );
pwr_RequestState_t          Pwr_Reg_Set_WakeUpPin       ( const pwr_WakeUpPinInfo_t * const info,
                                                          const pwr_WakeUpPinConfig_t * const pinConfig );
pwr_RequestState_t          Pwr_Reg_Set_WakeUpPinInactive ( const pwr_WakeUpPinInfo_t * const info );
pwr_RequestState_t          Pwr_Reg_Get_WakeUpPinConfig ( const pwr_WakeUpPinInfo_t * const info,
                                                          pwr_FunctionState_t * const pinActive,
                                                          pwr_WakeUpPinConfig_t * const pinConfig );
pwr_WakeUpSource_t          Pwr_Reg_Get_WakeUpSource    ( void );
void                        Pwr_Reg_Set_WakeUpFlagClear ( void );

/* ----------------------------- Retention ----------------------------------- */

pwr_RequestState_t          Pwr_Reg_Set_SramRetention   ( pwr_SramId_t sramId, pwr_FunctionState_t retention );
pwr_RequestState_t          Pwr_Reg_Get_SramRetention   ( pwr_SramId_t sramId, pwr_FunctionState_t * const retention );
pwr_RequestState_t          Pwr_Reg_Set_IoRetention     ( pwr_IoGroup_t ioGroup, pwr_FunctionState_t retention );

/* ----------------------------- Supervision --------------------------------- */

pwr_RequestState_t          Pwr_Reg_Set_PvdLevel        ( pwr_PvdLevel_t pvdLevel );
pwr_RequestState_t          Pwr_Reg_Get_PvdLevel        ( pwr_PvdLevel_t * const pvdLevel );
pwr_RequestState_t          Pwr_Reg_Set_AvdLevel        ( pwr_AvdLevel_t avdLevel );
pwr_RequestState_t          Pwr_Reg_Get_AvdLevel        ( pwr_AvdLevel_t * const avdLevel );
pwr_RequestState_t          Pwr_Reg_Set_PvdState        ( pwr_FunctionState_t pvdState );
pwr_RequestState_t          Pwr_Reg_Set_AvdState        ( pwr_FunctionState_t avdState );
pwr_FunctionState_t         Pwr_Reg_Get_PvdState        ( void );
pwr_FunctionState_t         Pwr_Reg_Get_AvdState        ( void );
pwr_RequestState_t          Pwr_Reg_Set_BackupMonitor   ( pwr_FunctionState_t monitorState );
pwr_FunctionState_t         Pwr_Reg_Get_BackupMonitor   ( void );
pwr_RequestState_t          Pwr_Reg_Get_SupplyState     ( pwr_Supply_t supply, pwr_SupplyState_t * const supplyState );

/* ----------------------------- Backup domain and supplies ------------------ */

pwr_RequestState_t          Pwr_Reg_Set_BackupAccess    ( pwr_FunctionState_t accessState );
pwr_FunctionState_t         Pwr_Reg_Get_BackupAccess    ( void );
pwr_RequestState_t          Pwr_Reg_Set_VbatCharging    ( pwr_VbatCharging_t vbatCharging );
pwr_RequestState_t          Pwr_Reg_Set_UsbSupply       ( pwr_FunctionState_t supplyState );

#ifdef __cplusplus
}
#endif

#endif /* PWR_REG_PWR_REG_H */
