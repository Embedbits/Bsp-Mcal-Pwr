/**
 * \author Mr.Nobody
 * \file Pwr_Port.h
 * \ingroup Pwr
 * \brief Pwr module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not be used outside of the module.
 *
 */

#ifndef PWR_PWR_PORT_H
#define PWR_PWR_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Pwr_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

pwr_ModuleVersion_t         Pwr_Get_ModuleVersion        ( void );

pwr_RequestState_t          Pwr_Init                     ( const pwr_Config_t * const pwrConfig );
pwr_RequestState_t          Pwr_Deinit                   ( void );
void                        Pwr_Task                     ( void );
pwr_RequestState_t          Pwr_Get_DefaultConfig        ( pwr_Config_t * const pwrConfig );

/* ----------------------------- Voltage scaling ----------------------------- */

pwr_RequestState_t          Pwr_Set_VoltageScale         ( pwr_VoltageScale_t voltageScale );
pwr_RequestState_t          Pwr_Get_VoltageScale         ( pwr_VoltageScale_t * const voltageScale );
pwr_RequestState_t          Pwr_Set_RegulatorMode        ( pwr_RegulatorMode_t regulatorMode );
pwr_RequestState_t          Pwr_Get_RegulatorMode        ( pwr_RegulatorMode_t * const regulatorMode );

/* ----------------------------- Low-power modes ----------------------------- */

pwr_RequestState_t          Pwr_Set_SleepEnter           ( pwr_WakeUpEvent_t wakeUpEvent );
pwr_RequestState_t          Pwr_Set_StopEnter            ( pwr_StopMode_t stopMode );
pwr_RequestState_t          Pwr_Set_StandbyEnter         ( void );
pwr_RequestState_t          Pwr_Set_ShutdownEnter        ( void );

/* ----------------------------- Wake-up --------------------------------------- */

pwr_RequestState_t          Pwr_Set_WakeUpPin            ( pwr_WakeUpPin_t wakeUpPin,
                                                           const pwr_WakeUpPinConfig_t * const pinConfig );
pwr_RequestState_t          Pwr_Set_WakeUpPinInactive    ( pwr_WakeUpPin_t wakeUpPin );
pwr_RequestState_t          Pwr_Get_WakeUpSource         ( pwr_WakeUpSource_t * const wakeUpSource );
pwr_RequestState_t          Pwr_Get_StandbyFlag          ( pwr_FunctionState_t * const standbyFlag );
pwr_RequestState_t          Pwr_Set_WakeUpFlagClear      ( void );

/* ----------------------------- Retention ------------------------------------- */

pwr_RequestState_t          Pwr_Set_SramRetention        ( pwr_SramId_t sramId, pwr_FunctionState_t retention );
pwr_RequestState_t          Pwr_Get_SramRetention        ( pwr_SramId_t sramId, pwr_FunctionState_t * const retention );
pwr_RequestState_t          Pwr_Set_IoRetention          ( pwr_IoGroup_t ioGroup, pwr_FunctionState_t retention );

/* ----------------------------- Supervision ----------------------------------- */

pwr_RequestState_t          Pwr_Set_PvdConfig            ( const pwr_PvdConfig_t * const pvdConfig );
pwr_RequestState_t          Pwr_Set_PvdActive            ( void );
pwr_RequestState_t          Pwr_Set_PvdInactive          ( void );
pwr_RequestState_t          Pwr_Set_AvdConfig            ( const pwr_AvdConfig_t * const avdConfig );
pwr_RequestState_t          Pwr_Set_AvdActive            ( void );
pwr_RequestState_t          Pwr_Set_AvdInactive          ( void );
pwr_RequestState_t          Pwr_Set_BackupMonitor        ( pwr_FunctionState_t monitorState );
pwr_RequestState_t          Pwr_Get_SupplyState          ( pwr_Supply_t supply, pwr_SupplyState_t * const supplyState );

/* ----------------------------- Backup domain and supplies -------------------- */

pwr_RequestState_t          Pwr_Set_BackupAccessActive   ( void );
pwr_RequestState_t          Pwr_Set_BackupAccessInactive ( void );
pwr_RequestState_t          Pwr_Set_VbatCharging         ( pwr_VbatCharging_t vbatCharging );
pwr_RequestState_t          Pwr_Set_UsbSupplyActive      ( void );
pwr_RequestState_t          Pwr_Set_UsbSupplyInactive    ( void );
pwr_RequestState_t          Pwr_Set_Io2SupplyActive      ( void );

/* ----------------------------- Interrupts ------------------------------------ */

pwr_RequestState_t          Pwr_Set_IrqPriority          ( pwr_IrqPrio_t irqPriority );
pwr_RequestState_t          Pwr_Get_IrqPriority          ( pwr_IrqPrio_t * const irqPriority );

#ifdef __cplusplus
}
#endif

#endif /* PWR_PWR_PORT_H */
