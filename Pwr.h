/**
 * \author Mr.Nobody
 * \file Pwr.h
 * \ingroup Pwr
 * \brief Pwr module private header file.
 *
 * Holds the module state shared between Pwr.c and the other files of the module. Not
 * included by any other module.
 *
 */

#ifndef PWR_PWR_H
#define PWR_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Pwr_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/** \brief Count of the users that hold the write access to the backup domain */
typedef uint32_t pwr_BackupCnt_t;


/** \brief Module runtime state, kept private to the module. */
typedef struct
{
    pwr_FunctionState_t   Initialized;                           /**< Active once \ref Pwr_Init succeeded      */
    pwr_HookCallback_t   *PreEnterCallback;                      /**< Hook before a low-power mode entry       */
    pwr_HookCallback_t   *PostExitCallback;                      /**< Hook after a low-power mode exit         */
    pwr_SupplyCallback_t *SupplyCallback;                        /**< PVD / AVD callback                       */
    pwr_FunctionState_t   SupplyIsr;                             /**< Supervisor events from the interrupt     */
    pwr_FunctionState_t   LineActive;                            /**< PVD / AVD EXTI line and interrupt enabled */
    pwr_FunctionState_t   PvdActive;                             /**< PVD enabled                              */
    pwr_FunctionState_t   AvdActive;                             /**< AVD enabled                              */
    pwr_SupplyEvent_t     PvdEvent;                              /**< Change of VDD reported to the callback   */
    pwr_SupplyEvent_t     AvdEvent;                              /**< Change of VDDA reported to the callback  */
    pwr_SupplyState_t     PvdState;                              /**< Last state of VDD seen by the supervisor */
    pwr_SupplyState_t     AvdState;                              /**< Last state of VDDA seen by the supervisor */
    pwr_FunctionState_t   WakeUpPinActive[ PWR_WAKEUP_PIN_CNT ]; /**< Wake-up pins armed by the module         */
    pwr_BackupCnt_t       BackupAccessCnt;                       /**< Users of the backup domain write access  */
    pwr_FunctionState_t   BackupAccessOwned;                     /**< DBP was set by this module               */
}   pwr_State_t;

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS ================================ */

/* ========================== EXPORTED VARIABLES ============================= */

/* ========================= EXPORTED FUNCTIONS =============================== */

#ifdef __cplusplus
}
#endif

#endif /* PWR_PWR_H */
