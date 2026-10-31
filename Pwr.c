/**
 * \author Mr.Nobody
 * \file Pwr.c
 * \ingroup Pwr
 * \brief Pwr module main functionality (STM32H5).
 *
 * Implements \ref Pwr_Port.h against the PWR registers (via \ref Pwr_Reg.h), the core deep sleep
 * and wait instructions (Scb), the pins (Gpio), the PVD / AVD interrupt (Exti line and Nvic) and
 * the callbacks of the application.
 *
 * Design decisions of this implementation:
 * - The VOS and DBP bits and the VDDUSB supply bit are written also by the Rcc module (through the
 *   LL driver, until it is migrated to this module). \ref Pwr_Get_DefaultConfig returns the current
 *   hardware state and \ref Pwr_Init does not write the voltage scale when it is already applied.
 * - The write access to the backup domain (PWR_DBPCR.DBP) is reference counted. The module clears
 *   DBP only when it was the one that set it, a DBP set by another module (Rcc) is left set.
 * - The voltage scale is changed one scale at a time with a bounded wait for every step. The scale
 *   is reset to scale 3 by the hardware when the MCU leaves the Stop mode, \ref Pwr_Set_StopEnter
 *   restores the Run scale it found at the entry before the post exit callback is called.
 * - Stop and Standby use the deep sleep selection and the wait instructions of the Scb component.
 *   The PWR has no clock enable bit, the module does not link the Rcc module.
 * - The PVD and the AVD share the EXTI line 16 and one interrupt. The EXTI line is configured for
 *   both edges, the handler compares the PVDO / AVDO flags with the last seen state and reports the
 *   changes that match the configured event to the callback.
 * - The functions that only read or clear the wake-up and Standby flags and the backup access
 *   functions work without \ref Pwr_Init, they are used right after a restart and by the modules that
 *   do not know whether the application initialized this module.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Pwr.h"                            /* Module private definitions     */
#include "Pwr_Port.h"                       /* Module public functionality    */
#include "Pwr_Reg.h"                        /* Module register access         */
#include "Gpio_Port.h"                      /* Wake-up and PVD_IN pins        */
#include "Nvic_Port.h"                      /* PVD / AVD interrupt            */
#include "Exti_Port.h"                      /* PVD / AVD wake-up line         */
#include "Scb_Port.h"                       /* Deep sleep and wait for wake-up */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** \brief Module version */
#define PWR_MODULE_VERSION_MAJOR    ( 1u )
#define PWR_MODULE_VERSION_MINOR    ( 0u )
#define PWR_MODULE_VERSION_PATCH    ( 0u )

/** \brief Highest count of the users of the backup domain write access */
#define PWR_BACKUP_ACCESS_CNT_MAX   ( 0xFFFFFFFFu )

/** \brief Count of the function states of the configuration that are checked as a list */
#define PWR_CONFIG_STATE_CNT        ( 5u )

/* ======================== FORWARD DECLARATIONS ============================ */

static pwr_RequestState_t   Pwr_Check_Config            ( const pwr_Config_t * const pwrConfig );
static pwr_RequestState_t   Pwr_Check_FunctionState     ( pwr_FunctionState_t functionState );
static pwr_RequestState_t   Pwr_Check_Event             ( pwr_SupplyEvent_t supplyEvent );
static pwr_RequestState_t   Pwr_Check_WakeUpPinConfig   ( const pwr_WakeUpPinConfig_t * const pinConfig );
static pwr_RequestState_t   Pwr_Apply_Config            ( const pwr_Config_t * const pwrConfig );
static pwr_RequestState_t   Pwr_Apply_Regulator         ( pwr_RegulatorMode_t regulatorMode );
static pwr_RequestState_t   Pwr_Apply_BackupMonitor     ( pwr_FunctionState_t monitorState );
static pwr_RequestState_t   Pwr_Apply_BackupRam         ( pwr_FunctionState_t retention );
static pwr_RequestState_t   Pwr_Apply_WakeUpPin         ( pwr_WakeUpPin_t wakeUpPin,
                                                          const pwr_WakeUpPinConfig_t * const pinConfig );
static pwr_RequestState_t   Pwr_Apply_WakeUpPinInactive ( pwr_WakeUpPin_t wakeUpPin );
static pwr_RequestState_t   Pwr_Apply_PvdConfig         ( const pwr_PvdConfig_t * const pvdConfig );
static gpio_RequestState_t  Pwr_Set_AnalogPin           ( gpio_PortId_t gpioPort, gpio_PinId_t gpioPin );
static pwr_RequestState_t   Pwr_Apply_AvdConfig         ( const pwr_AvdConfig_t * const avdConfig );
static pwr_RequestState_t   Pwr_Apply_PvdState          ( pwr_FunctionState_t pvdState );
static pwr_RequestState_t   Pwr_Apply_AvdState          ( pwr_FunctionState_t avdState );
static pwr_RequestState_t   Pwr_Set_IsrConfig           ( const pwr_Config_t * const pwrConfig );
static pwr_RequestState_t   Pwr_Set_SupervisorLine      ( void );
static pwr_SupplyState_t    Pwr_Get_SupervisorState     ( pwr_Supply_t supply );
static pwr_FunctionState_t  Pwr_Get_EventReported       ( pwr_SupplyEvent_t supplyEvent,
                                                          pwr_SupplyState_t supplyState );
static void                 Pwr_Call_Hook               ( pwr_HookCallback_t * const hookCallback );
static void                 Pwr_Process_Supply          ( pwr_Supply_t supply );
static void                 Pwr_IsrHandler              ( void );

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

static pwr_State_t pwr_State =
{
    .Initialized       = PWR_FUNCTION_INACTIVE,
    .PreEnterCallback  = PWR_NULL_PTR,
    .PostExitCallback  = PWR_NULL_PTR,
    .SupplyCallback    = PWR_NULL_PTR,
    .SupplyIsr         = PWR_FUNCTION_INACTIVE,
    .LineActive        = PWR_FUNCTION_INACTIVE,
    .PvdActive         = PWR_FUNCTION_INACTIVE,
    .AvdActive         = PWR_FUNCTION_INACTIVE,
    .PvdEvent          = PWR_SUPPLY_EVENT_DROP,
    .AvdEvent          = PWR_SUPPLY_EVENT_DROP,
    .PvdState          = PWR_SUPPLY_STATE_VALID,
    .AvdState          = PWR_SUPPLY_STATE_VALID,
    .WakeUpPinActive   = { PWR_FUNCTION_INACTIVE },
    .BackupAccessCnt   = 0u,
    .BackupAccessOwned = PWR_FUNCTION_INACTIVE,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns the module version.
 * \return The module version.
 */
pwr_ModuleVersion_t Pwr_Get_ModuleVersion( void )
{
    pwr_ModuleVersion_t version;

    version.Major = (pwr_MajorVersion_t)PWR_MODULE_VERSION_MAJOR;
    version.Minor = (pwr_MinorVersion_t)PWR_MODULE_VERSION_MINOR;
    version.Patch = (pwr_PatchVersion_t)PWR_MODULE_VERSION_PATCH;

    return ( version );
}


/**
 * \brief Initializes the module with the given configuration.
 *
 * The regulator, the voltage scale (written only when it differs from the applied one), the flash
 * low-power selection of Stop mode, the backup monitor, the wake-up pins, the PVD and the AVD and the
 * callbacks are applied. Every register write is verified by read-back, when one of the steps fails
 * the module is deinitialized and an error is returned. The state of the module is not changed when
 * the configuration is invalid.
 *
 * \param pwrConfig [in]: Configuration to apply (start from \ref Pwr_Get_DefaultConfig)
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the configuration is invalid or
 *         a step failed
 */
pwr_RequestState_t Pwr_Init( const pwr_Config_t * const pwrConfig )
{
    pwr_RequestState_t retState   = PWR_REQUEST_ERROR;
    pwr_RequestState_t checkState = Pwr_Check_Config( pwrConfig );

    if( PWR_REQUEST_OK == checkState )
    {
        const pwr_RequestState_t applyState = Pwr_Apply_Config( pwrConfig );

        if( PWR_REQUEST_OK == applyState )
        {
            pwr_State.Initialized = PWR_FUNCTION_ACTIVE;
            retState              = PWR_REQUEST_OK;
        }
        else
        {
            /* A step failed, the configuration applied so far is withdrawn */
            (void)Pwr_Deinit();
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid configuration, the module stays as it was */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes the module: disables the PVD and the AVD, their interrupt and the wake-up pins
 *        armed by the module and clears the callbacks.
 *
 * The voltage scale, the retention selections and the backup access count are not changed.
 *
 * \return \ref PWR_REQUEST_OK when everything was disabled, \ref PWR_REQUEST_ERROR when a register did not
 *         take the value (the state of the module is cleared in any case)
 */
pwr_RequestState_t Pwr_Deinit( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_OK;

    pwr_State.Initialized      = PWR_FUNCTION_INACTIVE;
    pwr_State.SupplyCallback   = PWR_NULL_PTR;
    pwr_State.PreEnterCallback = PWR_NULL_PTR;
    pwr_State.PostExitCallback = PWR_NULL_PTR;
    pwr_State.SupplyIsr        = PWR_FUNCTION_INACTIVE;

    if( PWR_FUNCTION_ACTIVE == pwr_State.PvdActive )
    {
        const pwr_RequestState_t pvdState = Pwr_Apply_PvdState( PWR_FUNCTION_INACTIVE );

        if( PWR_REQUEST_OK != pvdState )
        {
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The PVD is disabled */
        }
    }
    else
    {
        /* The PVD was not enabled by the module */
    }

    if( PWR_FUNCTION_ACTIVE == pwr_State.AvdActive )
    {
        const pwr_RequestState_t avdState = Pwr_Apply_AvdState( PWR_FUNCTION_INACTIVE );

        if( PWR_REQUEST_OK != avdState )
        {
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The AVD is disabled */
        }
    }
    else
    {
        /* The AVD was not enabled by the module */
    }

    for( uint32_t pinIdx = 0u; (uint32_t)PWR_WAKEUP_PIN_UNUSED > pinIdx; pinIdx++ )
    {
        if( PWR_FUNCTION_ACTIVE == pwr_State.WakeUpPinActive[pinIdx] )
        {
            const pwr_RequestState_t pinState = Pwr_Apply_WakeUpPinInactive( (pwr_WakeUpPin_t)pinIdx );

            if( PWR_REQUEST_OK != pinState )
            {
                retState = PWR_REQUEST_ERROR;
            }
            else
            {
                /* The wake-up pin is disarmed */
            }
        }
        else
        {
            /* The pin was not armed by the module */
        }
    }

    return ( retState );
}


/**
 * \brief Polled service task of the supply supervisors.
 *
 * When the callback is registered and the supervisor events are not taken from the interrupt, the
 * PVD and AVD flags are compared with the last seen state and the changes that match the
 * configured event are reported to the callback. Nothing is done in the interrupt mode.
 */
void Pwr_Task( void )
{
    if( ( PWR_FUNCTION_ACTIVE   == pwr_State.Initialized   ) &&
        ( PWR_FUNCTION_INACTIVE == pwr_State.SupplyIsr     ) &&
        ( PWR_NULL_PTR          != pwr_State.SupplyCallback )    )
    {
        Pwr_Process_Supply( PWR_SUPPLY_VDD );
        Pwr_Process_Supply( PWR_SUPPLY_ANALOG );
    }
    else
    {
        /* Module not initialized, interrupt mode or no callback registered, no action required */
    }
}


/**
 * \brief Fills a configuration with the current hardware state (no callback, no interrupt).
 *
 * \param pwrConfig [out]: Configuration filled with the current voltage scale, regulator, flash
 *                         low-power selection, backup monitor, wake-up pins and supervisors
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Get_DefaultConfig( pwr_Config_t * const pwrConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != pwrConfig )
    {
        pwrConfig->VoltageScale     = PWR_VOLTAGE_SCALE_3;
        pwrConfig->RegulatorMode    = PWR_REGULATOR_LDO;
        pwrConfig->Pvd.Level        = PWR_PVD_LEVEL_0;
        pwrConfig->Avd.Level        = PWR_AVD_LEVEL_0;

        /* The values the registers cannot tell stay at the defaults above */
        (void)Pwr_Reg_Get_VoltageScale( &pwrConfig->VoltageScale );
        (void)Pwr_Reg_Get_RegulatorMode( &pwrConfig->RegulatorMode );
        (void)Pwr_Reg_Get_PvdLevel( &pwrConfig->Pvd.Level );
        (void)Pwr_Reg_Get_AvdLevel( &pwrConfig->Avd.Level );

        pwrConfig->StopFlashLowPower = Pwr_Reg_Get_StopFlashLowPower();
        pwrConfig->BackupMonitor     = Pwr_Reg_Get_BackupMonitor();
        pwrConfig->PvdActive         = Pwr_Reg_Get_PvdState();
        pwrConfig->AvdActive         = Pwr_Reg_Get_AvdState();
        pwrConfig->Pvd.Event         = PWR_SUPPLY_EVENT_DROP;
        pwrConfig->Avd.Event         = PWR_SUPPLY_EVENT_DROP;

        for( uint32_t pinIdx = 0u; (uint32_t)PWR_WAKEUP_PIN_CNT > pinIdx; pinIdx++ )
        {
            const pwr_WakeUpPinInfo_t *info = PWR_NULL_PTR;

            pwrConfig->WakeUpPinActive[pinIdx]     = PWR_FUNCTION_INACTIVE;
            pwrConfig->WakeUpPin[pinIdx].Polarity  = PWR_WAKEUP_RISING;
            pwrConfig->WakeUpPin[pinIdx].Pull      = PWR_PULL_NONE;

            const pwr_RequestState_t infoState = Pwr_Reg_Get_WakeUpPinInfo( (pwr_WakeUpPin_t)pinIdx, &info );

            if( PWR_REQUEST_OK == infoState )
            {
                (void)Pwr_Reg_Get_WakeUpPinConfig( info,
                                                   &pwrConfig->WakeUpPinActive[pinIdx],
                                                   &pwrConfig->WakeUpPin[pinIdx] );
            }
            else
            {
                /* The item has no pin, it keeps the defaults */
            }
        }

        pwrConfig->PreEnterCallback = PWR_NULL_PTR;
        pwrConfig->PostExitCallback = PWR_NULL_PTR;
        pwrConfig->SupplyCallback   = PWR_NULL_PTR;
        pwrConfig->SupplyIsr        = PWR_FUNCTION_INACTIVE;
        pwrConfig->IrqPriority      = (pwr_IrqPrio_t)0u;

        retState = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid configuration pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Applies a new voltage scale of VCORE, one scale at a time.
 *
 * Call this function before the system clock is raised and after it is lowered, the module does not
 * check the clock. The function returns when VCORE has reached the scale, the wait is bounded.
 *
 * \param voltageScale [in]: Required voltage scale
 * \return \ref PWR_REQUEST_OK if the scale is applied, \ref PWR_REQUEST_ERROR when the module is not
 *         initialized, the scale is invalid or a step timed out (the scale reached so far stays applied)
 */
pwr_RequestState_t Pwr_Set_VoltageScale( pwr_VoltageScale_t voltageScale )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Set_VoltageScale( voltageScale );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the voltage scale that is applied to VCORE.
 *
 * \param voltageScale [out]: Applied voltage scale
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer or when the module
 *         is not initialized
 */
pwr_RequestState_t Pwr_Get_VoltageScale( pwr_VoltageScale_t * const voltageScale )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Get_VoltageScale( voltageScale );
    }
    else
    {
        /* Module not initialized, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the supply of VCORE.
 *
 * The LDO or SMPS regulator is selected by the hardware according to the package, so the request
 * for them is accepted only when it matches the package. The bypass (external VCORE supply) can be
 * written only once after the power-on reset.
 *
 * \param regulatorMode [in]: Required supply of VCORE
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         mode is invalid or not the one of the package, or the bypass is locked
 */
pwr_RequestState_t Pwr_Set_RegulatorMode( pwr_RegulatorMode_t regulatorMode )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_Regulator( regulatorMode );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the supply of VCORE.
 *
 * \param regulatorMode [out]: Supply of VCORE
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer or when the module
 *         is not initialized
 */
pwr_RequestState_t Pwr_Get_RegulatorMode( pwr_RegulatorMode_t * const regulatorMode )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Get_RegulatorMode( regulatorMode );
    }
    else
    {
        /* Module not initialized, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Puts the core to Sleep mode and returns after the wake-up.
 *
 * The pre enter callback is called before and the post exit callback after the sleep. The clocks run
 * in Sleep mode, the peripherals are not touched.
 *
 * \param wakeUpEvent [in]: Wait for interrupt (WFI) or wait for event (WFE)
 * \return \ref PWR_REQUEST_OK after the wake-up, \ref PWR_REQUEST_ERROR when the module is not
 *         initialized or the instruction is invalid (the core does not sleep then)
 */
pwr_RequestState_t Pwr_Set_SleepEnter( pwr_WakeUpEvent_t wakeUpEvent )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_WAKEUP_EVENT    >= wakeUpEvent           )    )
    {
        Pwr_Call_Hook( pwr_State.PreEnterCallback );

        Scb_Set_DeepSleepInactive();

        if( PWR_WAKEUP_EVENT == wakeUpEvent )
        {
            Scb_Set_WaitForEvent();
        }
        else
        {
            Scb_Set_WaitForInterrupt();
        }

        Pwr_Call_Hook( pwr_State.PostExitCallback );

        retState = PWR_REQUEST_OK;
    }
    else
    {
        /* Module not initialized or invalid instruction, the core does not sleep */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Puts the MCU to Stop mode and returns after the wake-up.
 *
 * The Stop sub-mode and the Stop selection of the deep sleep are written and verified before the pre
 * enter callback is called, so a failing register leaves the MCU running. After the wake-up the Run
 * voltage scale found at the entry is restored (the hardware resets it to scale 3) and the post exit
 * callback is called, the clocks are restored by the application (Rcc). The EXTI pending flags and
 * the peripheral flags that generate wake-up interrupts have to be cleared before the entry, otherwise
 * the hardware ignores the entry and the function returns at once.
 *
 * \param stopMode [in]: Stop sub-mode
 * \return \ref PWR_REQUEST_OK after the wake-up with the Run scale restored, \ref PWR_REQUEST_ERROR when the
 *         module is not initialized, the sub-mode is invalid, a register did not take its value (no entry)
 *         or the Run scale could not be restored
 */
pwr_RequestState_t Pwr_Set_StopEnter( pwr_StopMode_t stopMode )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_STOP_CNT        >  stopMode              )    )
    {
        pwr_VoltageScale_t runScale   = PWR_VOLTAGE_SCALE_CNT;
        pwr_RequestState_t scaleState = Pwr_Reg_Get_VoltageScale( &runScale );
        pwr_RequestState_t modeState  = PWR_REQUEST_ERROR;

        if( PWR_REQUEST_OK == scaleState )
        {
            modeState = Pwr_Reg_Set_StopMode( stopMode );
        }
        else
        {
            /* The Run scale is not known, the entry is refused */
            modeState = PWR_REQUEST_ERROR;
        }

        if( PWR_REQUEST_OK == modeState )
        {
            Pwr_Reg_Set_ModeFlagClear();
            Pwr_Call_Hook( pwr_State.PreEnterCallback );

            Scb_Set_DeepSleepActive();
            Scb_Set_WaitForInterrupt();
            Scb_Set_DeepSleepInactive();

            retState = Pwr_Reg_Set_VoltageScale( runScale );

            Pwr_Call_Hook( pwr_State.PostExitCallback );
        }
        else
        {
            /* A register did not take its value, the MCU does not enter Stop mode */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module not initialized or invalid sub-mode, the MCU does not enter Stop mode */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Puts the MCU to Standby mode, the MCU restarts after the wake-up and the function does not return.
 *
 * The Standby selection is written and verified, the Standby and Stop flags are cleared and the pre
 * enter callback is called before the deep sleep. A wake-up flag that is pending at the entry makes the
 * hardware ignore the entry: the function then returns an error after the post exit callback (clear the
 * flags with \ref Pwr_Set_WakeUpFlagClear before the entry).
 *
 * \return \ref PWR_REQUEST_ERROR always when it returns: the module is not initialized, the register did
 *         not take the value or the hardware refused the entry
 */
pwr_RequestState_t Pwr_Set_StandbyEnter( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        const pwr_RequestState_t modeState = Pwr_Reg_Set_StandbyMode();

        if( PWR_REQUEST_OK == modeState )
        {
            Pwr_Reg_Set_ModeFlagClear();
            Pwr_Call_Hook( pwr_State.PreEnterCallback );

            Scb_Set_DeepSleepActive();
            Scb_Set_WaitForInterrupt();

            /* The code runs on only when the entry was refused (a wake-up flag is pending) */
            Scb_Set_DeepSleepInactive();
            Pwr_Reg_Set_StopDefault();

            Pwr_Call_Hook( pwr_State.PostExitCallback );
        }
        else
        {
            /* The Standby selection did not take the value, the MCU does not enter Standby mode */
            Pwr_Reg_Set_StopDefault();
        }

        retState = PWR_REQUEST_ERROR;
    }
    else
    {
        /* Module not initialized, the MCU does not enter Standby mode */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Shutdown mode, the STM32H5 devices do not have it.
 *
 * \return \ref PWR_REQUEST_ERROR always
 */
pwr_RequestState_t Pwr_Set_ShutdownEnter( void )
{
    return ( PWR_REQUEST_ERROR );
}


/**
 * \brief Arms a wake-up pin for the Standby mode: the pin is set to analog mode without pull in the GPIO,
 *        the polarity and the pull resistor are written to the PWR and the pin is enabled.
 *
 * \param wakeUpPin [in]: Wake-up pin
 * \param pinConfig [in]: Polarity and pull of the pin
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         pin or the configuration is invalid, or a register did not take its value
 */
pwr_RequestState_t Pwr_Set_WakeUpPin( pwr_WakeUpPin_t wakeUpPin, const pwr_WakeUpPinConfig_t * const pinConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_WakeUpPin( wakeUpPin, pinConfig );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disarms a wake-up pin (the polarity and the pull stay written).
 *
 * \param wakeUpPin [in]: Wake-up pin
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         pin is invalid or the register did not take the value
 */
pwr_RequestState_t Pwr_Set_WakeUpPinInactive( pwr_WakeUpPin_t wakeUpPin )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_WakeUpPinInactive( wakeUpPin );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the wake-up flags of the pins and the Standby and Stop flags (no initialization needed).
 *
 * The flags of the RTC and TAMP events are read from the Rtc and Tamp modules.
 *
 * \param wakeUpSource [out]: Mask of the \ref pwr_WakeUpSourceBit_t items
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Get_WakeUpSource( pwr_WakeUpSource_t * const wakeUpSource )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != wakeUpSource )
    {
        *wakeUpSource = Pwr_Reg_Get_WakeUpSource();
        retState      = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads whether the system has been in Standby mode (no initialization needed).
 *
 * The flag is set by the hardware at the wake-up from Standby and cleared by \ref Pwr_Set_WakeUpFlagClear
 * or by a power-on reset.
 *
 * \param standbyFlag [out]: \ref PWR_FUNCTION_ACTIVE if the system has been in Standby mode
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR for a null pointer
 */
pwr_RequestState_t Pwr_Get_StandbyFlag( pwr_FunctionState_t * const standbyFlag )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != standbyFlag )
    {
        const pwr_WakeUpSource_t wakeUpSource = Pwr_Reg_Get_WakeUpSource();

        if( 0u != ( wakeUpSource & (pwr_WakeUpSource_t)PWR_WAKEUP_SOURCE_STANDBY ) )
        {
            *standbyFlag = PWR_FUNCTION_ACTIVE;
        }
        else
        {
            *standbyFlag = PWR_FUNCTION_INACTIVE;
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
 * \brief Clears the wake-up flags of the pins and the Standby and Stop flags (no initialization needed).
 *
 * \return \ref PWR_REQUEST_OK always
 */
pwr_RequestState_t Pwr_Set_WakeUpFlagClear( void )
{
    Pwr_Reg_Set_WakeUpFlagClear();

    return ( PWR_REQUEST_OK );
}


/**
 * \brief Selects whether the content of a retention block is kept in Stop mode.
 *
 * The backup SRAM is kept in Standby and VBAT modes by the backup regulator, the write access to the
 * backup domain is taken for the time of the write.
 *
 * \param sramId    [in]: Retention block
 * \param retention [in]: \ref PWR_FUNCTION_ACTIVE keeps the content
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         block or the state is invalid, or the register did not take the value
 */
pwr_RequestState_t Pwr_Set_SramRetention( pwr_SramId_t sramId, pwr_FunctionState_t retention )
{
    pwr_RequestState_t retState   = PWR_REQUEST_ERROR;
    pwr_RequestState_t stateCheck = Pwr_Check_FunctionState( retention );

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_SRAM_CNT        >  sramId                ) &&
        ( PWR_REQUEST_OK      == stateCheck            )    )
    {
        if( PWR_SRAM_BACKUP == sramId )
        {
            retState = Pwr_Apply_BackupRam( retention );
        }
        else
        {
            retState = Pwr_Reg_Set_SramRetention( sramId, retention );
        }
    }
    else
    {
        /* Module not initialized or invalid block or state, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads whether the content of a retention block is kept.
 *
 * \param sramId    [in]: Retention block
 * \param retention [out]: \ref PWR_FUNCTION_ACTIVE if the content is kept
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized or the
 *         block or the pointer is invalid
 */
pwr_RequestState_t Pwr_Get_SramRetention( pwr_SramId_t sramId, pwr_FunctionState_t * const retention )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Get_SramRetention( sramId, retention );
    }
    else
    {
        /* Module not initialized, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects whether the output state of a group of I/Os is retained in Standby mode.
 *
 * The retention stays set after the wake-up from Standby until it is cleared, clear it after the GPIO
 * modules were configured again.
 *
 * \param ioGroup   [in]: Group of I/Os
 * \param retention [in]: \ref PWR_FUNCTION_ACTIVE retains the output state
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         group or the state is invalid, or the register did not take the value
 */
pwr_RequestState_t Pwr_Set_IoRetention( pwr_IoGroup_t ioGroup, pwr_FunctionState_t retention )
{
    pwr_RequestState_t retState   = PWR_REQUEST_ERROR;
    pwr_RequestState_t stateCheck = Pwr_Check_FunctionState( retention );

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_REQUEST_OK      == stateCheck            )    )
    {
        retState = Pwr_Reg_Set_IoRetention( ioGroup, retention );
    }
    else
    {
        /* Module not initialized or invalid state, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the threshold and the reported event of the programmable voltage detector.
 *
 * The PVD_IN pin (PB15) is set to analog mode when it is the compared voltage.
 *
 * \param pvdConfig [in]: Threshold and event of the PVD
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         configuration is invalid or a register did not take the value (the PVD can be locked by SBS)
 */
pwr_RequestState_t Pwr_Set_PvdConfig( const pwr_PvdConfig_t * const pvdConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_PvdConfig( pvdConfig );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables the programmable voltage detector.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized or the
 *         register did not take the value (the PVD can be locked by SBS)
 */
pwr_RequestState_t Pwr_Set_PvdActive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_PvdState( PWR_FUNCTION_ACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the programmable voltage detector.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized or the
 *         register did not take the value
 */
pwr_RequestState_t Pwr_Set_PvdInactive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_PvdState( PWR_FUNCTION_INACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the threshold and the reported event of the analog voltage detector.
 *
 * \param avdConfig [in]: Threshold and event of the AVD
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         configuration is invalid or the register did not take the value
 */
pwr_RequestState_t Pwr_Set_AvdConfig( const pwr_AvdConfig_t * const avdConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_AvdConfig( avdConfig );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables the analog voltage detector.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized or the
 *         register did not take the value
 */
pwr_RequestState_t Pwr_Set_AvdActive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_AvdState( PWR_FUNCTION_ACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the analog voltage detector.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized or the
 *         register did not take the value
 */
pwr_RequestState_t Pwr_Set_AvdInactive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Apply_AvdState( PWR_FUNCTION_INACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the monitoring of the VBAT voltage and of the temperature.
 *
 * The write access to the backup domain is taken for the time of the write.
 *
 * \param monitorState [in]: \ref PWR_FUNCTION_ACTIVE enables the monitoring
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         state is invalid or the register did not take the value
 */
pwr_RequestState_t Pwr_Set_BackupMonitor( pwr_FunctionState_t monitorState )
{
    pwr_RequestState_t retState   = PWR_REQUEST_ERROR;
    pwr_RequestState_t stateCheck = Pwr_Check_FunctionState( monitorState );

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_REQUEST_OK      == stateCheck            )    )
    {
        retState = Pwr_Apply_BackupMonitor( monitorState );
    }
    else
    {
        /* Module not initialized or invalid state, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the state of a supply.
 *
 * The monitor of the supply has to be enabled (PVD, AVD, USB supply, backup monitor), the VDDIO2
 * monitor works always.
 *
 * \param supply      [in]: Supply
 * \param supplyState [out]: State of the supply
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         supply or the pointer is invalid or the monitor of the supply is not enabled
 */
pwr_RequestState_t Pwr_Get_SupplyState( pwr_Supply_t supply, pwr_SupplyState_t * const supplyState )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Get_SupplyState( supply, supplyState );
    }
    else
    {
        /* Module not initialized, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Takes the write access to the backup domain (no initialization needed).
 *
 * The access is reference counted: the first holder sets PWR_DBPCR.DBP (when it is not set already by
 * another module), every holder has to release it with \ref Pwr_Set_BackupAccessInactive.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when DBP did not take the value or the
 *         count would overflow
 */
pwr_RequestState_t Pwr_Set_BackupAccessActive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_BACKUP_ACCESS_CNT_MAX > pwr_State.BackupAccessCnt )
    {
        if( 0u == pwr_State.BackupAccessCnt )
        {
            const pwr_FunctionState_t accessState = Pwr_Reg_Get_BackupAccess();

            if( PWR_FUNCTION_ACTIVE == accessState )
            {
                /* DBP is set by another module, it is left set when the access is released */
                pwr_State.BackupAccessOwned = PWR_FUNCTION_INACTIVE;
                retState                    = PWR_REQUEST_OK;
            }
            else
            {
                retState = Pwr_Reg_Set_BackupAccess( PWR_FUNCTION_ACTIVE );

                if( PWR_REQUEST_OK == retState )
                {
                    pwr_State.BackupAccessOwned = PWR_FUNCTION_ACTIVE;
                }
                else
                {
                    /* DBP did not take the value, the count is not changed */
                }
            }
        }
        else
        {
            /* The access is already taken by an earlier holder */
            retState = PWR_REQUEST_OK;
        }

        if( PWR_REQUEST_OK == retState )
        {
            pwr_State.BackupAccessCnt++;
        }
        else
        {
            /* The access was not taken */
        }
    }
    else
    {
        /* Count overflow, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases the write access to the backup domain (no initialization needed).
 *
 * The protection is enabled again when the last holder releases the access and DBP was set by this module.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the access was not taken (the
 *         release is not balanced) or DBP did not take the value
 */
pwr_RequestState_t Pwr_Set_BackupAccessInactive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( 0u < pwr_State.BackupAccessCnt )
    {
        pwr_State.BackupAccessCnt--;
        retState = PWR_REQUEST_OK;

        if( ( 0u                  == pwr_State.BackupAccessCnt   ) &&
            ( PWR_FUNCTION_ACTIVE == pwr_State.BackupAccessOwned )    )
        {
            retState                    = Pwr_Reg_Set_BackupAccess( PWR_FUNCTION_INACTIVE );
            pwr_State.BackupAccessOwned = PWR_FUNCTION_INACTIVE;
        }
        else
        {
            /* Other users hold the access or DBP was not set by this module */
        }
    }
    else
    {
        /* The access was not taken, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Selects the charging of the VBAT battery through a resistor.
 *
 * \param vbatCharging [in]: Charging resistor or charging disabled
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         selection is invalid or a register did not take the value
 */
pwr_RequestState_t Pwr_Set_VbatCharging( pwr_VbatCharging_t vbatCharging )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Set_VbatCharging( vbatCharging );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables the VDDUSB monitor, waits until VDDUSB is ready and declares the supply valid.
 *
 * The Rcc module can declare the supply valid too, the state is not shared between the modules.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         device line has no VDDUSB supply, VDDUSB is not ready or a register did not take the value
 */
pwr_RequestState_t Pwr_Set_UsbSupplyActive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Set_UsbSupply( PWR_FUNCTION_ACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Declares the VDDUSB supply not valid and disables its monitor.
 *
 * \return \ref PWR_REQUEST_OK on success, \ref PWR_REQUEST_ERROR when the module is not initialized, the
 *         device line has no VDDUSB supply or a register did not take the value
 */
pwr_RequestState_t Pwr_Set_UsbSupplyInactive( void )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        retState = Pwr_Reg_Set_UsbSupply( PWR_FUNCTION_INACTIVE );
    }
    else
    {
        /* Module not initialized, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Independent I/O bank supply, the STM32H5 PWR has no validity bit for VDDIO2 (the monitor works
 *        always, read it with \ref Pwr_Get_SupplyState).
 *
 * \return \ref PWR_REQUEST_ERROR always
 */
pwr_RequestState_t Pwr_Set_Io2SupplyActive( void )
{
    return ( PWR_REQUEST_ERROR );
}


/**
 * \brief Sets the priority of the PVD / AVD interrupt.
 *
 * \param irqPriority [in]: Interrupt priority (0 - highest)
 * \return Result of \ref Nvic_Set_PeriphIrq_Prio for \ref NVIC_PERIPH_IRQ_PVD_AVD, \ref PWR_REQUEST_ERROR
 *         when the module is not initialized
 */
pwr_RequestState_t Pwr_Set_IrqPriority( pwr_IrqPrio_t irqPriority )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE == pwr_State.Initialized )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_PVD_AVD, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* The priority was not accepted by the Nvic module */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module not initialized, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads the priority of the PVD / AVD interrupt.
 *
 * \param irqPriority [out]: Interrupt priority (0 - highest)
 * \return Result of \ref Nvic_Get_PeriphIrq_Prio for \ref NVIC_PERIPH_IRQ_PVD_AVD, \ref PWR_REQUEST_ERROR
 *         when the module is not initialized or the pointer is null
 */
pwr_RequestState_t Pwr_Get_IrqPriority( pwr_IrqPrio_t * const irqPriority )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( ( PWR_FUNCTION_ACTIVE == pwr_State.Initialized ) &&
        ( PWR_NULL_PTR        != irqPriority           )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( NVIC_PERIPH_IRQ_PVD_AVD, irqPriority );

        if( NVIC_REQUEST_OK == nvicState )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* The priority was not read by the Nvic module */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Module not initialized or invalid pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks the configuration of \ref Pwr_Init before any register is written.
 *
 * \param pwrConfig [in]: Configuration to check
 * \return \ref PWR_REQUEST_OK if every item is valid, otherwise \ref PWR_REQUEST_ERROR
 */
static pwr_RequestState_t Pwr_Check_Config( const pwr_Config_t * const pwrConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != pwrConfig )
    {
        const pwr_FunctionState_t stateList[PWR_CONFIG_STATE_CNT] =
        {
            pwrConfig->StopFlashLowPower,
            pwrConfig->BackupMonitor,
            pwrConfig->PvdActive,
            pwrConfig->AvdActive,
            pwrConfig->SupplyIsr,
        };

        retState = PWR_REQUEST_OK;

        if( ( PWR_VOLTAGE_SCALE_CNT <= pwrConfig->VoltageScale  ) ||
            ( PWR_REGULATOR_CNT     <= pwrConfig->RegulatorMode ) ||
            ( PWR_PVD_LEVEL_CNT     <= pwrConfig->Pvd.Level     ) ||
            ( PWR_AVD_LEVEL_CNT     <= pwrConfig->Avd.Level     )    )
        {
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The scalar items are in range */
        }

        for( uint32_t stateIdx = 0u; PWR_CONFIG_STATE_CNT > stateIdx; stateIdx++ )
        {
            const pwr_RequestState_t stateCheck = Pwr_Check_FunctionState( stateList[stateIdx] );

            if( PWR_REQUEST_OK != stateCheck )
            {
                retState = PWR_REQUEST_ERROR;
            }
            else
            {
                /* The function state is valid */
            }
        }

        const pwr_RequestState_t pvdEvent = Pwr_Check_Event( pwrConfig->Pvd.Event );
        const pwr_RequestState_t avdEvent = Pwr_Check_Event( pwrConfig->Avd.Event );

        if( ( PWR_REQUEST_OK != pvdEvent ) ||
            ( PWR_REQUEST_OK != avdEvent )    )
        {
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The events are valid */
        }

        for( uint32_t pinIdx = 0u; (uint32_t)PWR_WAKEUP_PIN_UNUSED > pinIdx; pinIdx++ )
        {
            const pwr_RequestState_t activeCheck = Pwr_Check_FunctionState( pwrConfig->WakeUpPinActive[pinIdx] );
            const pwr_RequestState_t pinCheck    = Pwr_Check_WakeUpPinConfig( &pwrConfig->WakeUpPin[pinIdx] );

            if( ( PWR_REQUEST_OK != activeCheck ) ||
                ( PWR_REQUEST_OK != pinCheck    )    )
            {
                retState = PWR_REQUEST_ERROR;
            }
            else
            {
                /* The wake-up pin item is valid */
            }
        }
    }
    else
    {
        /* Invalid configuration pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a function state is one of the items of the enumeration.
 *
 * \param functionState [in]: Function state
 * \return \ref PWR_REQUEST_OK if the state is valid
 */
static pwr_RequestState_t Pwr_Check_FunctionState( pwr_FunctionState_t functionState )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_FUNCTION_ACTIVE >= functionState )
    {
        retState = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid function state, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a supply event is one of the items of the enumeration.
 *
 * \param supplyEvent [in]: Supply event
 * \return \ref PWR_REQUEST_OK if the event is valid
 */
static pwr_RequestState_t Pwr_Check_Event( pwr_SupplyEvent_t supplyEvent )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_SUPPLY_EVENT_BOTH >= supplyEvent )
    {
        retState = PWR_REQUEST_OK;
    }
    else
    {
        /* Invalid supply event, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the polarity and the pull of a wake-up pin configuration.
 *
 * \param pinConfig [in]: Wake-up pin configuration
 * \return \ref PWR_REQUEST_OK if both items are valid
 */
static pwr_RequestState_t Pwr_Check_WakeUpPinConfig( const pwr_WakeUpPinConfig_t * const pinConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != pinConfig )
    {
        if( ( PWR_WAKEUP_FALLING >= pinConfig->Polarity ) &&
            ( PWR_PULL_DOWN      >= pinConfig->Pull     )    )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* Invalid polarity or pull, no action required */
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
 * \brief Applies a checked configuration step by step, stops at the first step that fails.
 *
 * The callbacks and the interrupt are prepared before the supervisors are enabled.
 *
 * \param pwrConfig [in]: Checked configuration
 * \return \ref PWR_REQUEST_OK if every step succeeded and the voltage scale reads back as required
 */
static pwr_RequestState_t Pwr_Apply_Config( const pwr_Config_t * const pwrConfig )
{
    pwr_RequestState_t retState = Pwr_Apply_Regulator( pwrConfig->RegulatorMode );

    if( PWR_REQUEST_OK == retState )
    {
        retState = Pwr_Reg_Set_VoltageScale( pwrConfig->VoltageScale );
    }
    else
    {
        /* The regulator was not applied, the scale is not written */
    }

    if( PWR_REQUEST_OK == retState )
    {
        retState = Pwr_Reg_Set_StopFlashLowPower( pwrConfig->StopFlashLowPower );
    }
    else
    {
        /* An earlier step failed */
    }

    if( PWR_REQUEST_OK == retState )
    {
        const pwr_FunctionState_t monitorState = Pwr_Reg_Get_BackupMonitor();

        if( monitorState != pwrConfig->BackupMonitor )
        {
            retState = Pwr_Apply_BackupMonitor( pwrConfig->BackupMonitor );
        }
        else
        {
            /* The monitor is in the required state, no write access is needed */
        }
    }
    else
    {
        /* An earlier step failed */
    }

    for( uint32_t pinIdx = 0u;
         ( (uint32_t)PWR_WAKEUP_PIN_UNUSED > pinIdx ) &&
         ( PWR_REQUEST_OK                  == retState );
         pinIdx++ )
    {
        if( PWR_FUNCTION_ACTIVE == pwrConfig->WakeUpPinActive[pinIdx] )
        {
            retState = Pwr_Apply_WakeUpPin( (pwr_WakeUpPin_t)pinIdx, &pwrConfig->WakeUpPin[pinIdx] );
        }
        else
        {
            /* The pin is not armed by the configuration */
        }
    }

    if( PWR_REQUEST_OK == retState )
    {
        pwr_State.PreEnterCallback = pwrConfig->PreEnterCallback;
        pwr_State.PostExitCallback = pwrConfig->PostExitCallback;
        pwr_State.SupplyCallback   = pwrConfig->SupplyCallback;
        pwr_State.SupplyIsr        = pwrConfig->SupplyIsr;

        retState = Pwr_Set_IsrConfig( pwrConfig );
    }
    else
    {
        /* An earlier step failed */
    }

    if( PWR_REQUEST_OK == retState )
    {
        retState = Pwr_Apply_PvdConfig( &pwrConfig->Pvd );
    }
    else
    {
        /* An earlier step failed */
    }

    if( PWR_REQUEST_OK == retState )
    {
        retState = Pwr_Apply_AvdConfig( &pwrConfig->Avd );
    }
    else
    {
        /* An earlier step failed */
    }

    if( ( PWR_REQUEST_OK        == retState             ) &&
        ( PWR_FUNCTION_ACTIVE   == pwrConfig->PvdActive )    )
    {
        retState = Pwr_Apply_PvdState( PWR_FUNCTION_ACTIVE );
    }
    else
    {
        /* An earlier step failed or the PVD is not enabled by the configuration */
    }

    if( ( PWR_REQUEST_OK        == retState             ) &&
        ( PWR_FUNCTION_ACTIVE   == pwrConfig->AvdActive )    )
    {
        retState = Pwr_Apply_AvdState( PWR_FUNCTION_ACTIVE );
    }
    else
    {
        /* An earlier step failed or the AVD is not enabled by the configuration */
    }

    if( PWR_REQUEST_OK == retState )
    {
        pwr_VoltageScale_t appliedScale = PWR_VOLTAGE_SCALE_CNT;

        retState = Pwr_Reg_Get_VoltageScale( &appliedScale );

        if( ( PWR_REQUEST_OK              == retState                  ) &&
            ( pwrConfig->VoltageScale     != appliedScale              )    )
        {
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The voltage scale reads back as required */
        }
    }
    else
    {
        /* An earlier step failed */
    }

    return ( retState );
}


/**
 * \brief Applies the supply of VCORE: the bypass is written (once), the LDO and the SMPS only have to
 *        match the regulator the package selected.
 *
 * \param regulatorMode [in]: Required supply of VCORE
 * \return \ref PWR_REQUEST_OK if the supply is as required
 */
static pwr_RequestState_t Pwr_Apply_Regulator( pwr_RegulatorMode_t regulatorMode )
{
    pwr_RequestState_t   retState     = PWR_REQUEST_ERROR;
    pwr_RegulatorMode_t  actualMode   = PWR_REGULATOR_CNT;
    pwr_RequestState_t   actualState  = Pwr_Reg_Get_RegulatorMode( &actualMode );

    if( PWR_REGULATOR_CNT > regulatorMode )
    {
        if( ( PWR_REQUEST_OK == actualState ) &&
            ( regulatorMode  == actualMode  )    )
        {
            /* The supply is as required, nothing is written */
            retState = PWR_REQUEST_OK;
        }
        else if( PWR_REGULATOR_BYPASS == regulatorMode )
        {
            retState = Pwr_Reg_Set_Bypass();
        }
        else
        {
            /* The LDO / SMPS is selected by the package, it cannot be changed by software */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid mode, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the monitoring of the VBAT voltage and of the temperature with the write access to
 *        the backup domain taken for the time of the write.
 *
 * \param monitorState [in]: \ref PWR_FUNCTION_ACTIVE enables the monitoring
 * \return \ref PWR_REQUEST_OK if the access was taken, the bit took the value and the access was released
 */
static pwr_RequestState_t Pwr_Apply_BackupMonitor( pwr_FunctionState_t monitorState )
{
    pwr_RequestState_t retState    = PWR_REQUEST_ERROR;
    pwr_RequestState_t accessState = Pwr_Set_BackupAccessActive();

    if( PWR_REQUEST_OK == accessState )
    {
        const pwr_RequestState_t writeState   = Pwr_Reg_Set_BackupMonitor( monitorState );
        const pwr_RequestState_t releaseState = Pwr_Set_BackupAccessInactive();

        if( ( PWR_REQUEST_OK == writeState   ) &&
            ( PWR_REQUEST_OK == releaseState )    )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* The bit did not take the value or the access could not be released */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* The write access to the backup domain could not be taken, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the retention of the backup SRAM (backup regulator) with the write access to the
 *        backup domain taken for the time of the write.
 *
 * \param retention [in]: \ref PWR_FUNCTION_ACTIVE keeps the content in Standby and VBAT modes
 * \return \ref PWR_REQUEST_OK if the access was taken, the regulator took the value and the access was released
 */
static pwr_RequestState_t Pwr_Apply_BackupRam( pwr_FunctionState_t retention )
{
    pwr_RequestState_t retState    = PWR_REQUEST_ERROR;
    pwr_RequestState_t accessState = Pwr_Set_BackupAccessActive();

    if( PWR_REQUEST_OK == accessState )
    {
        const pwr_RequestState_t writeState   = Pwr_Reg_Set_SramRetention( PWR_SRAM_BACKUP, retention );
        const pwr_RequestState_t releaseState = Pwr_Set_BackupAccessInactive();

        if( ( PWR_REQUEST_OK == writeState   ) &&
            ( PWR_REQUEST_OK == releaseState )    )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* The regulator did not take the value or the access could not be released */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* The write access to the backup domain could not be taken, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Arms a wake-up pin: the pin is set to analog mode without pull in the GPIO, then the polarity
 *        and the pull are written to the PWR and the pin is enabled.
 *
 * \param wakeUpPin [in]: Wake-up pin
 * \param pinConfig [in]: Polarity and pull of the pin
 * \return \ref PWR_REQUEST_OK if the pin was armed
 */
static pwr_RequestState_t Pwr_Apply_WakeUpPin( pwr_WakeUpPin_t wakeUpPin,
                                               const pwr_WakeUpPinConfig_t * const pinConfig )
{
    pwr_RequestState_t         retState   = PWR_REQUEST_ERROR;
    const pwr_WakeUpPinInfo_t *info       = PWR_NULL_PTR;
    const pwr_RequestState_t   infoState  = Pwr_Reg_Get_WakeUpPinInfo( wakeUpPin, &info );
    const pwr_RequestState_t   configCheck = Pwr_Check_WakeUpPinConfig( pinConfig );

    if( ( PWR_REQUEST_OK == infoState   ) &&
        ( PWR_REQUEST_OK == configCheck )    )
    {
        const gpio_RequestState_t pinState = Pwr_Set_AnalogPin( info->GpioPort, info->GpioPin );

        if( GPIO_REQUEST_OK == pinState )
        {
            retState = Pwr_Reg_Set_WakeUpPin( info, pinConfig );

            if( PWR_REQUEST_OK == retState )
            {
                pwr_State.WakeUpPinActive[wakeUpPin] = PWR_FUNCTION_ACTIVE;
            }
            else
            {
                /* The pin was not armed */
            }
        }
        else
        {
            /* The pin could not be set to analog mode, it is not armed */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid pin or configuration, nothing is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Sets a GPIO pin to analog mode without pull through \ref Gpio_Init, which also enables the clock of the
 *        GPIO port (the pin modes of a port without a clock are not written).
 *
 * \param gpioPort [in]: GPIO port of the pin
 * \param gpioPin  [in]: GPIO pin number
 * \return \ref GPIO_REQUEST_OK if the pin was configured, otherwise \ref GPIO_REQUEST_ERROR
 */
static gpio_RequestState_t Pwr_Set_AnalogPin( gpio_PortId_t gpioPort, gpio_PinId_t gpioPin )
{
    gpio_Config_t gpioConfig;

    gpioConfig.PortId         = gpioPort;
    gpioConfig.PinId          = gpioPin;
    gpioConfig.PinMode        = GPIO_PIN_MODE_ANALOG;
    gpioConfig.PinPull        = GPIO_PIN_PULL_NONE;
    gpioConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
    gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    gpioConfig.PinAltFunction = GPIO_ALT_FUNC_0;
    gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    return ( Gpio_Init( &gpioConfig ) );
}


/**
 * \brief Disarms a wake-up pin and forgets that the module armed it.
 *
 * \param wakeUpPin [in]: Wake-up pin
 * \return \ref PWR_REQUEST_OK if the enable bit of the pin was cleared
 */
static pwr_RequestState_t Pwr_Apply_WakeUpPinInactive( pwr_WakeUpPin_t wakeUpPin )
{
    pwr_RequestState_t         retState  = PWR_REQUEST_ERROR;
    const pwr_WakeUpPinInfo_t *info      = PWR_NULL_PTR;
    const pwr_RequestState_t   infoState = Pwr_Reg_Get_WakeUpPinInfo( wakeUpPin, &info );

    if( PWR_REQUEST_OK == infoState )
    {
        retState = Pwr_Reg_Set_WakeUpPinInactive( info );

        if( PWR_REQUEST_OK == retState )
        {
            pwr_State.WakeUpPinActive[wakeUpPin] = PWR_FUNCTION_INACTIVE;
        }
        else
        {
            /* The pin stays armed */
        }
    }
    else
    {
        /* Invalid pin, no register is written */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the PVD threshold (the PVD_IN pin to analog mode when it is compared) and keeps the event.
 *
 * \param pvdConfig [in]: Threshold and event of the PVD
 * \return \ref PWR_REQUEST_OK if the threshold read back as written
 */
static pwr_RequestState_t Pwr_Apply_PvdConfig( const pwr_PvdConfig_t * const pvdConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != pvdConfig )
    {
        const pwr_RequestState_t eventCheck = Pwr_Check_Event( pvdConfig->Event );
        gpio_RequestState_t      pinState   = GPIO_REQUEST_OK;

        if( ( PWR_REQUEST_OK    == eventCheck         ) &&
            ( PWR_PVD_LEVEL_CNT >  pvdConfig->Level   )    )
        {
            if( PWR_PVD_LEVEL_PIN == pvdConfig->Level )
            {
                pinState = Pwr_Set_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_15 );
            }
            else
            {
                /* The VDD is compared, no pin is used */
            }

            if( GPIO_REQUEST_OK == pinState )
            {
                retState = Pwr_Reg_Set_PvdLevel( pvdConfig->Level );

                if( PWR_REQUEST_OK == retState )
                {
                    pwr_State.PvdEvent = pvdConfig->Event;
                }
                else
                {
                    /* The threshold did not take the value */
                }
            }
            else
            {
                /* The PVD_IN pin could not be set to analog mode */
                retState = PWR_REQUEST_ERROR;
            }
        }
        else
        {
            /* Invalid threshold or event, no register is written */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid configuration pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes the AVD threshold and keeps the event.
 *
 * \param avdConfig [in]: Threshold and event of the AVD
 * \return \ref PWR_REQUEST_OK if the threshold read back as written
 */
static pwr_RequestState_t Pwr_Apply_AvdConfig( const pwr_AvdConfig_t * const avdConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_ERROR;

    if( PWR_NULL_PTR != avdConfig )
    {
        const pwr_RequestState_t eventCheck = Pwr_Check_Event( avdConfig->Event );

        if( ( PWR_REQUEST_OK    == eventCheck       ) &&
            ( PWR_AVD_LEVEL_CNT >  avdConfig->Level )    )
        {
            retState = Pwr_Reg_Set_AvdLevel( avdConfig->Level );

            if( PWR_REQUEST_OK == retState )
            {
                pwr_State.AvdEvent = avdConfig->Event;
            }
            else
            {
                /* The threshold did not take the value */
            }
        }
        else
        {
            /* Invalid threshold or event, no register is written */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid configuration pointer, no action required */
        retState = PWR_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables or disables the PVD, remembers the supply state seen at the enabling and updates the
 *        supervisor interrupt line.
 *
 * \param pvdState [in]: \ref PWR_FUNCTION_ACTIVE enables the PVD
 * \return \ref PWR_REQUEST_OK if the PVD took the state and the interrupt line follows it
 */
static pwr_RequestState_t Pwr_Apply_PvdState( pwr_FunctionState_t pvdState )
{
    pwr_RequestState_t retState = Pwr_Reg_Set_PvdState( pvdState );

    if( PWR_REQUEST_OK == retState )
    {
        pwr_State.PvdActive = pvdState;

        if( PWR_FUNCTION_ACTIVE == pvdState )
        {
            pwr_State.PvdState = Pwr_Get_SupervisorState( PWR_SUPPLY_VDD );
        }
        else
        {
            /* The PVD is disabled, there is no state to remember */
        }

        retState = Pwr_Set_SupervisorLine();
    }
    else
    {
        /* The PVD did not take the state (it can be locked by SBS), the interrupt line is not changed */
    }

    return ( retState );
}


/**
 * \brief Enables or disables the AVD, remembers the supply state seen at the enabling and updates the
 *        supervisor interrupt line.
 *
 * \param avdState [in]: \ref PWR_FUNCTION_ACTIVE enables the AVD
 * \return \ref PWR_REQUEST_OK if the AVD took the state and the interrupt line follows it
 */
static pwr_RequestState_t Pwr_Apply_AvdState( pwr_FunctionState_t avdState )
{
    pwr_RequestState_t retState = Pwr_Reg_Set_AvdState( avdState );

    if( PWR_REQUEST_OK == retState )
    {
        pwr_State.AvdActive = avdState;

        if( PWR_FUNCTION_ACTIVE == avdState )
        {
            pwr_State.AvdState = Pwr_Get_SupervisorState( PWR_SUPPLY_ANALOG );
        }
        else
        {
            /* The AVD is disabled, there is no state to remember */
        }

        retState = Pwr_Set_SupervisorLine();
    }
    else
    {
        /* The AVD did not take the state, the interrupt line is not changed */
    }

    return ( retState );
}


/**
 * \brief Registers the handler of the PVD / AVD interrupt and sets its priority when the supervisor
 *        events are taken from the interrupt.
 *
 * \param pwrConfig [in]: Checked configuration
 * \return \ref PWR_REQUEST_OK if the interrupt is prepared or not used
 */
static pwr_RequestState_t Pwr_Set_IsrConfig( const pwr_Config_t * const pwrConfig )
{
    pwr_RequestState_t retState = PWR_REQUEST_OK;

    if( ( PWR_FUNCTION_ACTIVE == pwrConfig->SupplyIsr      ) &&
        ( PWR_NULL_PTR        != pwrConfig->SupplyCallback )    )
    {
        const nvic_RequestState_t handlerState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_PVD_AVD, Pwr_IsrHandler );
        const nvic_RequestState_t prioState    = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_PVD_AVD,
                                                                          pwrConfig->IrqPriority );

        if( ( NVIC_REQUEST_OK == handlerState ) &&
            ( NVIC_REQUEST_OK == prioState    )    )
        {
            retState = PWR_REQUEST_OK;
        }
        else
        {
            /* The interrupt could not be prepared */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else
    {
        /* The supervisor events are polled or not used */
        retState = PWR_REQUEST_OK;
    }

    return ( retState );
}


/**
 * \brief Enables the EXTI line and the interrupt of the PVD / AVD when a supervisor is enabled and the
 *        events are taken from the interrupt, otherwise disables them (only changes are written).
 *
 * \return \ref PWR_REQUEST_OK if the line and the interrupt are in the required state
 */
static pwr_RequestState_t Pwr_Set_SupervisorLine( void )
{
    pwr_RequestState_t  retState   = PWR_REQUEST_OK;
    pwr_FunctionState_t lineWanted = PWR_FUNCTION_INACTIVE;

    if( ( PWR_NULL_PTR        != pwr_State.SupplyCallback ) &&
        ( PWR_FUNCTION_ACTIVE == pwr_State.SupplyIsr      )    )
    {
        if( PWR_FUNCTION_ACTIVE == pwr_State.PvdActive )
        {
            lineWanted = PWR_FUNCTION_ACTIVE;
        }
        else if( PWR_FUNCTION_ACTIVE == pwr_State.AvdActive )
        {
            lineWanted = PWR_FUNCTION_ACTIVE;
        }
        else
        {
            /* No supervisor is enabled */
            lineWanted = PWR_FUNCTION_INACTIVE;
        }
    }
    else
    {
        /* No callback or the events are polled */
        lineWanted = PWR_FUNCTION_INACTIVE;
    }

    if( ( PWR_FUNCTION_ACTIVE   == lineWanted              ) &&
        ( PWR_FUNCTION_INACTIVE == pwr_State.LineActive    )    )
    {
        exti_LineConfig_t         lineConfig;
        lineConfig.LineMode    = EXTI_LINE_MODE_INTERRUPT;
        lineConfig.TriggerEdge = EXTI_TRIGGER_EDGE_BOTH;

        const exti_RequestState_t extiState = Exti_Set_LineActive( EXTI_LINE_PVD_AVD, &lineConfig );
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_PVD_AVD );

        if( ( EXTI_REQUEST_OK == extiState ) &&
            ( NVIC_REQUEST_OK == nvicState )    )
        {
            pwr_State.LineActive = PWR_FUNCTION_ACTIVE;
        }
        else
        {
            /* The line or the interrupt could not be enabled */
            retState = PWR_REQUEST_ERROR;
        }
    }
    else if( ( PWR_FUNCTION_INACTIVE == lineWanted           ) &&
             ( PWR_FUNCTION_ACTIVE   == pwr_State.LineActive )    )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_PVD_AVD );
        const exti_RequestState_t extiState = Exti_Set_LineInactive( EXTI_LINE_PVD_AVD );

        pwr_State.LineActive = PWR_FUNCTION_INACTIVE;

        if( ( EXTI_REQUEST_OK != extiState ) ||
            ( NVIC_REQUEST_OK != nvicState )    )
        {
            /* The line or the interrupt could not be disabled */
            retState = PWR_REQUEST_ERROR;
        }
        else
        {
            /* The line and the interrupt are disabled */
        }
    }
    else
    {
        /* The line is in the required state */
    }

    return ( retState );
}


/**
 * \brief Reads the state of a supervised supply.
 *
 * \param supply [in]: \ref PWR_SUPPLY_VDD or \ref PWR_SUPPLY_ANALOG
 * \return State of the supply, valid when the state cannot be read
 */
static pwr_SupplyState_t Pwr_Get_SupervisorState( pwr_Supply_t supply )
{
    pwr_SupplyState_t        supplyState = PWR_SUPPLY_STATE_VALID;
    const pwr_RequestState_t readState   = Pwr_Reg_Get_SupplyState( supply, &supplyState );

    if( PWR_REQUEST_OK != readState )
    {
        supplyState = PWR_SUPPLY_STATE_VALID;
    }
    else
    {
        /* The state was read */
    }

    return ( supplyState );
}


/**
 * \brief Tells whether a change of a supply is reported for the configured event.
 *
 * \param supplyEvent [in]: Configured event
 * \param supplyState [in]: State the supply changed to
 * \return \ref PWR_FUNCTION_ACTIVE if the callback is called for the change
 */
static pwr_FunctionState_t Pwr_Get_EventReported( pwr_SupplyEvent_t supplyEvent,
                                                 pwr_SupplyState_t supplyState )
{
    pwr_FunctionState_t retState = PWR_FUNCTION_INACTIVE;

    if( PWR_SUPPLY_EVENT_BOTH == supplyEvent )
    {
        retState = PWR_FUNCTION_ACTIVE;
    }
    else if( ( PWR_SUPPLY_EVENT_DROP      == supplyEvent ) &&
             ( PWR_SUPPLY_STATE_NOT_VALID == supplyState )    )
    {
        retState = PWR_FUNCTION_ACTIVE;
    }
    else if( ( PWR_SUPPLY_EVENT_RISE   == supplyEvent ) &&
             ( PWR_SUPPLY_STATE_VALID  == supplyState )    )
    {
        retState = PWR_FUNCTION_ACTIVE;
    }
    else
    {
        /* The change does not match the configured event */
        retState = PWR_FUNCTION_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Calls a hook of the application when it is registered.
 *
 * \param hookCallback [in]: Hook, \ref PWR_NULL_PTR when not registered
 */
static void Pwr_Call_Hook( pwr_HookCallback_t * const hookCallback )
{
    if( PWR_NULL_PTR != hookCallback )
    {
        hookCallback();
    }
    else
    {
        /* No hook registered, no action required */
    }
}


/**
 * \brief Compares the state of a supervised supply with the last seen one and calls the callback for a
 *        change that matches the configured event.
 *
 * \param supply [in]: \ref PWR_SUPPLY_VDD or \ref PWR_SUPPLY_ANALOG
 */
static void Pwr_Process_Supply( pwr_Supply_t supply )
{
    pwr_FunctionState_t supervisorActive = pwr_State.PvdActive;
    pwr_SupplyState_t   lastState        = pwr_State.PvdState;
    pwr_SupplyEvent_t   supplyEvent      = pwr_State.PvdEvent;

    if( PWR_SUPPLY_ANALOG == supply )
    {
        supervisorActive = pwr_State.AvdActive;
        lastState        = pwr_State.AvdState;
        supplyEvent      = pwr_State.AvdEvent;
    }
    else
    {
        /* The PVD supervises VDD */
    }

    if( PWR_FUNCTION_ACTIVE == supervisorActive )
    {
        const pwr_SupplyState_t   newState = Pwr_Get_SupervisorState( supply );
        pwr_SupplyCallback_t     *callback = pwr_State.SupplyCallback;

        if( newState != lastState )
        {
            const pwr_FunctionState_t reported = Pwr_Get_EventReported( supplyEvent, newState );

            if( PWR_SUPPLY_ANALOG == supply )
            {
                pwr_State.AvdState = newState;
            }
            else
            {
                pwr_State.PvdState = newState;
            }

            if( ( PWR_FUNCTION_ACTIVE == reported ) &&
                ( PWR_NULL_PTR        != callback )    )
            {
                callback( supply, newState );
            }
            else
            {
                /* The change is not reported */
            }
        }
        else
        {
            /* The supply did not change */
        }
    }
    else
    {
        /* The supervisor is not enabled */
    }
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief Handler of the PVD / AVD interrupt (EXTI line 16): clears the pending flag of the line and
 *        reports the supply changes to the callback.
 */
static void Pwr_IsrHandler( void )
{
    (void)Exti_Set_LinePendingClear( EXTI_LINE_PVD_AVD );

    Pwr_Process_Supply( PWR_SUPPLY_VDD );
    Pwr_Process_Supply( PWR_SUPPLY_ANALOG );
}

/* ================================ TASKS =================================== */
