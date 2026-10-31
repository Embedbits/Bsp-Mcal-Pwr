/**
 * \author Mr.Nobody
 * \file ItTest_Pwr.c
 * \ingroup Pwr
 * \brief Integration tests of the Pwr module on target.
 *
 * The Pwr module runs on the MCU together with the real GPIO, EXTI, NVIC and SCB modules and the real PWR peripheral.
 * The tests verify what the unit tests cannot (the emulated registers do not follow the hardware): the voltage scale
 * that becomes ready on the real regulator, the read-back of the registers of the real peripheral, the supply monitors
 * that report a valid supply on a powered board, the pin mode of a wake-up pin on the real GPIO.
 *
 * The tests never put the MCU to Stop or Standby mode (the target would not be reachable for the framework), do not
 * need any external wiring (no fixture feature is required) and restore the settings they change: the voltage scale is
 * only raised (the clock configuration of the start-up needs the applied scale or a lower one) and put back to the
 * applied one. The entry to the low-power modes and the wake-up sources are covered by the unit tests only and are not
 * run on hardware yet.
 *
 * Used resources:
 * - PWR (the registers of the supervisors, the retention, the wake-up and the backup access)
 * - GPIO port A pin 0 (WKUP1, set to analog mode by the wake-up pin test)
 *
 * \note The supply states of VDDIO2 and of the backup domain depend on the board (supply of VDDIO2, battery on VBAT):
 *       the tests check that they can be read, not their value. The VDDUSB supply and the VBAT charging are not
 *       tested (they depend on the board wiring / the battery).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Pwr_Port.h"                       /* Module under test              */
#include "Gpio_Port.h"                      /* Pin mode read-back             */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItPwr_Init_Default          ( void );
static pwr_SupplyState_t ItPwr_Get_SettledState ( pwr_Supply_t supply );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Expected version of the module */
#define IT_PWR_VERSION_MAJOR                ( 1u )
#define IT_PWR_VERSION_MINOR                ( 0u )
#define IT_PWR_VERSION_PATCH                ( 0u )

/** Interrupt priority used by the tests */
#define IT_PWR_IRQ_PRIO                     ( 3u )

/** Count of the holders of the backup domain access taken by the test */
#define IT_PWR_HOLDER_CNT                   ( 2u )

/** Count of the polls of the status after a write that the hardware needs time to apply */
#define IT_PWR_SETTLE_POLL_CNT              ( 1000u )

/** Value of a flag that is not set */
#define IT_PWR_COUNT_NONE                   ( 0u )

/* ========================== LOCAL VARIABLES =============================== */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* No per-test state */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* =============================== TESTS ==================================== */

/**
 * \brief The module version of the firmware is the compiled-in one.
 * \par Expected results
 * Major / minor / patch fields hold the version of the module.
 */
void It_Pwr_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    const pwr_ModuleVersion_t version = Pwr_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT32( IT_PWR_VERSION_MAJOR, version.Major );
    TEST_ASSERT_EQUAL_UINT32( IT_PWR_VERSION_MINOR, version.Minor );
    TEST_ASSERT_EQUAL_UINT32( IT_PWR_VERSION_PATCH, version.Patch );
}


/**
 * \brief The configuration read from the real peripheral initializes the module and keeps the applied scale.
 * \par Expected results
 * \ref Pwr_Init succeeds, the voltage scale read through the module equals the one of the default configuration (the
 * scale that the start-up clock configuration applied, nothing is changed), \ref Pwr_Deinit succeeds.
 */
void It_Pwr_Init_DefaultConfig_KeepsAppliedScale( void )
{
    pwr_Config_t       pwrConfig;
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_DefaultConfig( &pwrConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Init( &pwrConfig ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &voltageScale ) );
    TEST_ASSERT_EQUAL( pwrConfig.VoltageScale, voltageScale );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Deinit() );
}


/**
 * \brief The regulator that the package uses is read and accepted as a request.
 * \par Expected results
 * The regulator mode is one of the list, the request for the same mode succeeds without a write.
 */
void It_Pwr_RegulatorMode_Package_IsReadAndAccepted( void )
{
    pwr_RegulatorMode_t regulatorMode = PWR_REGULATOR_CNT;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_TRUE( PWR_REGULATOR_CNT > regulatorMode );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_RegulatorMode( regulatorMode ) );
}


/**
 * \brief The voltage scale is raised scale by scale on the real regulator and put back.
 * \par Expected results
 * From the applied scale to the scale 0 (the highest performance) every scale is reached (the status reads it back
 * and VCORE is ready), then the applied scale is restored. The raise of the performance is always allowed, the
 * restore goes back to the scale the clock configuration needs.
 */
void It_Pwr_Set_VoltageScale_RaiseAndRestore_ReachesEveryScale( void )
{
    pwr_VoltageScale_t startScale   = PWR_VOLTAGE_SCALE_CNT;
    pwr_VoltageScale_t actualScale  = PWR_VOLTAGE_SCALE_CNT;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &startScale ) );

    for( uint32_t scaleIdx = (uint32_t)startScale; PWR_VOLTAGE_SCALE_0 < scaleIdx; scaleIdx-- )
    {
        const pwr_VoltageScale_t targetScale = (pwr_VoltageScale_t)( scaleIdx - 1u );

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VoltageScale( targetScale ) );
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &actualScale ) );
        TEST_ASSERT_EQUAL( targetScale, actualScale );
    }

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VoltageScale( startScale ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &actualScale ) );
    TEST_ASSERT_EQUAL( startScale, actualScale );
}


/**
 * \brief The write access to the backup domain is reference counted.
 * \par Expected results
 * Two takes and two releases succeed, a release without a take is rejected.
 */
void It_Pwr_BackupAccess_TakeAndRelease_IsBalanced( void )
{
    for( uint32_t holderIdx = 0u; IT_PWR_HOLDER_CNT > holderIdx; holderIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessActive() );
    }

    for( uint32_t holderIdx = 0u; IT_PWR_HOLDER_CNT > holderIdx; holderIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessInactive() );
    }

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupAccessInactive() );
}


/**
 * \brief The PVD with the lowest threshold reports a valid VDD on a powered board and is disabled again.
 * \par Expected results
 * The PVD enabling succeeds (the PVD is not locked), VDD reads valid (the board runs above the lowest threshold of
 * about 1.95 V), after the disabling the supply state cannot be read.
 */
void It_Pwr_Pvd_LowestLevel_ReportsValidVdd( void )
{
    const pwr_PvdConfig_t pvdConfig    = { .Level = PWR_PVD_LEVEL_0, .Event = PWR_SUPPLY_EVENT_DROP };
    pwr_SupplyState_t     supplyState  = PWR_SUPPLY_STATE_NOT_VALID;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdConfig( &pvdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdActive() );

    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, ItPwr_Get_SettledState( PWR_SUPPLY_VDD ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdInactive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );
}


/**
 * \brief The AVD with the lowest threshold reports a valid VDDA on a powered board and is disabled again.
 * \par Expected results
 * The AVD enabling succeeds, VDDA reads valid (above the lowest threshold of about 1.7 V), after the disabling the
 * supply state cannot be read.
 */
void It_Pwr_Avd_LowestLevel_ReportsValidVdda( void )
{
    const pwr_AvdConfig_t avdConfig    = { .Level = PWR_AVD_LEVEL_0, .Event = PWR_SUPPLY_EVENT_DROP };
    pwr_SupplyState_t     supplyState  = PWR_SUPPLY_STATE_NOT_VALID;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdConfig( &avdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdActive() );

    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, ItPwr_Get_SettledState( PWR_SUPPLY_ANALOG ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdInactive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );
}


/**
 * \brief The supply states that depend on the board can be read.
 * \par Expected results
 * The VDDIO2 state reads without an error. The backup monitor is enabled (the write access to the backup domain is
 * taken and released by the module), the VBAT and the temperature states read without an error, the monitor is
 * disabled again. The values are not checked (they depend on the board).
 */
void It_Pwr_Get_SupplyState_BoardDependentSupplies_AreReadable( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_VALID;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_IO2, &supplyState ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupMonitor( PWR_FUNCTION_ACTIVE ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupMonitor( PWR_FUNCTION_INACTIVE ) );
}


/**
 * \brief The retention of the first SRAM block and of the I/O groups is written and read back on the real registers.
 * \par Expected results
 * The retention of AHB SRAM1 reads back as written for both states and is restored to the initial state, the
 * retention of both I/O groups is written for both states (it acts in Standby mode only).
 */
void It_Pwr_Retention_SramAndIo_ReadBackAsWritten( void )
{
    pwr_FunctionState_t initialState = PWR_FUNCTION_INACTIVE;
    pwr_FunctionState_t retention    = PWR_FUNCTION_INACTIVE;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( PWR_SRAM_1, &initialState ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( PWR_SRAM_1, PWR_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( PWR_SRAM_1, &retention ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, retention );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( PWR_SRAM_1, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( PWR_SRAM_1, &retention ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, retention );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( PWR_SRAM_1, initialState ) );

    for( uint32_t groupIdx = 0u; (uint32_t)PWR_IO_GROUP_CNT > groupIdx; groupIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IoRetention( (pwr_IoGroup_t)groupIdx, PWR_FUNCTION_ACTIVE ) );
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IoRetention( (pwr_IoGroup_t)groupIdx, PWR_FUNCTION_INACTIVE ) );
    }
}


/**
 * \brief A wake-up pin is armed on the real GPIO and PWR and disarmed again, the flags are cleared.
 * \par Expected results
 * WKUP1 (PA0) is armed with the rising edge and the pull-down: the GPIO reads the analog mode of the pin, the
 * wake-up source mask can be read, the disarming succeeds, the clearing of the flags succeeds and the Standby flag
 * reads inactive afterwards. The MCU does not enter Standby mode.
 */
void It_Pwr_Set_WakeUpPin_Wkup1_ConfiguresAnalogPinAndArms( void )
{
    const pwr_WakeUpPinConfig_t pinConfig     = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_DOWN };
    pwr_WakeUpSource_t          wakeUpSource  = PWR_WAKEUP_SOURCE_NONE;
    pwr_FunctionState_t         standbyFlag   = PWR_FUNCTION_ACTIVE;
    gpio_PinMode_t              pinMode       = GPIO_PIN_MODE_OUTPUT;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_1, &pinConfig ) );
    TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Get_PinMode( GPIO_PORT_A, GPIO_PIN_ID_0, &pinMode ) );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ANALOG, pinMode );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPinInactive( PWR_WAKEUP_PIN_1 ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpFlagClear() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_StandbyFlag( &standbyFlag ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, standbyFlag );
}


/**
 * \brief The interrupt priority of the PVD / AVD interrupt is written and read back on the real Nvic.
 * \par Expected results
 * The priority read equals the priority written.
 */
void It_Pwr_IrqPriority_SetThenGet_RoundTrips( void )
{
    pwr_IrqPrio_t irqPriority = IT_PWR_COUNT_NONE;

    ItPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IrqPriority( IT_PWR_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_IrqPriority( &irqPriority ) );
    TEST_ASSERT_EQUAL_UINT32( IT_PWR_IRQ_PRIO, irqPriority );
}

/* ========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Initializes the module with the configuration read from the hardware.
 */
static void ItPwr_Init_Default( void )
{
    pwr_Config_t pwrConfig;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_DefaultConfig( &pwrConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Init( &pwrConfig ) );
}


/**
 * \brief Polls the state of a supply a bounded number of times until it reads valid.
 *
 * The detectors need a short time to deliver a valid output after they are enabled, the polling is bounded
 * and ends at the first valid state.
 *
 * \param supply [in]: Supply with an enabled monitor
 * \return The last state read (valid when the supply became valid within the polls)
 */
static pwr_SupplyState_t ItPwr_Get_SettledState( pwr_Supply_t supply )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_NOT_VALID;

    for( uint32_t pollIdx = 0u;
         ( IT_PWR_SETTLE_POLL_CNT > pollIdx ) &&
         ( PWR_SUPPLY_STATE_VALID != supplyState );
         pollIdx++ )
    {
        (void)Pwr_Get_SupplyState( supply, &supplyState );
    }

    return ( supplyState );
}
