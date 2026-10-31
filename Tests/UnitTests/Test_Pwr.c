/**
 * \author Mr.Nobody
 * \file Test_Pwr.c
 * \ingroup Pwr
 * \brief Unit tests of the Pwr module (STM32H5).
 *
 * Pwr.c and Pwr_Reg.c are compiled unchanged. The PWR registers are emulated by RegMem (plain memory:
 * write-1-to-clear, read-only status and self-clearing bits are not emulated). The behavior of the
 * hardware that the module waits for or verifies (the voltage scale becomes ready, a bit that is locked
 * does not take the written value, the flags the hardware sets) is emulated by HW models of RegMem or by
 * the preset of the status registers, the tests that use a model run serially. Gpio, Nvic, Exti and Scb are
 * mocked by CMock; the entry to a low-power mode is checked by the order of the calls of the mocks and
 * of the hooks, the instruction that stops the core is a stub that emulates what the hardware does
 * (for the Stop mode: the voltage scale is reset to scale 3).
 *
 * Limits of the emulation: the clearing of the flags by write-1-to-clear (PWR_WUSCR, PWR_PMCR.CSSF) is
 * checked by the written bits, the real clearing of PWR_WUSR / PWR_PMSR is not emulated. The wake-up from
 * Standby (a reset) cannot be run on the host.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Pwr_Port.h"                       /* Module under test              */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockExti_Port.h"                  /* EXTI module mock               */
#include "MockScb_Port.h"                   /* SCB component mock             */
#include "Stm32.h"                          /* PWR registers definition       */
/* ============================= TYPEDEFS =================================== */

/** Row of the table of the wake-up pins of the device line (CubeMX database and the reference manual of the line) */
typedef struct
{
    pwr_WakeUpPin_t Pin;        /**< Wake-up pin item                                  */
    gpio_PortId_t   Port;       /**< GPIO port of the pin                              */
    gpio_PinId_t    GpioPin;    /**< GPIO pin number                                   */
    uint32_t        EnableMask; /**< Enable bit of the pin in PWR_WUCR                 */
    uint32_t        FlagMask;   /**< Wake-up flag of the pin in PWR_WUSR               */
    uint32_t        SourceBit;  /**< Bit of the pin in the wake-up source mask         */
}   utPwr_PinRow_t;


/** Row of the table of the retention blocks of the device line (reference manual of the line) */
typedef struct
{
    pwr_SramId_t Sram;        /**< Retention block                                   */
    uint32_t     ShutOffMask; /**< Shut-off bit of the block in PWR_PMCR             */
}   utPwr_SramRow_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 UtPwr_Preset_Hardware       ( void );
static void                 UtPwr_Expect_AnalogPin      ( gpio_PortId_t gpioPort,
                                                          gpio_PinId_t gpioPin,
                                                          gpio_RequestState_t gpioState );
static pwr_Config_t         UtPwr_Get_Config            ( void );
static void                 UtPwr_Init_Default          ( void );
static void                 UtPwr_Init_Hooks            ( void );
static void                 UtPwr_Init_Supervised       ( pwr_SupplyEvent_t pvdEvent,
                                                          pwr_SupplyEvent_t avdEvent );
static void                 UtPwr_Init_Config           ( const pwr_Config_t * const pwrConfig );
static void                 UtPwr_Set_Scale             ( pwr_VoltageScale_t voltageScale );
static void                 UtPwr_Expect_IsrInit        ( pwr_IrqPrio_t irqPriority );
static void                 UtPwr_Expect_LineEnable     ( void );
static void                 UtPwr_Expect_LineDisable    ( void );
static void                 UtPwr_HookPre               ( void );
static void                 UtPwr_HookPost              ( void );
static void                 UtPwr_SupplyCallback        ( pwr_Supply_t supply, pwr_SupplyState_t supplyState );
static nvic_RequestState_t  UtPwr_NvicSetHandlerStub    ( nvic_PeriphIrqList_t irqId,
                                                          const nvic_IsrCallback_t irqHandler,
                                                          int callCnt );
static void                 UtPwr_ScbWfiStub            ( int callCnt );
static void                 UtPwr_ScbWfiStubVosReset    ( int callCnt );
static void                 UtPwr_Expect_Sleep          ( void );
static void                 UtPwr_Expect_Deep           ( void );
static void                 UtPwr_HwModel_Vos           ( void );
static void                 UtPwr_HwModel_VosNoRaiseReady( void );
static void                 UtPwr_HwModel_PvdLocked     ( void );
static void                 UtPwr_HwModel_DbpLocked     ( void );
static void                 UtPwr_HwModel_SvosNotKept   ( void );
static void                 UtPwr_HwModel_BypassLocked  ( void );
static void                 UtPwr_HwModel_PmcrNotKept   ( void );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Expected version of the module */
#define UT_PWR_VERSION_MAJOR                ( 1u )
#define UT_PWR_VERSION_MINOR                ( 0u )
#define UT_PWR_VERSION_PATCH                ( 0u )

/** Register value without any bit set */
#define UT_PWR_REG_ZERO                     ( 0u )

/** Count of the calls of a hook or of a callback */
#define UT_PWR_CALLS_NONE                   ( 0u )
#define UT_PWR_CALLS_FOUR                   ( 4u )
#define UT_PWR_CALLS_ONCE                   ( 1u )
#define UT_PWR_CALLS_TWICE                  ( 2u )

/** Item one past the last item of an enumeration (an invalid value) */
#define UT_PWR_ITEM_OFFSET                  ( 1u )

/** Values of the VOS / ACTVOS field for the scales (RM0481 PWR_VOSCR, unshifted) */
#define UT_PWR_VOS_FIELD_SCALE_3            ( 0x0u )
#define UT_PWR_VOS_FIELD_SCALE_2            ( 0x1u )
#define UT_PWR_VOS_FIELD_SCALE_1            ( 0x2u )
#define UT_PWR_VOS_FIELD_SCALE_0            ( 0x3u )

/** Values of the SVOS field for the Stop sub-modes (RM0481 PWR_PMCR, unshifted) */
#define UT_PWR_SVOS_FIELD_3                 ( 0x3u )
#define UT_PWR_SVOS_FIELD_4                 ( 0x2u )
#define UT_PWR_SVOS_FIELD_5                 ( 0x1u )

/** Values of the pull field of a wake-up pin (RM0481 PWR_WUCR, unshifted) */
#define UT_PWR_PULL_FIELD_NONE              ( 0x0u )
#define UT_PWR_PULL_FIELD_UP                ( 0x1u )
#define UT_PWR_PULL_FIELD_DOWN              ( 0x2u )

/** Width of the pull field of a wake-up pin [bit] */
#define UT_PWR_PULL_FIELD_WIDTH             ( 2u )

/** Count of the steps of the voltage scale walk between the scales with the given distance */
#define UT_PWR_STEPS_NONE                   ( 0u )
#define UT_PWR_STEPS_ONE                    ( 1u )
#define UT_PWR_STEPS_TWO                    ( 2u )
#define UT_PWR_STEPS_THREE                  ( 3u )

/** Count of the pull selections and of the polarity selections of a wake-up pin */
#define UT_PWR_PULL_COMBINATIONS            ( 3u )
#define UT_PWR_POLARITY_CNT                 ( 2u )

/** Interrupt priority used by the tests */
#define UT_PWR_IRQ_PRIO                     ( 5u )
#define UT_PWR_IRQ_PRIO_OTHER               ( 9u )

/** Index of the third step of a voltage scale walk in the recorder */
#define UT_PWR_STEP_THIRD                   ( 2u )

/** Capacity of the recorder of the VOS values written */
#define UT_PWR_VOS_HISTORY_SIZE             ( 8u )

/** Wake-up flags and the flag masks preset by the tests */
#define UT_PWR_WUF_NONE                     ( 0u )

/** Highest value of the count of the holders of the backup access */
#define UT_PWR_HOLDER_CNT                   ( 3u )

/* ========================== LOCAL VARIABLES =============================== */

/** Count of the Gpio_Init configurations kept for the mock of one test */
#define UT_PWR_GPIO_EXPECT_CNT              ( 32u )

/** Expected Gpio_Init configurations (the mock compares them when the call happens) */
static gpio_Config_t utPwr_ExpectedGpio[ UT_PWR_GPIO_EXPECT_CNT ];

/** Count of the expected Gpio_Init configurations used in the running test */
static uint32_t utPwr_ExpectedGpioCnt;

/** Sequence counter shared by the hooks, the callbacks and the stubs (increments at every event) */
static uint32_t utPwr_Seq;

/** Sequence number of the pre enter hook, of the post exit hook and of the stop instruction */
static uint32_t utPwr_PreSeq;
static uint32_t utPwr_PostSeq;
static uint32_t utPwr_WfiSeq;

/** Count of the calls of the hooks */
static uint32_t utPwr_PreCnt;
static uint32_t utPwr_PostCnt;

/** Register values seen by the stop instruction stub */
static uint32_t utPwr_PmcrAtWfi;

/** Recorder of the VOS values written (HW model) */
static uint32_t utPwr_VosHistory[ UT_PWR_VOS_HISTORY_SIZE ];
static uint32_t utPwr_VosHistoryCnt;

/** Last call of the supervisor callback */
static pwr_Supply_t      utPwr_CbSupply;
static pwr_SupplyState_t utPwr_CbState;
static uint32_t          utPwr_CbCnt;

/** Interrupt handler registered with the Nvic mock */
static nvic_IsrCallback_t utPwr_Isr;

/** Last VOS field value seen by the HW model */
static uint32_t utPwr_VosLast;

/** VOS / ACTVOS field value of every voltage scale, indexed by the scale */
static const uint32_t utPwr_ScaleField[ PWR_VOLTAGE_SCALE_CNT ] =
{
    UT_PWR_VOS_FIELD_SCALE_0,
    UT_PWR_VOS_FIELD_SCALE_1,
    UT_PWR_VOS_FIELD_SCALE_2,
    UT_PWR_VOS_FIELD_SCALE_3,
};

/** Configuration of the EXTI line that the module enables for the PVD / AVD interrupt */
static const exti_LineConfig_t utPwr_LineConfig =
{
    .LineMode    = EXTI_LINE_MODE_INTERRUPT,
    .TriggerEdge = EXTI_TRIGGER_EDGE_BOTH,
};

/** The wake-up pins of the device line: the pins of the CubeMX database, the bits of the register bit fields */
static const utPwr_PinRow_t utPwr_PinRows[] =
{
    { PWR_WAKEUP_PIN_1, GPIO_PORT_A, GPIO_PIN_ID_0,  PWR_WUCR_WUPEN1, PWR_WUSR_WUF1, PWR_WAKEUP_SOURCE_PIN_1 },
    { PWR_WAKEUP_PIN_2, GPIO_PORT_A, GPIO_PIN_ID_2,  PWR_WUCR_WUPEN2, PWR_WUSR_WUF2, PWR_WAKEUP_SOURCE_PIN_2 },
#if defined(STM32H503xx)
    { PWR_WAKEUP_PIN_3, GPIO_PORT_C, GPIO_PIN_ID_1,  PWR_WUCR_WUPEN3, PWR_WUSR_WUF3, PWR_WAKEUP_SOURCE_PIN_3 },
#elif defined(PWR_LINE_HAS_WKUP_3)
    { PWR_WAKEUP_PIN_3, GPIO_PORT_I, GPIO_PIN_ID_8,  PWR_WUCR_WUPEN3, PWR_WUSR_WUF3, PWR_WAKEUP_SOURCE_PIN_3 },
#endif
    { PWR_WAKEUP_PIN_4, GPIO_PORT_C, GPIO_PIN_ID_13, PWR_WUCR_WUPEN4, PWR_WUSR_WUF4, PWR_WAKEUP_SOURCE_PIN_4 },
    { PWR_WAKEUP_PIN_5, GPIO_PORT_B, GPIO_PIN_ID_7,  PWR_WUCR_WUPEN5, PWR_WUSR_WUF5, PWR_WAKEUP_SOURCE_PIN_5 },
#if defined(PWR_LINE_HAS_WKUP_6_8)
    { PWR_WAKEUP_PIN_6, GPIO_PORT_C, GPIO_PIN_ID_1,  PWR_WUCR_WUPEN6, PWR_WUSR_WUF6, PWR_WAKEUP_SOURCE_PIN_6 },
    { PWR_WAKEUP_PIN_7, GPIO_PORT_D, GPIO_PIN_ID_2,  PWR_WUCR_WUPEN7, PWR_WUSR_WUF7, PWR_WAKEUP_SOURCE_PIN_7 },
    { PWR_WAKEUP_PIN_8, GPIO_PORT_D, GPIO_PIN_ID_3,  PWR_WUCR_WUPEN8, PWR_WUSR_WUF8, PWR_WAKEUP_SOURCE_PIN_8 },
#endif
};

/** Count of the rows of the wake-up pin table (one per item of \ref pwr_WakeUpPin_t) */
#define UT_PWR_PIN_ROW_CNT                  ( sizeof( utPwr_PinRows ) / sizeof( utPwr_PinRows[0] ) )

_Static_assert( ( sizeof( utPwr_PinRows ) / sizeof( utPwr_PinRows[0] ) ) == PWR_WAKEUP_PIN_UNUSED,
                "Test_Pwr: the pin table does not match the list of the wake-up pins." );

/** The retention blocks of the device line with a shut-off bit (the backup SRAM has none) */
static const utPwr_SramRow_t utPwr_SramRows[] =
{
    { PWR_SRAM_1,        PWR_PMCR_SRAM1SO     },
#if defined(PWR_LINE_SRAM2_SINGLE)
    { PWR_SRAM_2,        PWR_PMCR_SRAM2SO     },
#endif
#if defined(PWR_LINE_SRAM2_48)
    { PWR_SRAM_2_48,     PWR_PMCR_SRAM2_48SO  },
#endif
#if defined(PWR_LINE_SRAM2_48_SPLIT)
    { PWR_SRAM_2_48_LOW, PWR_PMCR_SRAM2_48LSO },
    { PWR_SRAM_2_48_HIGH, PWR_PMCR_SRAM2_48HSO },
#endif
#if defined(PWR_LINE_SRAM2_16)
    { PWR_SRAM_2_16,     PWR_PMCR_SRAM2_16SO  },
#endif
#if defined(PWR_LINE_SRAM2_16_SPLIT)
    { PWR_SRAM_2_16_LOW, PWR_PMCR_SRAM2_16LSO },
    { PWR_SRAM_2_16_HIGH, PWR_PMCR_SRAM2_16HSO },
#endif
#if defined(PWR_LINE_HAS_SRAM3)
    { PWR_SRAM_3,        PWR_PMCR_SRAM3SO     },
#endif
#if defined(PWR_LINE_HAS_SRAM4_5)
    { PWR_SRAM_4,        PWR_PMCR_SRAM4SO     },
    { PWR_SRAM_5,        PWR_PMCR_SRAM5SO     },
#endif
#if defined(PWR_LINE_HAS_ETH_RAM)
    { PWR_SRAM_ETHERNET, PWR_PMCR_ETHERNETSO  },
#endif
#if defined(PWR_LINE_HAS_LTDC_RAM)
    { PWR_SRAM_LTDC,     PWR_PMCR_LTDCSO      },
#endif
};

/** Count of the rows of the retention table (one per item of \ref pwr_SramId_t except the backup SRAM) */
#define UT_PWR_SRAM_ROW_CNT                 ( sizeof( utPwr_SramRows ) / sizeof( utPwr_SramRows[0] ) )

_Static_assert( ( sizeof( utPwr_SramRows ) / sizeof( utPwr_SramRows[0] ) ) == PWR_SRAM_BACKUP,
                "Test_Pwr: the SRAM table does not match the list of the retention blocks." );

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utPwr_Seq           = UT_PWR_CALLS_NONE;
    utPwr_ExpectedGpioCnt = UT_PWR_CALLS_NONE;
    utPwr_PreSeq        = UT_PWR_CALLS_NONE;
    utPwr_PostSeq       = UT_PWR_CALLS_NONE;
    utPwr_WfiSeq        = UT_PWR_CALLS_NONE;
    utPwr_PreCnt        = UT_PWR_CALLS_NONE;
    utPwr_PostCnt       = UT_PWR_CALLS_NONE;
    utPwr_PmcrAtWfi     = UT_PWR_REG_ZERO;
    utPwr_VosHistoryCnt = UT_PWR_CALLS_NONE;
    utPwr_CbCnt         = UT_PWR_CALLS_NONE;
    utPwr_CbSupply      = PWR_SUPPLY_VDD;
    utPwr_CbState       = PWR_SUPPLY_STATE_VALID;
    utPwr_Isr           = PWR_NULL_PTR;
}


void tearDown( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief The module version is returned with the fields populated.
 * \par Expected results
 * Major / minor / patch fields hold the compiled-in version.
 */
void Ut_Pwr_Get_ModuleVersion_Call_ReturnsVersion( void )
{
    const pwr_ModuleVersion_t version = Pwr_Get_ModuleVersion();

    TEST_ASSERT_EQUAL( UT_PWR_VERSION_MAJOR, version.Major );
    TEST_ASSERT_EQUAL( UT_PWR_VERSION_MINOR, version.Minor );
    TEST_ASSERT_EQUAL( UT_PWR_VERSION_PATCH, version.Patch );
}

/* ========================== DEFAULT CONFIG ================================ */

/**
 * \brief A null configuration pointer is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned.
 */
void Ut_Pwr_Get_DefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_DefaultConfig( PWR_NULL_PTR ) );
}


/**
 * \brief The default configuration of a hardware in the reset state holds the reset values.
 * \par Expected results
 * Scale 3, the LDO regulator, nothing enabled, no pin armed, no callback, priority 0.
 */
void Ut_Pwr_Get_DefaultConfig_ResetState_HoldsResetValues( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    TEST_ASSERT_EQUAL( PWR_VOLTAGE_SCALE_3, pwrConfig.VoltageScale );
    TEST_ASSERT_EQUAL( PWR_REGULATOR_LDO, pwrConfig.RegulatorMode );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.StopFlashLowPower );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.BackupMonitor );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.PvdActive );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.AvdActive );
    TEST_ASSERT_EQUAL( PWR_PVD_LEVEL_0, pwrConfig.Pvd.Level );
    TEST_ASSERT_EQUAL( PWR_AVD_LEVEL_0, pwrConfig.Avd.Level );
    TEST_ASSERT_EQUAL( PWR_NULL_PTR, pwrConfig.PreEnterCallback );
    TEST_ASSERT_EQUAL( PWR_NULL_PTR, pwrConfig.PostExitCallback );
    TEST_ASSERT_EQUAL( PWR_NULL_PTR, pwrConfig.SupplyCallback );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.SupplyIsr );
    TEST_ASSERT_EQUAL( UT_PWR_REG_ZERO, pwrConfig.IrqPriority );

    for( uint32_t pinIdx = 0u; (uint32_t)PWR_WAKEUP_PIN_CNT > pinIdx; pinIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.WakeUpPinActive[pinIdx] );
    }
}


/**
 * \brief The default configuration reflects the registers that were written by other modules.
 * \par Expected results
 * Scale 1 (ACTVOS), the bypass, the flash low-power, the backup monitor, the PVD level 2 enabled, the AVD level 1
 * enabled and WKUP1 armed on the falling edge with the pull-up are returned.
 */
void Ut_Pwr_Get_DefaultConfig_RegistersSet_ReflectsHardware( void )
{
    pwr_Config_t pwrConfig;

    UtPwr_Preset_Hardware();
    PWR->VOSSR = ( ( UT_PWR_VOS_FIELD_SCALE_1 << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY );
    PWR->SCCR  = PWR_SCCR_BYPASS;
    PWR->PMCR  = PWR_PMCR_FLPS;
    PWR->BDCR  = PWR_BDCR_MONEN;
    PWR->VMCR  = ( PWR_VMCR_PVDEN | ( PWR_PVD_LEVEL_2 << PWR_VMCR_PLS_Pos ) |
                   PWR_VMCR_AVDEN | ( PWR_AVD_LEVEL_1 << PWR_VMCR_ALS_Pos ) );
    PWR->WUCR  = ( PWR_WUCR_WUPEN1 | PWR_WUCR_WUPP1 | ( UT_PWR_PULL_FIELD_UP << PWR_WUCR_WUPPUPD1_Pos ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_DefaultConfig( &pwrConfig ) );

    TEST_ASSERT_EQUAL( PWR_VOLTAGE_SCALE_1, pwrConfig.VoltageScale );
    TEST_ASSERT_EQUAL( PWR_REGULATOR_BYPASS, pwrConfig.RegulatorMode );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.StopFlashLowPower );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.BackupMonitor );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.PvdActive );
    TEST_ASSERT_EQUAL( PWR_PVD_LEVEL_2, pwrConfig.Pvd.Level );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.AvdActive );
    TEST_ASSERT_EQUAL( PWR_AVD_LEVEL_1, pwrConfig.Avd.Level );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_1] );
    TEST_ASSERT_EQUAL( PWR_WAKEUP_FALLING, pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Polarity );
    TEST_ASSERT_EQUAL( PWR_PULL_UP, pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Pull );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_2] );
}


/**
 * \brief The pull-down of a wake-up pin is read from the register.
 * \par Expected results
 * WKUP2 is armed on the rising edge with the pull-down.
 */
void Ut_Pwr_Get_DefaultConfig_PullDown_ReflectsHardware( void )
{
    pwr_Config_t pwrConfig;

    UtPwr_Preset_Hardware();
    PWR->WUCR = ( PWR_WUCR_WUPEN2 | ( UT_PWR_PULL_FIELD_DOWN << ( PWR_WUCR_WUPPUPD1_Pos + UT_PWR_PULL_FIELD_WIDTH ) ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_DefaultConfig( &pwrConfig ) );

    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_2] );
    TEST_ASSERT_EQUAL( PWR_WAKEUP_RISING, pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_2].Polarity );
    TEST_ASSERT_EQUAL( PWR_PULL_DOWN, pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_2].Pull );
}

/* ============================ INITIALIZATION ================================ */

/**
 * \brief A null configuration pointer is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the module stays not initialized.
 */
void Ut_Pwr_Init_NullConfig_ReturnsError( void )
{
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( PWR_NULL_PTR ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_VoltageScale( &voltageScale ) );
}


/**
 * \brief A voltage scale outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the flash low-power bit that differs from the configuration is not touched.
 */
void Ut_Pwr_Init_InvalidVoltageScale_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    PWR->PMCR                = PWR_PMCR_FLPS;
    pwrConfig.VoltageScale   = PWR_VOLTAGE_SCALE_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_PMCR_FLPS, PWR->PMCR );
}


/**
 * \brief A regulator mode outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the bypass bit is not written.
 */
void Ut_Pwr_Init_InvalidRegulatorMode_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.RegulatorMode = PWR_REGULATOR_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_SCCR_LDOEN, PWR->SCCR );
}


/**
 * \brief A function state outside of the enumeration is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned for the PVD enable, nothing is written.
 */
void Ut_Pwr_Init_InvalidFunctionState_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive = (pwr_FunctionState_t)( (uint32_t)PWR_FUNCTION_ACTIVE + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
}


/**
 * \brief A PVD threshold outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the PVD register is not written.
 */
void Ut_Pwr_Init_InvalidPvdLevel_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.Pvd.Level = PWR_PVD_LEVEL_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
}


/**
 * \brief An AVD threshold outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the AVD register is not written.
 */
void Ut_Pwr_Init_InvalidAvdLevel_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.Avd.Level = PWR_AVD_LEVEL_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
}


/**
 * \brief A supply event outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned for the event of the PVD and for the event of the AVD.
 */
void Ut_Pwr_Init_InvalidSupplyEvent_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.Pvd.Event = (pwr_SupplyEvent_t)( (uint32_t)PWR_SUPPLY_EVENT_BOTH + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );

    pwrConfig           = UtPwr_Get_Config();
    pwrConfig.Avd.Event = (pwr_SupplyEvent_t)( (uint32_t)PWR_SUPPLY_EVENT_BOTH + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
}


/**
 * \brief A wake-up pin polarity or pull outside of the list is rejected before any register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned for an invalid polarity and for an invalid pull, no pin is armed.
 */
void Ut_Pwr_Init_InvalidWakeUpPinConfig_ReturnsErrorWritesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_1]     = PWR_FUNCTION_ACTIVE;
    pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Polarity  = (pwr_WakeUpPolarity_t)( (uint32_t)PWR_WAKEUP_FALLING +
                                                                                 UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );

    pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Polarity  = PWR_WAKEUP_RISING;
    pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Pull      = (pwr_Pull_t)( (uint32_t)PWR_PULL_DOWN + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief The configuration read from the hardware is accepted and the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned, the voltage scale can be read through the module afterwards.
 */
void Ut_Pwr_Init_DefaultConfig_InitializesModule( void )
{
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &voltageScale ) );
    TEST_ASSERT_EQUAL( PWR_VOLTAGE_SCALE_3, voltageScale );
}


/**
 * \brief The voltage scale that is already applied is not written again.
 * \par Expected results
 * The initialization succeeds without a model that makes VCORE ready and VOSCR keeps its value (the Rcc module
 * owns the scale of the clock configuration).
 */
void Ut_Pwr_Init_ScaleAlreadyApplied_DoesNotWriteVoltageScale( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    PWR->VOSSR = ( ( UT_PWR_VOS_FIELD_SCALE_2 << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY );
    pwrConfig.VoltageScale = PWR_VOLTAGE_SCALE_2;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VOSCR );
}


/**
 * \brief A higher performance than the applied one is reached scale by scale.
 * \par Expected results
 * From scale 3 to scale 0: the scales 2, 1 and 0 are written in this order and the module is initialized.
 */
void Ut_Pwr_Init_ScaleRaise_WalksScaleByScale( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.VoltageScale = PWR_VOLTAGE_SCALE_0;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_Vos ) );

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_STEPS_THREE, utPwr_VosHistoryCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_2, utPwr_VosHistory[0] );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_1, utPwr_VosHistory[1] );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_0, utPwr_VosHistory[UT_PWR_STEP_THIRD] );
}


/**
 * \brief A voltage scale that does not become ready fails the initialization.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the module is deinitialized (the functions that need it return an error).
 */
void Ut_Pwr_Init_ScaleNotReady_ReturnsErrorAndDeinitializes( void )
{
    pwr_Config_t       pwrConfig    = UtPwr_Get_Config();
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    pwrConfig.VoltageScale = PWR_VOLTAGE_SCALE_2;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_VoltageScale( &voltageScale ) );
}


/**
 * \brief The bypass of the regulator is written.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned and PWR_SCCR.BYPASS is set.
 */
void Ut_Pwr_Init_RegulatorBypass_WritesBypassBit( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.RegulatorMode = PWR_REGULATOR_BYPASS;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_BITS_HIGH( PWR_SCCR_BYPASS, PWR->SCCR );
}


/**
 * \brief The bypass that the hardware does not accept (the register is locked) fails the initialization.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (the read-back of PWR_SCCR.BYPASS differs).
 */
void Ut_Pwr_Init_RegulatorBypassLocked_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.RegulatorMode = PWR_REGULATOR_BYPASS;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_BypassLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
}


/**
 * \brief The LDO / SMPS regulator that the package does not use cannot be selected.
 * \par Expected results
 * On a line with the SMPS: the package uses the LDO, the request for the SMPS is rejected and the bypass bit is
 * not written. On the other lines the test is ignored.
 */
void Ut_Pwr_Init_RegulatorNotOfPackage_ReturnsError( void )
{
#if defined(PWR_LINE_HAS_SMPS)
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.RegulatorMode = PWR_REGULATOR_SMPS;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_BITS_LOW( PWR_SCCR_BYPASS, PWR->SCCR );
#else
    TEST_IGNORE_MESSAGE( "Device line without the SMPS regulator" );
#endif
}


/**
 * \brief The flash low-power selection of Stop mode is written.
 * \par Expected results
 * PWR_PMCR.FLPS is set.
 */
void Ut_Pwr_Init_StopFlashLowPower_WritesBit( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.StopFlashLowPower = PWR_FUNCTION_ACTIVE;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_BITS_HIGH( PWR_PMCR_FLPS, PWR->PMCR );
}


/**
 * \brief The backup monitor is enabled with the write access to the backup domain taken for the write.
 * \par Expected results
 * PWR_BDCR.MONEN is set and the write protection of the backup domain is enabled again (DBP cleared).
 */
void Ut_Pwr_Init_BackupMonitor_TakesAndReleasesBackupAccess( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.BackupMonitor = PWR_FUNCTION_ACTIVE;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_BITS_HIGH( PWR_BDCR_MONEN, PWR->BDCR );
    TEST_ASSERT_BITS_LOW( PWR_DBPCR_DBP, PWR->DBPCR );
}


/**
 * \brief The backup monitor fails the initialization when the write access cannot be taken.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (DBP does not stay set), PWR_BDCR.MONEN is not written.
 */
void Ut_Pwr_Init_BackupMonitorAccessLocked_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.BackupMonitor = PWR_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_DbpLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_BITS_LOW( PWR_BDCR_MONEN, PWR->BDCR );
}


/**
 * \brief A wake-up pin is set to analog mode in the GPIO and armed in the PWR.
 * \par Expected results
 * The Gpio mock is called for PA0 (WKUP1) with the analog mode and no pull, the PWR_WUCR bits of WKUP1 (enable,
 * falling edge, pull-up) are written.
 */
void Ut_Pwr_Init_WakeUpPin_ConfiguresGpioAndArmsPin( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_1]    = PWR_FUNCTION_ACTIVE;
    pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Polarity = PWR_WAKEUP_FALLING;
    pwrConfig.WakeUpPin[PWR_WAKEUP_PIN_1].Pull     = PWR_PULL_UP;

    UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_0, GPIO_REQUEST_OK );

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_HEX32( ( PWR_WUCR_WUPEN1 | PWR_WUCR_WUPP1 | ( UT_PWR_PULL_FIELD_UP << PWR_WUCR_WUPPUPD1_Pos ) ),
                             PWR->WUCR );
}


/**
 * \brief A wake-up pin that cannot be set to analog mode is not armed.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_WUCR is not written.
 */
void Ut_Pwr_Init_WakeUpPinGpioFails_ReturnsErrorPinNotArmed( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_1] = PWR_FUNCTION_ACTIVE;

    UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_0, GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief The PVD is enabled with the threshold and its interrupt line is prepared.
 * \par Expected results
 * The Nvic handler and priority are set, the EXTI line is enabled for both edges, the Nvic interrupt is enabled,
 * PWR_VMCR holds PVDEN and the threshold, the handler is registered.
 */
void Ut_Pwr_Init_PvdActiveWithIsr_EnablesLineAndInterrupt( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive        = PWR_FUNCTION_ACTIVE;
    pwrConfig.Pvd.Level        = PWR_PVD_LEVEL_4;
    pwrConfig.SupplyCallback   = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr        = PWR_FUNCTION_ACTIVE;
    pwrConfig.IrqPriority      = UT_PWR_IRQ_PRIO;

    UtPwr_Expect_IsrInit( UT_PWR_IRQ_PRIO );
    UtPwr_Expect_LineEnable();

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_HEX32( ( PWR_VMCR_PVDEN | ( PWR_PVD_LEVEL_4 << PWR_VMCR_PLS_Pos ) ), PWR->VMCR );
    TEST_ASSERT_NOT_NULL( utPwr_Isr );
}


/**
 * \brief The PVD without the interrupt does not touch the EXTI line and the Nvic.
 * \par Expected results
 * PWR_VMCR holds PVDEN, no mock is called.
 */
void Ut_Pwr_Init_PvdActiveWithoutIsr_DoesNotUseInterrupt( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive      = PWR_FUNCTION_ACTIVE;
    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_BITS_HIGH( PWR_VMCR_PVDEN, PWR->VMCR );
}


/**
 * \brief The PVD_IN pin is set to analog mode when its voltage is compared.
 * \par Expected results
 * The Gpio mock is called for PB15, PWR_VMCR holds the PVD_IN selection (PLS = 7).
 */
void Ut_Pwr_Init_PvdPinLevel_ConfiguresPvdInPin( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive = PWR_FUNCTION_ACTIVE;
    pwrConfig.Pvd.Level = PWR_PVD_LEVEL_PIN;

    UtPwr_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_15, GPIO_REQUEST_OK );

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_HEX32( ( PWR_VMCR_PVDEN | PWR_VMCR_PLS_Msk ), PWR->VMCR );
}


/**
 * \brief The PVD_IN pin that cannot be set to analog mode fails the initialization.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the threshold is not written.
 */
void Ut_Pwr_Init_PvdPinLevelGpioFails_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.Pvd.Level = PWR_PVD_LEVEL_PIN;

    UtPwr_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_15, GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_BITS_LOW( PWR_VMCR_PLS_Msk, PWR->VMCR );
}


/**
 * \brief A PVD that is locked (the enable bit does not take the value) fails the initialization.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (the read-back of PWR_VMCR.PVDE differs).
 */
void Ut_Pwr_Init_PvdLocked_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive = PWR_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_PvdLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
}


/**
 * \brief An interrupt handler that the Nvic does not accept fails the initialization.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the EXTI line is not enabled.
 */
void Ut_Pwr_Init_IsrHandlerRejected_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive      = PWR_FUNCTION_ACTIVE;
    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr      = PWR_FUNCTION_ACTIVE;
    pwrConfig.IrqPriority    = UT_PWR_IRQ_PRIO;

    Nvic_Set_PeriphIrq_Handler_ExpectAnyArgsAndReturn( NVIC_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, UT_PWR_IRQ_PRIO, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Init( &pwrConfig ) );
    TEST_ASSERT_BITS_LOW( PWR_VMCR_PVDEN, PWR->VMCR );
}


/**
 * \brief The AVD is enabled with the threshold.
 * \par Expected results
 * PWR_VMCR holds AVDEN and the threshold.
 */
void Ut_Pwr_Init_AvdActive_EnablesAvd( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.AvdActive = PWR_FUNCTION_ACTIVE;
    pwrConfig.Avd.Level = PWR_AVD_LEVEL_3;

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_EQUAL_HEX32( ( PWR_VMCR_AVDEN | ( PWR_AVD_LEVEL_3 << PWR_VMCR_ALS_Pos ) ), PWR->VMCR );
}

/* ============================ DEINITIALIZATION ============================== */

/**
 * \brief The deinitialization of a module that was not initialized succeeds and writes nothing.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned, the registers are unchanged.
 */
void Ut_Pwr_Deinit_NotInitialized_ReturnsOkWritesNothing( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Deinit() );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief The deinitialization disables what the initialization enabled.
 * \par Expected results
 * The Nvic interrupt and the EXTI line are disabled, the PVD, the AVD and the wake-up pin are disabled (the other
 * fields stay), the module is not initialized and the callbacks are cleared.
 */
void Ut_Pwr_Deinit_Initialized_DisablesSupervisorsLineAndPins( void )
{
    pwr_Config_t       pwrConfig    = UtPwr_Get_Config();
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    pwrConfig.PvdActive                         = PWR_FUNCTION_ACTIVE;
    pwrConfig.AvdActive                         = PWR_FUNCTION_ACTIVE;
    pwrConfig.SupplyCallback                    = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr                         = PWR_FUNCTION_ACTIVE;
    pwrConfig.WakeUpPinActive[PWR_WAKEUP_PIN_2] = PWR_FUNCTION_ACTIVE;

    UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_2, GPIO_REQUEST_OK );
    UtPwr_Expect_IsrInit( UT_PWR_REG_ZERO );
    UtPwr_Expect_LineEnable();

    UtPwr_Init_Config( &pwrConfig );

    UtPwr_Expect_LineDisable();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Deinit() );

    TEST_ASSERT_BITS_LOW( ( PWR_VMCR_PVDEN | PWR_VMCR_AVDEN ), PWR->VMCR );
    TEST_ASSERT_BITS_LOW( PWR_WUCR_WUPEN2, PWR->WUCR );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_VoltageScale( &voltageScale ) );

    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_CbCnt );
}

/* ============================= VOLTAGE SCALING ============================== */

/**
 * \brief The voltage scale cannot be changed before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_VOSCR is not written.
 */
void Ut_Pwr_Set_VoltageScale_NotInitialized_ReturnsErrorWritesNothing( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_0 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VOSCR );
}


/**
 * \brief A voltage scale outside of the list is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_VOSCR is not written.
 */
void Ut_Pwr_Set_VoltageScale_InvalidScale_ReturnsErrorWritesNothing( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VOSCR );
}


/**
 * \brief The scale that is applied already is accepted without a write.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned without a model that makes VCORE ready, PWR_VOSCR is not written.
 */
void Ut_Pwr_Set_VoltageScale_AppliedScale_ReturnsOkWritesNothing( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_3 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VOSCR );
}


/**
 * \brief Every change between two different scales walks through the intermediate scales one at a time.
 * \par Expected results
 * For all 12 ordered pairs of the 4 scales: \ref PWR_REQUEST_OK, one write of PWR_VOSCR per scale of the walk, the
 * written values follow each other in the direction of the target and the last one is the target.
 */
void Ut_Pwr_Set_VoltageScale_AllScalePairs_WalkScaleByScale( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_Vos ) );

    for( uint32_t fromScale = 0u; (uint32_t)PWR_VOLTAGE_SCALE_CNT > fromScale; fromScale++ )
    {
        for( uint32_t toScale = 0u; (uint32_t)PWR_VOLTAGE_SCALE_CNT > toScale; toScale++ )
        {
            uint32_t stepScale = fromScale;
            uint32_t distance  = toScale - fromScale;

            if( fromScale > toScale )
            {
                distance = fromScale - toScale;
            }
            else
            {
                /* The distance is the difference of the indexes */
            }

            UtPwr_Set_Scale( (pwr_VoltageScale_t)fromScale );

            TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VoltageScale( (pwr_VoltageScale_t)toScale ) );
            TEST_ASSERT_EQUAL_UINT32( distance, utPwr_VosHistoryCnt );

            for( uint32_t stepIdx = 0u; distance > stepIdx; stepIdx++ )
            {
                if( toScale > fromScale )
                {
                    stepScale++;
                }
                else
                {
                    stepScale--;
                }

                TEST_ASSERT_EQUAL_HEX32( utPwr_ScaleField[stepScale], utPwr_VosHistory[stepIdx] );
            }

            TEST_ASSERT_EQUAL_HEX32( ( utPwr_ScaleField[toScale] << PWR_VOSCR_VOS_Pos ), PWR->VOSCR );
        }
    }
}


/**
 * \brief A voltage scale that does not become ready is reported as an error.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (bounded wait), the first step of the walk stays written.
 */
void Ut_Pwr_Set_VoltageScale_NotReady_ReturnsError( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_0 ) );
    TEST_ASSERT_EQUAL_HEX32( ( UT_PWR_VOS_FIELD_SCALE_2 << PWR_VOSCR_VOS_Pos ), PWR->VOSCR );
}


/**
 * \brief Raising the performance waits for VOSRDY, lowering it does not.
 * \par Expected results
 * With a model that never sets VOSRDY: the raise from scale 3 to scale 2 returns \ref PWR_REQUEST_ERROR, the
 * lowering from scale 2 to scale 3 returns \ref PWR_REQUEST_OK.
 */
void Ut_Pwr_Set_VoltageScale_VosrdyMissing_RaiseFailsLowerSucceeds( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_VosNoRaiseReady ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_2 ) );

    UtPwr_Set_Scale( PWR_VOLTAGE_SCALE_2 );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_3 ) );
}


/**
 * \brief The voltage scale cannot be read before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned.
 */
void Ut_Pwr_Get_VoltageScale_NotInitialized_ReturnsError( void )
{
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_VoltageScale( &voltageScale ) );
}


/**
 * \brief A null pointer is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned.
 */
void Ut_Pwr_Get_VoltageScale_NullPtr_ReturnsError( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_VoltageScale( PWR_NULL_PTR ) );
}


/**
 * \brief Every value of PWR_VOSSR.ACTVOS is mapped to its voltage scale.
 * \par Expected results
 * The fields 3, 2, 1 and 0 (scale 0 is the highest performance) read as the scales 0, 1, 2 and 3.
 */
void Ut_Pwr_Get_VoltageScale_ActvosFields_MapToScales( void )
{
    UtPwr_Init_Default();

    for( uint32_t scaleIdx = 0u; (uint32_t)PWR_VOLTAGE_SCALE_CNT > scaleIdx; scaleIdx++ )
    {
        pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

        PWR->VOSSR = ( utPwr_ScaleField[scaleIdx] << PWR_VOSSR_ACTVOS_Pos );

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &voltageScale ) );
        TEST_ASSERT_EQUAL( scaleIdx, voltageScale );
    }
}

/* ================================ REGULATOR ================================= */

/**
 * \brief The regulator mode cannot be changed or read before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by the setter and by the getter, the bypass bit is not written.
 */
void Ut_Pwr_RegulatorMode_NotInitialized_ReturnsError( void )
{
    pwr_RegulatorMode_t regulatorMode = PWR_REGULATOR_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_RegulatorMode( PWR_REGULATOR_BYPASS ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_BITS_LOW( PWR_SCCR_BYPASS, PWR->SCCR );
}


/**
 * \brief The regulator of the package is accepted, the bypass is written, an invalid mode is rejected.
 * \par Expected results
 * The LDO request returns \ref PWR_REQUEST_OK without a write, the bypass request sets PWR_SCCR.BYPASS and an invalid
 * mode returns \ref PWR_REQUEST_ERROR.
 */
void Ut_Pwr_Set_RegulatorMode_Requests_FollowPackageRules( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_RegulatorMode( PWR_REGULATOR_LDO ) );
    TEST_ASSERT_BITS_LOW( PWR_SCCR_BYPASS, PWR->SCCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_RegulatorMode( PWR_REGULATOR_CNT ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_RegulatorMode( PWR_REGULATOR_BYPASS ) );
    TEST_ASSERT_BITS_HIGH( PWR_SCCR_BYPASS, PWR->SCCR );
}


/**
 * \brief The SMPS cannot be selected on a package with the LDO.
 * \par Expected results
 * On a line with the SMPS: \ref PWR_REQUEST_ERROR for the SMPS request. On the other lines the test is ignored.
 */
void Ut_Pwr_Set_RegulatorMode_SmpsOnLdoPackage_ReturnsError( void )
{
#if defined(PWR_LINE_HAS_SMPS)
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_RegulatorMode( PWR_REGULATOR_SMPS ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the SMPS regulator" );
#endif
}


/**
 * \brief The bypass that the hardware does not accept is reported as an error.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned when PWR_SCCR.BYPASS does not read back as set.
 */
void Ut_Pwr_Set_RegulatorMode_BypassLocked_ReturnsError( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_BypassLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_RegulatorMode( PWR_REGULATOR_BYPASS ) );
}


/**
 * \brief The regulator is read from the status bits of PWR_SCCR.
 * \par Expected results
 * LDOEN reads as the LDO, BYPASS as the bypass (also with LDOEN), SMPSEN as the SMPS (lines with the SMPS), no bit
 * is an error, a null pointer is an error.
 */
void Ut_Pwr_Get_RegulatorMode_StatusBits_MapToModes( void )
{
    pwr_RegulatorMode_t regulatorMode = PWR_REGULATOR_CNT;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_EQUAL( PWR_REGULATOR_LDO, regulatorMode );

    PWR->SCCR = ( PWR_SCCR_LDOEN | PWR_SCCR_BYPASS );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_EQUAL( PWR_REGULATOR_BYPASS, regulatorMode );

#if defined(PWR_LINE_HAS_SMPS)
    PWR->SCCR = PWR_SCCR_SMPSEN;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_EQUAL( PWR_REGULATOR_SMPS, regulatorMode );
#endif

    PWR->SCCR = UT_PWR_REG_ZERO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_RegulatorMode( &regulatorMode ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_RegulatorMode( PWR_NULL_PTR ) );
}

/* ============================== SLEEP MODE ================================== */

/**
 * \brief The Sleep mode cannot be entered before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the core is not put to sleep (no Scb call).
 */
void Ut_Pwr_Set_SleepEnter_NotInitialized_ReturnsErrorNoSleep( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_SleepEnter( PWR_WAKEUP_INTERRUPT ) );
}


/**
 * \brief A wake-up instruction outside of the list is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the hooks are not called and the core is not put to sleep.
 */
void Ut_Pwr_Set_SleepEnter_InvalidInstruction_ReturnsErrorNoHooks( void )
{
    UtPwr_Init_Hooks();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR,
                       Pwr_Set_SleepEnter( (pwr_WakeUpEvent_t)( (uint32_t)PWR_WAKEUP_EVENT + UT_PWR_ITEM_OFFSET ) ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PreCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PostCnt );
}


/**
 * \brief Sleep with the wait for interrupt selects the normal sleep and calls the hooks around the instruction.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned, the Scb mock sees the normal sleep selection and the wait for interrupt, the
 * pre enter hook runs before the instruction and the post exit hook after it, each once.
 */
void Ut_Pwr_Set_SleepEnter_Interrupt_UsesWfiBetweenHooks( void )
{
    UtPwr_Init_Hooks();

    Scb_Set_DeepSleepInactive_Expect();
    Scb_Set_WaitForInterrupt_Expect();
    Scb_Set_WaitForInterrupt_AddCallback( UtPwr_ScbWfiStub );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SleepEnter( PWR_WAKEUP_INTERRUPT ) );

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_PreCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_PostCnt );
    TEST_ASSERT_TRUE( utPwr_PreSeq < utPwr_WfiSeq );
    TEST_ASSERT_TRUE( utPwr_WfiSeq < utPwr_PostSeq );
}


/**
 * \brief Sleep with the wait for event uses the wait for event instruction.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned, the Scb mock sees the wait for event (not the wait for interrupt).
 */
void Ut_Pwr_Set_SleepEnter_Event_UsesWfe( void )
{
    UtPwr_Init_Hooks();

    Scb_Set_DeepSleepInactive_Expect();
    Scb_Set_WaitForEvent_Expect();
    Scb_Set_WaitForEvent_AddCallback( UtPwr_ScbWfiStub );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SleepEnter( PWR_WAKEUP_EVENT ) );

    TEST_ASSERT_TRUE( utPwr_PreSeq < utPwr_WfiSeq );
    TEST_ASSERT_TRUE( utPwr_WfiSeq < utPwr_PostSeq );
}


/**
 * \brief Sleep without registered hooks works.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned, the instruction is executed.
 */
void Ut_Pwr_Set_SleepEnter_NoHooks_ReturnsOk( void )
{
    UtPwr_Init_Default();

    UtPwr_Expect_Sleep();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SleepEnter( PWR_WAKEUP_INTERRUPT ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_WfiSeq );
}

/* =============================== STOP MODE ================================== */

/**
 * \brief The Stop mode cannot be entered before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, PWR_PMCR is not written and the core is not put to sleep.
 */
void Ut_Pwr_Set_StopEnter_NotInitialized_ReturnsErrorNoEntry( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StopEnter( PWR_STOP_0 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
}


/**
 * \brief A Stop sub-mode outside of the list is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, the hooks are not called, PWR_PMCR is not written.
 */
void Ut_Pwr_Set_StopEnter_InvalidMode_ReturnsErrorNoHooks( void )
{
    UtPwr_Init_Hooks();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StopEnter( PWR_STOP_CNT ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PreCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
}


/**
 * \brief Every Stop sub-mode writes its SVOS level and enters the deep sleep between the hooks.
 * \par Expected results
 * For PWR_STOP_0 / 1 / 2: \ref PWR_REQUEST_OK, at the stop instruction PWR_PMCR holds SVOS3 / SVOS4 / SVOS5 with
 * LPMS cleared (Stop, not Standby), the deep sleep is selected before and cleared after the instruction, the hooks
 * run in the order pre enter, instruction, post exit.
 */
void Ut_Pwr_Set_StopEnter_EachMode_WritesSvosAndEntersDeepSleep( void )
{
    const uint32_t svosField[PWR_STOP_CNT] =
    {
        UT_PWR_SVOS_FIELD_3,
        UT_PWR_SVOS_FIELD_4,
        UT_PWR_SVOS_FIELD_5,
    };

    UtPwr_Init_Hooks();

    for( uint32_t modeIdx = 0u; (uint32_t)PWR_STOP_CNT > modeIdx; modeIdx++ )
    {
        UtPwr_Expect_Deep();

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_StopEnter( (pwr_StopMode_t)modeIdx ) );

        TEST_ASSERT_EQUAL_HEX32( ( svosField[modeIdx] << PWR_PMCR_SVOS_Pos ),
                                 ( utPwr_PmcrAtWfi & ( PWR_PMCR_SVOS_Msk | PWR_PMCR_LPMS_Msk ) ) );
        TEST_ASSERT_TRUE( utPwr_PreSeq < utPwr_WfiSeq );
        TEST_ASSERT_TRUE( utPwr_WfiSeq < utPwr_PostSeq );
    }

    TEST_ASSERT_EQUAL_UINT32( (uint32_t)PWR_STOP_CNT, utPwr_PreCnt );
    TEST_ASSERT_EQUAL_UINT32( (uint32_t)PWR_STOP_CNT, utPwr_PostCnt );
}


/**
 * \brief The Standby and Stop flags are cleared before the entry to Stop mode.
 * \par Expected results
 * PWR_PMCR.CSSF is written before the stop instruction.
 */
void Ut_Pwr_Set_StopEnter_Call_ClearsModeFlagsBeforeEntry( void )
{
    UtPwr_Init_Default();
    UtPwr_Expect_Deep();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_StopEnter( PWR_STOP_0 ) );
    TEST_ASSERT_BITS_HIGH( PWR_PMCR_CSSF, utPwr_PmcrAtWfi );
}


/**
 * \brief The Run voltage scale is restored after the wake-up (the hardware resets it to scale 3).
 * \par Expected results
 * Entry at scale 1: the stop instruction resets the scale to 3, the module walks back through scale 2 to scale 1
 * before the post exit hook, \ref PWR_REQUEST_OK is returned and scale 1 is applied.
 */
void Ut_Pwr_Set_StopEnter_ScaleOne_RestoresScaleAfterWakeUp( void )
{
    pwr_VoltageScale_t voltageScale = PWR_VOLTAGE_SCALE_CNT;

    UtPwr_Init_Hooks();
    UtPwr_Set_Scale( PWR_VOLTAGE_SCALE_1 );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_Vos ) );

    UtPwr_Expect_Deep();
    Scb_Set_WaitForInterrupt_AddCallback( UtPwr_ScbWfiStubVosReset );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_StopEnter( PWR_STOP_0 ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_VoltageScale( &voltageScale ) );
    TEST_ASSERT_EQUAL( PWR_VOLTAGE_SCALE_1, voltageScale );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_STEPS_THREE, utPwr_VosHistoryCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_3, utPwr_VosHistory[0] );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_2, utPwr_VosHistory[1] );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_VOS_FIELD_SCALE_1, utPwr_VosHistory[UT_PWR_STEP_THIRD] );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_PostCnt );
}


/**
 * \brief A Run scale that cannot be restored is reported but the post exit hook is still called.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (the scale does not become ready), the post exit hook ran once.
 */
void Ut_Pwr_Set_StopEnter_ScaleNotRestored_ReturnsErrorAndCallsPostHook( void )
{
    UtPwr_Init_Hooks();
    UtPwr_Set_Scale( PWR_VOLTAGE_SCALE_1 );

    UtPwr_Expect_Deep();
    Scb_Set_WaitForInterrupt_AddCallback( UtPwr_ScbWfiStubVosReset );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StopEnter( PWR_STOP_0 ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_PostCnt );
}


/**
 * \brief A Stop selection that the hardware does not keep leaves the MCU running.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, neither a hook nor the Scb mock is called.
 */
void Ut_Pwr_Set_StopEnter_SvosNotKept_ReturnsErrorNoEntry( void )
{
    UtPwr_Init_Hooks();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_SvosNotKept ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StopEnter( PWR_STOP_2 ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PreCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PostCnt );
}

/* ============================== STANDBY MODE ================================ */

/**
 * \brief The Standby mode cannot be entered before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, PWR_PMCR is not written and the core is not put to sleep.
 */
void Ut_Pwr_Set_StandbyEnter_NotInitialized_ReturnsErrorNoEntry( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StandbyEnter() );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
}


/**
 * \brief The Standby entry selects Standby, clears the flags and enters the deep sleep between the hooks.
 * \par Expected results
 * At the stop instruction PWR_PMCR holds LPMS and CSSF. The unit test host returns from the instruction, which is the
 * refused entry (a wake-up flag pending): \ref PWR_REQUEST_ERROR, LPMS is cleared again, the deep sleep is cleared
 * and the post exit hook ran after the instruction.
 */
void Ut_Pwr_Set_StandbyEnter_Call_EntersDeepSleepAndReportsRefusedEntry( void )
{
    UtPwr_Init_Hooks();
    UtPwr_Expect_Deep();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StandbyEnter() );

    TEST_ASSERT_BITS_HIGH( ( PWR_PMCR_LPMS | PWR_PMCR_CSSF ), utPwr_PmcrAtWfi );
    TEST_ASSERT_BITS_LOW( PWR_PMCR_LPMS, PWR->PMCR );
    TEST_ASSERT_TRUE( utPwr_PreSeq < utPwr_WfiSeq );
    TEST_ASSERT_TRUE( utPwr_WfiSeq < utPwr_PostSeq );
}


/**
 * \brief A Standby selection that the hardware does not keep leaves the MCU running.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, neither a hook nor the Scb mock is called.
 */
void Ut_Pwr_Set_StandbyEnter_LpmsNotKept_ReturnsErrorNoEntry( void )
{
    UtPwr_Init_Hooks();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_PmcrNotKept ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_StandbyEnter() );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_PreCnt );
}

/* ============================= SHUTDOWN MODE ================================ */

/**
 * \brief The STM32H5 has no Shutdown mode.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, initialized or not, the core is not put to sleep.
 */
void Ut_Pwr_Set_ShutdownEnter_Call_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_ShutdownEnter() );

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_ShutdownEnter() );
}

/* ================================= WAKE-UP ================================== */

/**
 * \brief A wake-up pin cannot be armed before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (setter and disarming), no Gpio call, PWR_WUCR is not written.
 */
void Ut_Pwr_Set_WakeUpPin_NotInitialized_ReturnsError( void )
{
    const pwr_WakeUpPinConfig_t pinConfig = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_NONE };

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_1, &pinConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPinInactive( PWR_WAKEUP_PIN_1 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief An invalid pin or an invalid configuration is rejected before the GPIO or the PWR is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR for the unused item, an item outside of the list, a null configuration and an invalid
 * polarity, no Gpio call, PWR_WUCR is not written.
 */
void Ut_Pwr_Set_WakeUpPin_InvalidArguments_ReturnsError( void )
{
    pwr_WakeUpPinConfig_t pinConfig = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_NONE };

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_UNUSED, &pinConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_CNT, &pinConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_1, PWR_NULL_PTR ) );

    pinConfig.Polarity = (pwr_WakeUpPolarity_t)( (uint32_t)PWR_WAKEUP_FALLING + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_1, &pinConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPinInactive( PWR_WAKEUP_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief Every polarity and pull combination is written to the fields of the pin.
 * \par Expected results
 * For WKUP2: the enable bit, the polarity bit (falling edge) and the pull field (none, up, down) in PWR_WUCR match the
 * request, the other pins are not touched.
 */
void Ut_Pwr_Set_WakeUpPin_PolarityAndPull_AreWrittenToFields( void )
{
    const uint32_t pullField[UT_PWR_PULL_COMBINATIONS] =
    {
        UT_PWR_PULL_FIELD_NONE,
        UT_PWR_PULL_FIELD_UP,
        UT_PWR_PULL_FIELD_DOWN,
    };

    UtPwr_Init_Default();

    for( uint32_t polarityIdx = 0u; UT_PWR_POLARITY_CNT > polarityIdx; polarityIdx++ )
    {
        for( uint32_t pullIdx = 0u; UT_PWR_PULL_COMBINATIONS > pullIdx; pullIdx++ )
        {
            pwr_WakeUpPinConfig_t pinConfig;
            const uint32_t        pullShift = ( PWR_WUCR_WUPPUPD1_Pos + UT_PWR_PULL_FIELD_WIDTH );
            uint32_t              expected  = ( PWR_WUCR_WUPEN2 | ( pullField[pullIdx] << pullShift ) );

            pinConfig.Polarity = (pwr_WakeUpPolarity_t)polarityIdx;
            pinConfig.Pull     = (pwr_Pull_t)pullIdx;

            if( PWR_WAKEUP_FALLING == pinConfig.Polarity )
            {
                expected |= PWR_WUCR_WUPP2;
            }
            else
            {
                /* The rising edge keeps the polarity bit cleared */
            }

            PWR->WUCR = UT_PWR_REG_ZERO;

            UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_2, GPIO_REQUEST_OK );

            TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_2, &pinConfig ) );
            TEST_ASSERT_EQUAL_HEX32( expected, PWR->WUCR );
        }
    }
}


/**
 * \brief Every wake-up pin of the device line is mapped to its fixed GPIO pin and its own register bits.
 * \par Expected results
 * For every item of the line: the Gpio mock is called for the pin of the CubeMX database (analog mode, no pull) and the
 * enable bit of the matching WUPENx is set in PWR_WUCR.
 */
void Ut_Pwr_Set_WakeUpPin_AllPins_ConfigureTheirGpioAndEnableBit( void )
{
    const pwr_WakeUpPinConfig_t pinConfig = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_NONE };

    UtPwr_Init_Default();

    for( uint32_t rowIdx = 0u; UT_PWR_PIN_ROW_CNT > rowIdx; rowIdx++ )
    {
        PWR->WUCR = UT_PWR_REG_ZERO;

        UtPwr_Expect_AnalogPin( utPwr_PinRows[rowIdx].Port, utPwr_PinRows[rowIdx].GpioPin, GPIO_REQUEST_OK );

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPin( utPwr_PinRows[rowIdx].Pin, &pinConfig ) );
        TEST_ASSERT_EQUAL_HEX32( utPwr_PinRows[rowIdx].EnableMask, PWR->WUCR );
    }
}


/**
 * \brief A pin that cannot be set to analog mode is not armed.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_WUCR is not written.
 */
void Ut_Pwr_Set_WakeUpPin_GpioFails_ReturnsErrorPinNotArmed( void )
{
    const pwr_WakeUpPinConfig_t pinConfig = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_NONE };

    UtPwr_Init_Default();

    UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_0, GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_1, &pinConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->WUCR );
}


/**
 * \brief Disarming a pin clears only its enable bit.
 * \par Expected results
 * After the arming of WKUP2 with the falling edge and the pull-down the disarming clears WUPEN2, the polarity and the
 * pull fields keep their values.
 */
void Ut_Pwr_Set_WakeUpPinInactive_ArmedPin_ClearsOnlyEnableBit( void )
{
    const pwr_WakeUpPinConfig_t pinConfig = { .Polarity = PWR_WAKEUP_FALLING, .Pull = PWR_PULL_DOWN };

    UtPwr_Init_Default();

    UtPwr_Expect_AnalogPin( GPIO_PORT_A, GPIO_PIN_ID_2, GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_2, &pinConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpPinInactive( PWR_WAKEUP_PIN_2 ) );

    TEST_ASSERT_EQUAL_HEX32( ( PWR_WUCR_WUPP2 | ( UT_PWR_PULL_FIELD_DOWN << ( PWR_WUCR_WUPPUPD1_Pos +
                                                                                 UT_PWR_PULL_FIELD_WIDTH ) ) ),
                             PWR->WUCR );
}


/**
 * \brief The wake-up source mask holds the wake-up flags of the pins and the Standby and Stop flags.
 * \par Expected results
 * Without a flag: no bit. With WUF1 and WUF2: the pin 1 and 2 bits. With SBF: the Standby bit, with STOPF: the
 * Stop bit.
 * The function works without the initialization of the module.
 */
void Ut_Pwr_Get_WakeUpSource_Flags_AreMappedToSourceBits( void )
{
    pwr_WakeUpSource_t wakeUpSource = PWR_WAKEUP_SOURCE_STOP;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_WAKEUP_SOURCE_NONE, wakeUpSource );

    PWR->WUSR = ( PWR_WUSR_WUF1 | PWR_WUSR_WUF2 );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_WAKEUP_SOURCE_PIN_1 | PWR_WAKEUP_SOURCE_PIN_2 ), wakeUpSource );

    PWR->WUSR = UT_PWR_REG_ZERO;
    PWR->PMSR = PWR_PMSR_SBF;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_WAKEUP_SOURCE_STANDBY, wakeUpSource );

    PWR->PMSR = ( PWR_PMSR_SBF | PWR_PMSR_STOPF );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_WAKEUP_SOURCE_STANDBY | PWR_WAKEUP_SOURCE_STOP ), wakeUpSource );
}


/**
 * \brief The wake-up flag of every pin of the device line has its own source bit.
 * \par Expected results
 * For every item of the line the WUFx bit reads as the source bit of the same pin.
 */
void Ut_Pwr_Get_WakeUpSource_EveryPinFlag_HasOwnBit( void )
{
    pwr_WakeUpSource_t wakeUpSource = PWR_WAKEUP_SOURCE_NONE;

    for( uint32_t rowIdx = 0u; UT_PWR_PIN_ROW_CNT > rowIdx; rowIdx++ )
    {
        PWR->WUSR = utPwr_PinRows[rowIdx].FlagMask;

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_WakeUpSource( &wakeUpSource ) );
        TEST_ASSERT_EQUAL_HEX32( utPwr_PinRows[rowIdx].SourceBit, wakeUpSource );
    }
}


/**
 * \brief A null pointer is rejected by the wake-up source and the Standby flag getters.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by both.
 */
void Ut_Pwr_Get_WakeUpSource_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_WakeUpSource( PWR_NULL_PTR ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_StandbyFlag( PWR_NULL_PTR ) );
}


/**
 * \brief The Standby flag follows PWR_PMSR.SBF, without the initialization of the module.
 * \par Expected results
 * Inactive without SBF, active with SBF (the Stop flag alone does not make it active).
 */
void Ut_Pwr_Get_StandbyFlag_Sbf_FollowsFlag( void )
{
    pwr_FunctionState_t standbyFlag = PWR_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_StandbyFlag( &standbyFlag ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, standbyFlag );

    PWR->PMSR = PWR_PMSR_STOPF;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_StandbyFlag( &standbyFlag ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, standbyFlag );

    PWR->PMSR = PWR_PMSR_SBF;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_StandbyFlag( &standbyFlag ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, standbyFlag );
}


/**
 * \brief The clearing of the flags writes the clear bits of the wake-up flags and of the Standby and Stop flags.
 * \par Expected results
 * \ref PWR_REQUEST_OK is returned without the initialization, PWR_WUSCR holds all CWUFx bits, PWR_PMCR holds CSSF
 * (the clearing itself is done by the hardware and not emulated).
 */
void Ut_Pwr_Set_WakeUpFlagClear_Call_WritesClearBits( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_WakeUpFlagClear() );

    TEST_ASSERT_EQUAL_HEX32( PWR_WUSCR_CWUF_Msk, PWR->WUSCR );
    TEST_ASSERT_BITS_HIGH( PWR_PMCR_CSSF, PWR->PMCR );
}

/* ================================ RETENTION ================================= */

/**
 * \brief The retention functions cannot be used before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by the SRAM setter, the SRAM getter and the I/O setter, PWR_PMCR and
 * PWR_IORETR are not written.
 */
void Ut_Pwr_Retention_NotInitialized_ReturnsError( void )
{
    pwr_FunctionState_t retention = PWR_FUNCTION_INACTIVE;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_SramRetention( PWR_SRAM_1, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SramRetention( PWR_SRAM_1, &retention ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_IoRetention( PWR_IO_GROUP_GENERAL, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->IORETR );
}


/**
 * \brief Every SRAM block of the device line has its own shut-off bit, the content is kept when the bit is cleared.
 * \par Expected results
 * For every item of the line (list taken from the reference manual of the line): the retention request clears the
 * shut-off bit, the loss request sets it, the getter follows, the other shut-off bits are not touched.
 */
void Ut_Pwr_Set_SramRetention_EveryBlock_WritesItsShutOffBit( void )
{
    UtPwr_Init_Default();

    for( uint32_t rowIdx = 0u; UT_PWR_SRAM_ROW_CNT > rowIdx; rowIdx++ )
    {
        pwr_FunctionState_t retention = PWR_FUNCTION_INACTIVE;

        PWR->PMCR = utPwr_SramRows[rowIdx].ShutOffMask;

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( utPwr_SramRows[rowIdx].Sram, &retention ) );
        TEST_ASSERT_EQUAL( PWR_FUNCTION_INACTIVE, retention );

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( utPwr_SramRows[rowIdx].Sram, PWR_FUNCTION_ACTIVE ) );
        TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( utPwr_SramRows[rowIdx].Sram, &retention ) );
        TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, retention );

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK,
                           Pwr_Set_SramRetention( utPwr_SramRows[rowIdx].Sram, PWR_FUNCTION_INACTIVE ) );
        TEST_ASSERT_EQUAL_HEX32( utPwr_SramRows[rowIdx].ShutOffMask, PWR->PMCR );
    }
}


/**
 * \brief The backup SRAM retention enables the backup regulator and waits until it is ready.
 * \par Expected results
 * With BRRDY set: \ref PWR_REQUEST_OK, PWR_BDCR.BREN set, the write protection of the backup domain is enabled again
 * (DBP cleared). The loss request clears BREN. The getter follows BREN.
 */
void Ut_Pwr_Set_SramRetention_BackupSram_EnablesBackupRegulator( void )
{
    pwr_FunctionState_t retention = PWR_FUNCTION_INACTIVE;

    UtPwr_Init_Default();

    PWR->BDSR = PWR_BDSR_BRRDY;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( PWR_SRAM_BACKUP, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_HIGH( PWR_BDCR_BREN, PWR->BDCR );
    TEST_ASSERT_BITS_LOW( PWR_DBPCR_DBP, PWR->DBPCR );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SramRetention( PWR_SRAM_BACKUP, &retention ) );
    TEST_ASSERT_EQUAL( PWR_FUNCTION_ACTIVE, retention );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_SramRetention( PWR_SRAM_BACKUP, PWR_FUNCTION_INACTIVE ) );
    TEST_ASSERT_BITS_LOW( PWR_BDCR_BREN, PWR->BDCR );
}


/**
 * \brief A backup regulator that does not become ready is reported as an error.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned (bounded wait for BRRDY), the write access to the backup domain is released.
 */
void Ut_Pwr_Set_SramRetention_BackupRegulatorNotReady_ReturnsError( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_SramRetention( PWR_SRAM_BACKUP, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_LOW( PWR_DBPCR_DBP, PWR->DBPCR );
}


/**
 * \brief The backup SRAM retention fails when the write access to the backup domain cannot be taken.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_BDCR.BREN is not written.
 */
void Ut_Pwr_Set_SramRetention_BackupAccessLocked_ReturnsError( void )
{
    UtPwr_Init_Default();

    PWR->BDSR = PWR_BDSR_BRRDY;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_DbpLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_SramRetention( PWR_SRAM_BACKUP, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_LOW( PWR_BDCR_BREN, PWR->BDCR );
}


/**
 * \brief An invalid block, state or pointer is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR for a block outside of the list, a state outside of the enumeration and a null pointer,
 * PWR_PMCR is not written.
 */
void Ut_Pwr_SramRetention_InvalidArguments_ReturnsError( void )
{
    pwr_FunctionState_t retention = PWR_FUNCTION_ACTIVE;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_SramRetention( PWR_SRAM_CNT, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR,
                       Pwr_Set_SramRetention( PWR_SRAM_1,
                                              (pwr_FunctionState_t)( (uint32_t)PWR_FUNCTION_ACTIVE +
                                                                     UT_PWR_ITEM_OFFSET ) ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SramRetention( PWR_SRAM_1, PWR_NULL_PTR ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SramRetention( PWR_SRAM_CNT, &retention ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->PMCR );
}


/**
 * \brief The I/O retention of both groups is written to its own bit.
 * \par Expected results
 * The general group sets IORETEN, the JTAG group sets JTAGIORETEN, the loss request clears the bit, an invalid group
 * or state is rejected without a write.
 */
void Ut_Pwr_Set_IoRetention_Groups_WriteTheirBits( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IoRetention( PWR_IO_GROUP_GENERAL, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_IORETR_IORETEN, PWR->IORETR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IoRetention( PWR_IO_GROUP_JTAG, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_IORETR_IORETEN | PWR_IORETR_JTAGIORETEN ), PWR->IORETR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IoRetention( PWR_IO_GROUP_GENERAL, PWR_FUNCTION_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_IORETR_JTAGIORETEN, PWR->IORETR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_IoRetention( PWR_IO_GROUP_CNT, PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR,
                       Pwr_Set_IoRetention( PWR_IO_GROUP_GENERAL,
                                            (pwr_FunctionState_t)( (uint32_t)PWR_FUNCTION_ACTIVE +
                                                                   UT_PWR_ITEM_OFFSET ) ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_IORETR_JTAGIORETEN, PWR->IORETR );
}

/* ============================== SUPERVISION ================================= */

/**
 * \brief The supervisor configuration functions cannot be used before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by every PVD / AVD / backup monitor function, PWR_VMCR is not written.
 */
void Ut_Pwr_Supervision_NotInitialized_ReturnsError( void )
{
    const pwr_PvdConfig_t pvdConfig = { .Level = PWR_PVD_LEVEL_1, .Event = PWR_SUPPLY_EVENT_DROP };
    const pwr_AvdConfig_t avdConfig = { .Level = PWR_AVD_LEVEL_1, .Event = PWR_SUPPLY_EVENT_DROP };

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdConfig( &pvdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdInactive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_AvdConfig( &avdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_AvdActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_AvdInactive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupMonitor( PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->BDCR );
}


/**
 * \brief Every PVD threshold is written to the PLS field.
 * \par Expected results
 * The levels 0 .. 6 write the values 0 .. 6, PVDE is not touched. The PVD_IN selection configures PB15 as an analog
 * input and writes 7.
 */
void Ut_Pwr_Set_PvdConfig_Levels_WriteLevelField( void )
{
    pwr_PvdConfig_t pvdConfig = { .Level = PWR_PVD_LEVEL_0, .Event = PWR_SUPPLY_EVENT_DROP };

    UtPwr_Init_Default();

    for( uint32_t levelIdx = 0u; (uint32_t)PWR_PVD_LEVEL_PIN > levelIdx; levelIdx++ )
    {
        pvdConfig.Level = (pwr_PvdLevel_t)levelIdx;

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdConfig( &pvdConfig ) );
        TEST_ASSERT_EQUAL_HEX32( ( levelIdx << PWR_VMCR_PLS_Pos ), PWR->VMCR );
    }

    pvdConfig.Level = PWR_PVD_LEVEL_PIN;

    UtPwr_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_15, GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdConfig( &pvdConfig ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_VMCR_PLS_Msk, PWR->VMCR );
}


/**
 * \brief An invalid PVD configuration is rejected before a register is written.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR for a null pointer, a threshold outside of the list and an event outside of the list, a pin
 * that cannot be set to analog mode leaves the threshold unchanged.
 */
void Ut_Pwr_Set_PvdConfig_InvalidConfig_ReturnsErrorWritesNothing( void )
{
    pwr_PvdConfig_t pvdConfig = { .Level = PWR_PVD_LEVEL_PIN, .Event = PWR_SUPPLY_EVENT_DROP };

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdConfig( PWR_NULL_PTR ) );

    pvdConfig.Level = PWR_PVD_LEVEL_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdConfig( &pvdConfig ) );

    pvdConfig.Level = PWR_PVD_LEVEL_1;
    pvdConfig.Event = (pwr_SupplyEvent_t)( (uint32_t)PWR_SUPPLY_EVENT_BOTH + UT_PWR_ITEM_OFFSET );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdConfig( &pvdConfig ) );

    pvdConfig.Level = PWR_PVD_LEVEL_PIN;
    pvdConfig.Event = PWR_SUPPLY_EVENT_DROP;

    UtPwr_Expect_AnalogPin( GPIO_PORT_B, GPIO_PIN_ID_15, GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdConfig( &pvdConfig ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->VMCR );
}


/**
 * \brief The PVD is enabled and disabled without touching the threshold.
 * \par Expected results
 * PVDE is set and cleared, the threshold stays. A PVD that is locked (the bit does not take the value) returns
 * \ref PWR_REQUEST_ERROR.
 */
void Ut_Pwr_Set_PvdActive_EnableDisable_WritesPvdeOnly( void )
{
    const pwr_PvdConfig_t pvdConfig = { .Level = PWR_PVD_LEVEL_3, .Event = PWR_SUPPLY_EVENT_DROP };

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdConfig( &pvdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdActive() );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_VMCR_PVDEN | ( PWR_PVD_LEVEL_3 << PWR_VMCR_PLS_Pos ) ), PWR->VMCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdInactive() );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_PVD_LEVEL_3 << PWR_VMCR_PLS_Pos ), PWR->VMCR );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_PvdLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdActive() );
}


/**
 * \brief The AVD configuration, the enabling and the disabling are written to their fields.
 * \par Expected results
 * The levels 0 .. 3 write the ALS field, AVDEN is set and cleared, the threshold stays, an invalid configuration
 * is rejected.
 */
void Ut_Pwr_Set_AvdConfig_LevelsAndEnable_WriteFields( void )
{
    pwr_AvdConfig_t avdConfig = { .Level = PWR_AVD_LEVEL_0, .Event = PWR_SUPPLY_EVENT_DROP };

    UtPwr_Init_Default();

    for( uint32_t levelIdx = 0u; (uint32_t)PWR_AVD_LEVEL_CNT > levelIdx; levelIdx++ )
    {
        avdConfig.Level = (pwr_AvdLevel_t)levelIdx;

        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdConfig( &avdConfig ) );
        TEST_ASSERT_EQUAL_HEX32( ( levelIdx << PWR_VMCR_ALS_Pos ), PWR->VMCR );
    }

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdActive() );
    TEST_ASSERT_BITS_HIGH( PWR_VMCR_AVDEN, PWR->VMCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdInactive() );
    TEST_ASSERT_BITS_LOW( PWR_VMCR_AVDEN, PWR->VMCR );

    avdConfig.Level = PWR_AVD_LEVEL_CNT;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_AvdConfig( &avdConfig ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_AvdConfig( PWR_NULL_PTR ) );
}


/**
 * \brief The interrupt line follows the supervisors: enabled with the first one, disabled with the last one.
 * \par Expected results
 * With the callback and the interrupt: the PVD enabling enables the EXTI line (both edges) and the Nvic interrupt,
 * the AVD enabling does not touch them again, the PVD disabling keeps them, the AVD disabling disables the Nvic
 * interrupt and the EXTI line.
 */
void Ut_Pwr_SupervisorLine_PvdAndAvd_EnabledByFirstDisabledByLast( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr      = PWR_FUNCTION_ACTIVE;

    UtPwr_Expect_IsrInit( UT_PWR_REG_ZERO );
    UtPwr_Init_Config( &pwrConfig );

    UtPwr_Expect_LineEnable();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_PvdInactive() );

    UtPwr_Expect_LineDisable();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_AvdInactive() );
}


/**
 * \brief A line that the EXTI or the Nvic cannot enable is reported as an error.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by the PVD enabling when the EXTI mock reports an error.
 */
void Ut_Pwr_Set_PvdActive_LineRejected_ReturnsError( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr      = PWR_FUNCTION_ACTIVE;

    UtPwr_Expect_IsrInit( UT_PWR_REG_ZERO );
    UtPwr_Init_Config( &pwrConfig );

    Exti_Set_LineActive_ExpectAnyArgsAndReturn( EXTI_REQUEST_ERROR );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_PvdActive() );
}


/**
 * \brief The backup monitor is written with the write access to the backup domain taken for the write.
 * \par Expected results
 * MONEN is set and cleared, DBP is cleared after each write, an invalid state is rejected, a write access that
 * cannot be taken returns \ref PWR_REQUEST_ERROR.
 */
void Ut_Pwr_Set_BackupMonitor_EnableDisable_TakesAndReleasesAccess( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupMonitor( PWR_FUNCTION_ACTIVE ) );
    TEST_ASSERT_BITS_HIGH( PWR_BDCR_MONEN, PWR->BDCR );
    TEST_ASSERT_BITS_LOW( PWR_DBPCR_DBP, PWR->DBPCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupMonitor( PWR_FUNCTION_INACTIVE ) );
    TEST_ASSERT_BITS_LOW( PWR_BDCR_MONEN, PWR->BDCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR,
                       Pwr_Set_BackupMonitor( (pwr_FunctionState_t)( (uint32_t)PWR_FUNCTION_ACTIVE +
                                                                     UT_PWR_ITEM_OFFSET ) ) );

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_DbpLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupMonitor( PWR_FUNCTION_ACTIVE ) );
}


/**
 * \brief The supply state cannot be read before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned.
 */
void Ut_Pwr_Get_SupplyState_NotInitialized_ReturnsError( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_VALID;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_IO2, &supplyState ) );
}


/**
 * \brief An invalid supply or pointer is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR for a supply outside of the list and for a null pointer.
 */
void Ut_Pwr_Get_SupplyState_InvalidArguments_ReturnsError( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_VALID;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_CNT, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_IO2, PWR_NULL_PTR ) );
}


/**
 * \brief The VDD and the VDDA supply states follow the output flags of the detectors, which have to be enabled.
 * \par Expected results
 * With the detector disabled: \ref PWR_REQUEST_ERROR. Enabled: PVDO / AVDO clear is valid, set is not valid.
 */
void Ut_Pwr_Get_SupplyState_PvdAndAvd_FollowOutputFlags( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_VALID;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );

    PWR->VMCR = ( PWR_VMCR_PVDEN | PWR_VMCR_AVDEN );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->VMSR = PWR_VMSR_PVDO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->VMSR = PWR_VMSR_AVDO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_VDD, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_ANALOG, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
}


/**
 * \brief The VDDIO2 supply state follows its ready flag, the monitor works always.
 * \par Expected results
 * VDDIO2RDY set is valid, clear is not valid, no enabling is needed.
 */
void Ut_Pwr_Get_SupplyState_Io2_FollowsReadyFlag( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_NOT_VALID;

    UtPwr_Init_Default();

    PWR->VMSR = PWR_VMSR_VDDIO2RDY;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_IO2, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->VMSR = UT_PWR_REG_ZERO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_IO2, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
}


/**
 * \brief The VDDUSB supply state follows its ready flag once its monitor is enabled.
 * \par Expected results
 * On a line with the VDDUSB supply: monitor disabled is an error, enabled: USB33RDY set is valid, clear is not valid.
 * On the other lines the test is ignored.
 */
void Ut_Pwr_Get_SupplyState_Usb_FollowsReadyFlag( void )
{
#if defined(PWR_LINE_HAS_USB_SUPPLY)
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_NOT_VALID;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_USB, &supplyState ) );

    PWR->USBSCR = PWR_USBSCR_USB33DEN;
    PWR->VMSR   = PWR_VMSR_USB33RDY;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_USB, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->VMSR = UT_PWR_REG_ZERO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_USB, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
#else
    TEST_IGNORE_MESSAGE( "Device line without the VDDUSB supply" );
#endif
}


/**
 * \brief The VBAT and temperature states follow the threshold flags once the backup monitor is enabled.
 * \par Expected results
 * Monitor disabled: \ref PWR_REQUEST_ERROR for both. Enabled: no flag is valid, VBATL or VBATH makes the VBAT
 * supply not valid only, TEMPL or TEMPH makes the temperature not valid only.
 */
void Ut_Pwr_Get_SupplyState_BackupAndTemperature_FollowThresholdFlags( void )
{
    pwr_SupplyState_t supplyState = PWR_SUPPLY_STATE_VALID;

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );

    PWR->BDCR = PWR_BDCR_MONEN;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->BDSR = PWR_BDSR_VBATL;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );

    PWR->BDSR = PWR_BDSR_VBATH;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );

    PWR->BDSR = PWR_BDSR_TEMPL;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_BACKUP, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, supplyState );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );

    PWR->BDSR = PWR_BDSR_TEMPH;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_SupplyState( PWR_SUPPLY_TEMPERATURE, &supplyState ) );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, supplyState );
}

/* ============================ SUPERVISOR EVENTS ============================= */

/**
 * \brief The interrupt reports only the drop of the supply when the drop event is configured.
 * \par Expected results
 * The handler clears the pending flag of the EXTI line at every call. PVDO set (drop): the callback is called once with
 * VDD not valid. PVDO cleared (rise): the callback is not called.
 */
void Ut_Pwr_IsrHandler_PvdDropEvent_ReportsOnlyDrop( void )
{
    UtPwr_Init_Supervised( PWR_SUPPLY_EVENT_DROP, PWR_SUPPLY_EVENT_DROP );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = PWR_VMSR_PVDO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_VDD, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, utPwr_CbState );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = UT_PWR_REG_ZERO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
}


/**
 * \brief The interrupt reports only the rise of the supply when the rise event is configured.
 * \par Expected results
 * VDD is below the threshold at the enabling. PVDO cleared (rise): the callback is called once with VDD valid. A drop
 * afterwards is not reported.
 */
void Ut_Pwr_IsrHandler_PvdRiseEvent_ReportsOnlyRise( void )
{
    PWR->VMSR = PWR_VMSR_PVDO;

    UtPwr_Init_Supervised( PWR_SUPPLY_EVENT_RISE, PWR_SUPPLY_EVENT_RISE );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = UT_PWR_REG_ZERO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_VDD, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, utPwr_CbState );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = PWR_VMSR_PVDO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
}


/**
 * \brief The interrupt reports both changes of the supplies when both events are configured.
 * \par Expected results
 * PVDO set: VDD not valid, PVDO cleared: VDD valid, AVDO set: VDDA not valid, AVDO cleared: VDDA valid, four calls.
 * A call without a change does not call the callback.
 */
void Ut_Pwr_IsrHandler_BothEvents_ReportBothChangesOfBothSupplies( void )
{
    UtPwr_Init_Supervised( PWR_SUPPLY_EVENT_BOTH, PWR_SUPPLY_EVENT_BOTH );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = PWR_VMSR_PVDO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL( PWR_SUPPLY_VDD, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, utPwr_CbState );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = UT_PWR_REG_ZERO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL( PWR_SUPPLY_VDD, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, utPwr_CbState );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = PWR_VMSR_AVDO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL( PWR_SUPPLY_ANALOG, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, utPwr_CbState );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = UT_PWR_REG_ZERO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL( PWR_SUPPLY_ANALOG, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_VALID, utPwr_CbState );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_FOUR, utPwr_CbCnt );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_FOUR, utPwr_CbCnt );
}


/**
 * \brief The AVD drop event is independent of the PVD event.
 * \par Expected results
 * PVD with the rise event, AVD with the drop event: AVDO set is reported (VDDA not valid), the later PVDO change from
 * valid to not valid is not reported.
 */
void Ut_Pwr_IsrHandler_AvdDropAndPvdRise_AreIndependent( void )
{
    UtPwr_Init_Supervised( PWR_SUPPLY_EVENT_RISE, PWR_SUPPLY_EVENT_DROP );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = PWR_VMSR_AVDO;
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_ANALOG, utPwr_CbSupply );

    Exti_Set_LinePendingClear_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
    PWR->VMSR = ( PWR_VMSR_AVDO | PWR_VMSR_PVDO );
    utPwr_Isr();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
}


/**
 * \brief The polled task reports the changes when the events are not taken from the interrupt.
 * \par Expected results
 * Without the interrupt (no Nvic / EXTI call): the task without a change does not call the callback, after PVDO is
 * set it calls it once (VDD not valid) and not again for the same state.
 */
void Ut_Pwr_Task_PolledSupervisor_ReportsChangeOnce( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive      = PWR_FUNCTION_ACTIVE;
    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;
    pwrConfig.Pvd.Event      = PWR_SUPPLY_EVENT_BOTH;

    UtPwr_Init_Config( &pwrConfig );

    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_CbCnt );

    PWR->VMSR = PWR_VMSR_PVDO;

    Pwr_Task();
    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_ONCE, utPwr_CbCnt );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_VDD, utPwr_CbSupply );
    TEST_ASSERT_EQUAL( PWR_SUPPLY_STATE_NOT_VALID, utPwr_CbState );
}


/**
 * \brief The task does nothing before the initialization and without the registered callback.
 * \par Expected results
 * No callback call in both cases even when PVDO changed.
 */
void Ut_Pwr_Task_NotPolled_DoesNothing( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    PWR->VMSR = PWR_VMSR_PVDO;

    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_CbCnt );

    pwrConfig.PvdActive = PWR_FUNCTION_ACTIVE;

    UtPwr_Init_Config( &pwrConfig );
    PWR->VMSR = UT_PWR_REG_ZERO;

    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_CbCnt );
}


/**
 * \brief The task does nothing in the interrupt mode.
 * \par Expected results
 * With the callback and the interrupt, a change of PVDO is not reported by the task.
 */
void Ut_Pwr_Task_InterruptMode_DoesNothing( void )
{
    UtPwr_Init_Supervised( PWR_SUPPLY_EVENT_BOTH, PWR_SUPPLY_EVENT_BOTH );

    PWR->VMSR = PWR_VMSR_PVDO;

    Pwr_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_PWR_CALLS_NONE, utPwr_CbCnt );
}

/* ============================= BACKUP DOMAIN ================================ */

/**
 * \brief The first holder sets DBP, the other holders only count, the last release clears DBP.
 * \par Expected results
 * Three takes: DBP set after the first. Two releases: DBP still set. The third release clears DBP. No
 * initialization of the module is needed.
 */
void Ut_Pwr_BackupAccess_ThreeHolders_ClearedByLastRelease( void )
{
    for( uint32_t holderIdx = 0u; UT_PWR_HOLDER_CNT > holderIdx; holderIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessActive() );
        TEST_ASSERT_BITS_HIGH( PWR_DBPCR_DBP, PWR->DBPCR );
    }

    for( uint32_t holderIdx = 0u; ( UT_PWR_HOLDER_CNT - UT_PWR_ITEM_OFFSET ) > holderIdx; holderIdx++ )
    {
        TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessInactive() );
        TEST_ASSERT_BITS_HIGH( PWR_DBPCR_DBP, PWR->DBPCR );
    }

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessInactive() );
    TEST_ASSERT_BITS_LOW( PWR_DBPCR_DBP, PWR->DBPCR );
}


/**
 * \brief A release without a take is rejected.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and DBP, set by another module, stays set.
 */
void Ut_Pwr_BackupAccess_UnbalancedRelease_ReturnsError( void )
{
    PWR->DBPCR = PWR_DBPCR_DBP;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupAccessInactive() );
    TEST_ASSERT_BITS_HIGH( PWR_DBPCR_DBP, PWR->DBPCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessInactive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupAccessInactive() );
}


/**
 * \brief DBP that was set by another module is not cleared by the last release.
 * \par Expected results
 * DBP set before the first take: take and release return \ref PWR_REQUEST_OK and DBP stays set (the Rcc module
 * owns it).
 */
void Ut_Pwr_BackupAccess_SetByOtherModule_IsNotCleared( void )
{
    PWR->DBPCR = PWR_DBPCR_DBP;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_BackupAccessInactive() );
    TEST_ASSERT_BITS_HIGH( PWR_DBPCR_DBP, PWR->DBPCR );
}


/**
 * \brief A DBP that does not take the value fails the take and does not count the holder.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR for the take, then \ref PWR_REQUEST_ERROR for the release (nothing is held).
 */
void Ut_Pwr_BackupAccess_DbpLocked_TakeFailsAndIsNotCounted( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtPwr_HwModel_DbpLocked ) );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupAccessActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_BackupAccessInactive() );
}


/* ================================ SUPPLIES ================================== */

/**
 * \brief The VBAT charging cannot be selected before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned and PWR_BDCR is not written.
 */
void Ut_Pwr_Set_VbatCharging_NotInitialized_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_VbatCharging( PWR_VBAT_CHARGING_5K ) );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->BDCR );
}


/**
 * \brief The VBAT charging selections write the resistor and the enable bit.
 * \par Expected results
 * 5 kOhm: VBE set, VBRS clear. 1.5 kOhm: VBE and VBRS set. Off: VBE clear (VBRS stays). An invalid selection is
 * rejected without a write.
 */
void Ut_Pwr_Set_VbatCharging_Selections_WriteResistorAndEnable( void )
{
    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VbatCharging( PWR_VBAT_CHARGING_5K ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_BDCR_VBE, PWR->BDCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VbatCharging( PWR_VBAT_CHARGING_1K5 ) );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_BDCR_VBE | PWR_BDCR_VBRS ), PWR->BDCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_VbatCharging( PWR_VBAT_CHARGING_OFF ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_BDCR_VBRS, PWR->BDCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR,
                       Pwr_Set_VbatCharging( (pwr_VbatCharging_t)( (uint32_t)PWR_VBAT_CHARGING_1K5 +
                                                                   UT_PWR_ITEM_OFFSET ) ) );
    TEST_ASSERT_EQUAL_HEX32( PWR_BDCR_VBRS, PWR->BDCR );
}


/**
 * \brief The VDDUSB supply cannot be changed before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by the activation and the deactivation.
 */
void Ut_Pwr_UsbSupply_NotInitialized_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_UsbSupplyActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_UsbSupplyInactive() );
}


/**
 * \brief The VDDUSB supply is declared valid when its monitor reports it ready.
 * \par Expected results
 * On a line with the VDDUSB supply: with USB33RDY set the monitor (USB33DEN) and the valid bit (USB33SV) are set, the
 * deactivation clears both. Without USB33RDY the activation fails and the monitor is disabled again. On a line
 * without the supply both requests return \ref PWR_REQUEST_ERROR.
 */
void Ut_Pwr_Set_UsbSupplyActive_ReadyMonitor_SetsValidBit( void )
{
    UtPwr_Init_Default();

#if defined(PWR_LINE_HAS_USB_SUPPLY)
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_UsbSupplyActive() );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->USBSCR );

    PWR->VMSR = PWR_VMSR_USB33RDY;

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_UsbSupplyActive() );
    TEST_ASSERT_EQUAL_HEX32( ( PWR_USBSCR_USB33DEN | PWR_USBSCR_USB33SV ), PWR->USBSCR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_UsbSupplyInactive() );
    TEST_ASSERT_EQUAL_HEX32( UT_PWR_REG_ZERO, PWR->USBSCR );
#else
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_UsbSupplyActive() );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_UsbSupplyInactive() );
#endif
}


/**
 * \brief The VDDIO2 supply has no validity bit on the STM32H5.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned, initialized or not.
 */
void Ut_Pwr_Set_Io2SupplyActive_Call_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_Io2SupplyActive() );

    UtPwr_Init_Default();

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_Io2SupplyActive() );
}

/* ============================= INTERRUPT PRIORITY =========================== */

/**
 * \brief The interrupt priority cannot be used before the module is initialized.
 * \par Expected results
 * \ref PWR_REQUEST_ERROR is returned by the setter and by the getter, no Nvic call.
 */
void Ut_Pwr_IrqPriority_NotInitialized_ReturnsError( void )
{
    pwr_IrqPrio_t irqPriority = UT_PWR_REG_ZERO;

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_IrqPriority( UT_PWR_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_IrqPriority( &irqPriority ) );
}


/**
 * \brief The priority is passed to the Nvic module and its result is returned.
 * \par Expected results
 * The Nvic mock is called for the PVD / AVD interrupt, \ref PWR_REQUEST_OK for the accepted priority and
 * \ref PWR_REQUEST_ERROR for the rejected one.
 */
void Ut_Pwr_Set_IrqPriority_Nvic_PassesPriorityAndResult( void )
{
    UtPwr_Init_Default();

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, UT_PWR_IRQ_PRIO, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Set_IrqPriority( UT_PWR_IRQ_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, UT_PWR_IRQ_PRIO_OTHER, NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Set_IrqPriority( UT_PWR_IRQ_PRIO_OTHER ) );
}


/**
 * \brief The priority is read from the Nvic module.
 * \par Expected results
 * The value returned by the Nvic mock is returned, an Nvic error and a null pointer return \ref PWR_REQUEST_ERROR.
 */
void Ut_Pwr_Get_IrqPriority_Nvic_ReturnsPriority( void )
{
    pwr_IrqPrio_t  irqPriority = UT_PWR_REG_ZERO;
    nvic_IrqPrio_t nvicPrio    = UT_PWR_IRQ_PRIO;

    UtPwr_Init_Default();

    Nvic_Get_PeriphIrq_Prio_ExpectAnyArgsAndReturn( NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &nvicPrio );

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_IrqPriority( &irqPriority ) );
    TEST_ASSERT_EQUAL_UINT32( UT_PWR_IRQ_PRIO, irqPriority );

    Nvic_Get_PeriphIrq_Prio_ExpectAnyArgsAndReturn( NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_IrqPriority( &irqPriority ) );
    TEST_ASSERT_EQUAL( PWR_REQUEST_ERROR, Pwr_Get_IrqPriority( PWR_NULL_PTR ) );
}

/* ============================ LOCAL FUNCTIONS =============================== */

/**
 * \brief Expects the analog configuration of a pin through \ref Gpio_Init (port clock, analog mode, no pull).
 *
 * \param gpioPort  [in]: Expected GPIO port
 * \param gpioPin   [in]: Expected GPIO pin
 * \param gpioState [in]: Result the mock returns
 */
static void UtPwr_Expect_AnalogPin( gpio_PortId_t gpioPort, gpio_PinId_t gpioPin, gpio_RequestState_t gpioState )
{
    TEST_ASSERT_TRUE( UT_PWR_GPIO_EXPECT_CNT > utPwr_ExpectedGpioCnt );

    gpio_Config_t * const expected = &utPwr_ExpectedGpio[ utPwr_ExpectedGpioCnt ];

    utPwr_ExpectedGpioCnt++;

    *expected                = (gpio_Config_t){ 0 };
    expected->PortId         = gpioPort;
    expected->PinId          = gpioPin;
    expected->PinMode        = GPIO_PIN_MODE_ANALOG;
    expected->PinPull        = GPIO_PIN_PULL_NONE;
    expected->PinSpeed       = GPIO_PIN_SPEED_LOW;
    expected->PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expected->PinAltFunction = GPIO_ALT_FUNC_0;
    expected->PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( expected, gpioState );
}


/**
 * \brief Presets the status the hardware has after the reset: the LDO regulator, the scale 3 applied and ready.
 */
static void UtPwr_Preset_Hardware( void )
{
    PWR->SCCR  = PWR_SCCR_LDOEN;
    PWR->VOSSR = ( PWR_VOSSR_ACTVOSRDY | PWR_VOSSR_VOSRDY );
}


/**
 * \brief Presets the hardware and returns the configuration that holds its current state.
 *
 * \return Default configuration of the module
 */
static pwr_Config_t UtPwr_Get_Config( void )
{
    pwr_Config_t pwrConfig;

    UtPwr_Preset_Hardware();

    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Get_DefaultConfig( &pwrConfig ) );

    return ( pwrConfig );
}


/**
 * \brief Initializes the module with a configuration and checks that it succeeded.
 *
 * \param pwrConfig [in]: Configuration
 */
static void UtPwr_Init_Config( const pwr_Config_t * const pwrConfig )
{
    TEST_ASSERT_EQUAL( PWR_REQUEST_OK, Pwr_Init( pwrConfig ) );
}


/**
 * \brief Initializes the module with the default configuration of the preset hardware (no hooks).
 */
static void UtPwr_Init_Default( void )
{
    const pwr_Config_t pwrConfig = UtPwr_Get_Config();

    UtPwr_Init_Config( &pwrConfig );
}


/**
 * \brief Initializes the module with the default configuration and the two hooks of the tests.
 */
static void UtPwr_Init_Hooks( void )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PreEnterCallback = UtPwr_HookPre;
    pwrConfig.PostExitCallback = UtPwr_HookPost;

    UtPwr_Init_Config( &pwrConfig );
}


/**
 * \brief Initializes the module with both supervisors enabled and their events taken from the interrupt.
 *
 * The Nvic and EXTI expectations of the initialization are set here, the handler of the interrupt is stored in
 * \ref utPwr_Isr.
 *
 * \param pvdEvent [in]: Event of the PVD
 * \param avdEvent [in]: Event of the AVD
 */
static void UtPwr_Init_Supervised( pwr_SupplyEvent_t pvdEvent, pwr_SupplyEvent_t avdEvent )
{
    pwr_Config_t pwrConfig = UtPwr_Get_Config();

    pwrConfig.PvdActive      = PWR_FUNCTION_ACTIVE;
    pwrConfig.AvdActive      = PWR_FUNCTION_ACTIVE;
    pwrConfig.Pvd.Event      = pvdEvent;
    pwrConfig.Avd.Event      = avdEvent;
    pwrConfig.SupplyCallback = UtPwr_SupplyCallback;
    pwrConfig.SupplyIsr      = PWR_FUNCTION_ACTIVE;

    UtPwr_Expect_IsrInit( UT_PWR_REG_ZERO );
    UtPwr_Expect_LineEnable();

    UtPwr_Init_Config( &pwrConfig );

    TEST_ASSERT_NOT_NULL( utPwr_Isr );
}


/**
 * \brief Applies a voltage scale directly to the registers (the scale is applied and ready) and restarts the recorder.
 *
 * \param voltageScale [in]: Voltage scale
 */
static void UtPwr_Set_Scale( pwr_VoltageScale_t voltageScale )
{
    const uint32_t vosField = utPwr_ScaleField[voltageScale];

    PWR->VOSCR = ( vosField << PWR_VOSCR_VOS_Pos );
    PWR->VOSSR = ( ( vosField << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY | PWR_VOSSR_VOSRDY );

    utPwr_VosLast       = vosField;
    utPwr_VosHistoryCnt = UT_PWR_CALLS_NONE;
}


/**
 * \brief Sets the expectations of the initialization of the interrupt: the handler is stored, the priority is set.
 *
 * \param irqPriority [in]: Priority expected by the Nvic mock
 */
static void UtPwr_Expect_IsrInit( pwr_IrqPrio_t irqPriority )
{
    Nvic_Set_PeriphIrq_Handler_Stub( UtPwr_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, irqPriority, NVIC_REQUEST_OK );
}


/**
 * \brief Sets the expectations of the enabling of the EXTI line (both edges, interrupt) and of the Nvic interrupt.
 */
static void UtPwr_Expect_LineEnable( void )
{
    Exti_Set_LineActive_ExpectWithArrayAndReturn( EXTI_LINE_PVD_AVD, &utPwr_LineConfig, 1, EXTI_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, NVIC_REQUEST_OK );
}


/**
 * \brief Sets the expectations of the disabling of the Nvic interrupt and of the EXTI line.
 */
static void UtPwr_Expect_LineDisable( void )
{
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_PVD_AVD, NVIC_REQUEST_OK );
    Exti_Set_LineInactive_ExpectAndReturn( EXTI_LINE_PVD_AVD, EXTI_REQUEST_OK );
}


/**
 * \brief Sets the expectations of a Sleep entry: normal sleep selection and the wait for interrupt.
 */
static void UtPwr_Expect_Sleep( void )
{
    Scb_Set_DeepSleepInactive_Expect();
    Scb_Set_WaitForInterrupt_Expect();
    Scb_Set_WaitForInterrupt_AddCallback( UtPwr_ScbWfiStub );
}


/**
 * \brief Sets the expectations of a Stop or Standby entry: deep sleep selected, wait for interrupt, deep sleep cleared.
 */
static void UtPwr_Expect_Deep( void )
{
    Scb_Set_DeepSleepActive_Expect();
    Scb_Set_WaitForInterrupt_Expect();
    Scb_Set_WaitForInterrupt_AddCallback( UtPwr_ScbWfiStub );
    Scb_Set_DeepSleepInactive_Expect();
}


/** Pre enter hook of the tests */
static void UtPwr_HookPre( void )
{
    utPwr_Seq++;
    utPwr_PreSeq = utPwr_Seq;
    utPwr_PreCnt++;
}


/** Post exit hook of the tests */
static void UtPwr_HookPost( void )
{
    utPwr_Seq++;
    utPwr_PostSeq = utPwr_Seq;
    utPwr_PostCnt++;
}


/** Supervisor callback of the tests: remembers the last call */
static void UtPwr_SupplyCallback( pwr_Supply_t supply, pwr_SupplyState_t supplyState )
{
    utPwr_CbSupply = supply;
    utPwr_CbState  = supplyState;
    utPwr_CbCnt++;
}


/** Nvic stub: stores the interrupt handler the module registers for the PVD / AVD interrupt */
static nvic_RequestState_t UtPwr_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId,
                                                     const nvic_IsrCallback_t irqHandler,
                                                     int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( NVIC_PERIPH_IRQ_PVD_AVD, irqId );

    utPwr_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** Scb stub: the instruction that stops the core, records its place in the sequence and the mode register */
static void UtPwr_ScbWfiStub( int callCnt )
{
    (void)callCnt;

    utPwr_Seq++;
    utPwr_WfiSeq    = utPwr_Seq;
    utPwr_PmcrAtWfi = PWR->PMCR;
}


/** Scb stub: as the stop instruction, and the hardware resets the voltage scale to scale 3 at the wake-up */
static void UtPwr_ScbWfiStubVosReset( int callCnt )
{
    UtPwr_ScbWfiStub( callCnt );

    PWR->VOSCR = ( UT_PWR_VOS_FIELD_SCALE_3 << PWR_VOSCR_VOS_Pos );
    PWR->VOSSR = ( ( UT_PWR_VOS_FIELD_SCALE_3 << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY | PWR_VOSSR_VOSRDY );
}

/* ============================== HW MODELS =================================== */

/** HW model: VCORE reaches the selected scale at once (ACTVOS, ACTVOSRDY, VOSRDY), the VOS values are recorded */
static void UtPwr_HwModel_Vos( void )
{
    const uint32_t vosField = ( ( PWR->VOSCR & PWR_VOSCR_VOS_Msk ) >> PWR_VOSCR_VOS_Pos );

    if( vosField != utPwr_VosLast )
    {
        if( UT_PWR_VOS_HISTORY_SIZE > utPwr_VosHistoryCnt )
        {
            utPwr_VosHistory[utPwr_VosHistoryCnt] = vosField;
            utPwr_VosHistoryCnt++;
        }
        else
        {
            /* The recorder is full */
        }

        utPwr_VosLast = vosField;
    }
    else
    {
        /* The scale did not change */
    }

    __atomic_store_n( &PWR->VOSSR,
                      ( ( vosField << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY | PWR_VOSSR_VOSRDY ),
                      __ATOMIC_SEQ_CST );
}


/** HW model: as \ref UtPwr_HwModel_Vos, but VOSRDY is never set (the raise of the performance never finishes) */
static void UtPwr_HwModel_VosNoRaiseReady( void )
{
    const uint32_t vosField = ( ( PWR->VOSCR & PWR_VOSCR_VOS_Msk ) >> PWR_VOSCR_VOS_Pos );

    __atomic_store_n( &PWR->VOSSR,
                      ( ( vosField << PWR_VOSSR_ACTVOS_Pos ) | PWR_VOSSR_ACTVOSRDY ),
                      __ATOMIC_SEQ_CST );
}


/** HW model: the PVD enable bit is locked (it does not take the value, the lock of the SBS module) */
static void UtPwr_HwModel_PvdLocked( void )
{
    (void)__atomic_and_fetch( &PWR->VMCR, (uint32_t)~PWR_VMCR_PVDEN, __ATOMIC_SEQ_CST );
}


/** HW model: the write access to the backup domain does not take the value (DBP stays cleared) */
static void UtPwr_HwModel_DbpLocked( void )
{
    (void)__atomic_and_fetch( &PWR->DBPCR, (uint32_t)~PWR_DBPCR_DBP, __ATOMIC_SEQ_CST );
}


/** HW model: the SVOS field of PWR_PMCR does not keep the written value */
static void UtPwr_HwModel_SvosNotKept( void )
{
    (void)__atomic_and_fetch( &PWR->PMCR, (uint32_t)~PWR_PMCR_SVOS_Msk, __ATOMIC_SEQ_CST );
}


/** HW model: the bypass bit is locked (written once after the power-on reset, a later write is ignored) */
static void UtPwr_HwModel_BypassLocked( void )
{
    (void)__atomic_and_fetch( &PWR->SCCR, (uint32_t)~PWR_SCCR_BYPASS, __ATOMIC_SEQ_CST );
}


/** HW model: the LPMS bit of PWR_PMCR does not keep the written value */
static void UtPwr_HwModel_PmcrNotKept( void )
{
    (void)__atomic_and_fetch( &PWR->PMCR, (uint32_t)~PWR_PMCR_LPMS, __ATOMIC_SEQ_CST );
}
