# PWR Peripheral Driver

This module provides an abstraction layer for the **power controller** (`PWR`) of STM32 MCUs: the **voltage scaling** of the
core, the **low-power modes** (Sleep, Stop, Standby, Shutdown) with their wake-up sources, the **supply supervision**
(programmable voltage detector, analog voltage detector), the access to the **backup domain**, the retention of the
memories, the battery charging of the `VBAT` pin and the supply of the USB transceiver and of the independent I/O banks.
It is a low-level MCAL module: it owns the PWR hardware, the application and the other modules never touch the registers.

The public interface (`Pwr_Port.h`, `Pwr_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- Every other module depends on the power controller without owning it: the RCC module needs the **voltage scale** that
  allows the system clock it is asked for, the RTC and the LSE need the **write access to the backup domain**, the USB
  needs the **VDDUSB supply** to be valid, a battery powered application needs the **Stop and Standby modes** and the
  **wake-up pins**. Today these steps are scattered; the module is the only place that handles them, and the other modules call
  its functions (`Pwr_Set_BackupAccessActive`, `Pwr_Set_VoltageScale` ...).
- The power modes are an **ordered procedure**, not a bit: the voltage scale has to be changed step by step, the regulator
  has to be ready, the wake-up sources have to be armed before the sleep and the flags cleared after, the deep sleep bit of
  the core has to be set, the clocks have to be restored after a Stop mode. The module offers `Pwr_Set_StopEnter` as one
  call that follows the procedure of the family and returns after the wake-up, with the hooks that let the application
  switch off its own peripherals before the sleep and restart them after it.
- The module reports **why the MCU woke up** (`Pwr_Get_WakeUpSource`) and **what mode it came from** (standby flag),
  so that an application can distinguish a power-on from a wake-up from Standby.
- The module hides the differences between the families: the number and the names of the voltage scales, the number of
  the Stop modes, the wake-up pins and their configuration, the retention options, the SMPS / LDO regulators, the
  supervisors and the TrustZone aliases differ, the interface stays the same.

What the module is **not**: it does not select or start the clocks (the RCC module does), it does not decide which
peripherals sleep (the application does, through the hooks), it does not drive the reset (`Rcc` reports the reset
source) and it does not save the application state over the Standby mode (the backup registers of the Rtc / Tamp modules).

---

## Concept

The module keeps the following areas apart:

| Area | Functions | Meaning |
|---|---|---|
| **Voltage scaling** | `Pwr_Set_VoltageScale`, `Pwr_Get_VoltageScale` | The core voltage scale (`PWR_VOLTAGE_SCALE_0` ... in the order of the family, 0 = the highest performance). The module walks through the intermediate scales as the family requires and waits for the ready flag; the RCC module asks for a scale before it raises the system clock. |
| **Low-power modes** | `Pwr_Set_SleepEnter`, `Pwr_Set_StopEnter`, `Pwr_Set_StandbyEnter`, `Pwr_Set_ShutdownEnter` | Sleep (the core stopped, the clocks run), Stop (the clocks stopped, the memories retained, the sub-modes of the family `PWR_STOP_0` ...), Standby (only the backup domain and the selected memories retained, the MCU restarts), Shutdown (the lowest consumption, where the family has it). |
| **Wake-up** | `Pwr_Set_WakeUpPin`, `Pwr_Get_WakeUpSource`, `Pwr_Set_WakeUpFlagClear` | The wake-up pins (`WKUPx`) with the polarity and the pull, the internal sources (RTC, tamper, the peripherals that work in Stop), the flags. |
| **Retention** | `Pwr_Set_SramRetention`, `Pwr_Set_IoRetention` | Which SRAMs and which I/O states are kept in Stop and Standby. |
| **Supervision** | `Pwr_Set_PvdConfig`, `Pwr_Set_PvdActive`, `Pwr_Set_AvdConfig`, `Pwr_Get_SupplyState` | The programmable voltage detector (threshold, edge, callback), the analog supply detector, the readiness of VDDUSB and VDDIO2, the monitoring of the backup domain voltage and of the temperature. |
| **Backup domain** | `Pwr_Set_BackupAccessActive`, `Pwr_Set_BackupAccessInactive`, `Pwr_Set_VbatCharging` | The write access to the backup domain registers (needed by the RTC, the LSE and the tamper), the charging of the backup battery through a resistor. |
| **Supplies** | `Pwr_Set_UsbSupplyActive`, `Pwr_Set_Io2SupplyActive`, `Pwr_Set_RegulatorMode` | The independent supplies of the USB transceiver and of the I/O bank, the selection of the regulator (LDO / SMPS) where the device has the choice. |

Rules:

1. A request that the hardware cannot do in the current state (the scale not reachable from the current one with the current
   clock, a Stop sub-mode the family does not have) is rejected **before** any register is written.
2. `Pwr_Set_VoltageScale` is called **before** the clock is raised and **after** it is lowered; the module does not check the clock
   (the RCC module does), it only guarantees the sequence of the power registers and the wait for the ready flag with a
   timeout.
3. The entry to a low-power mode is a call that **returns after the wake-up** (Sleep, Stop) or **does not return** (Standby,
   Shutdown: the MCU restarts). Before the entry the module calls the `PreEnterCallback`, after the wake-up the
   `PostExitCallback` (the application stops and restarts its peripherals; the RCC restores the clocks). Armed wake-up
   sources are the ones set by the application before the call.
4. After a restart from Standby `Pwr_Get_WakeUpSource` returns the wake-up source and the module clears the flags with
   `Pwr_Set_WakeUpFlagClear`; the application reads them before it clears them.
5. The write access to the backup domain is **reference counted**: every module that needs it (RCC for the LSE, RTC, Tamp) calls
   `Pwr_Set_BackupAccessActive` and releases it with `Pwr_Set_BackupAccessInactive`; the protection is re-enabled when
   the last user releases it.
6. The supervisor callbacks (PVD, AVD) are called from the interrupt of the family (through the EXTI line where
   the family routes it); the flags are cleared by the module.
7. All parameters are checked before a register is written; the module reports the TrustZone / privilege rules of the
   family as an error when the caller is not allowed.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Power controller | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `PWR`: voltage scales `VOS0` ... `VOS3`, LDO and (on STM32H563 / H573) SMPS, Sleep, Stop, Standby, wake-up pins `WKUP1` ... `WKUP8`, PVD, AVD, `VDDUSB` and `VDDIO2`, `VBAT` charging, TrustZone and privilege protection | planned (first) |
| STM32U5 | `Dev/STM32U5` | `PWR` with the Stop modes 0 ... 3, SMPS, wake-up pins, retention options of the SRAMs | planned |
| STM32U3 | `Dev/STM32U3` | `PWR` | planned |
| STM32G4, STM32L4 | `Dev/STM32G4`, `Dev/STM32L4` | `PWR` with the low-power modes Sleep, Low-power run / sleep, Stop 0 / 1 / 2, Standby, Shutdown, wake-up pins `WKUP1` ... `WKUP5`, PVD | planned |
| STM32H7 | `Dev/STM32H7` | `PWR` with the supply configuration (LDO / SMPS), the voltage scales, the domains D1 / D2 / D3 and their modes | planned |
| STM32F4, STM32F7 | `Dev/STM32F4`, `Dev/STM32F7` | `PWR` with the regulator voltage scale, over-drive (where the device has it), Sleep, Stop, Standby, `WKUP` pin(s), PVD | planned |

A function without a meaning in a family stays in the interface, returns `PWR_REQUEST_ERROR` and suppresses the unused
parameters (for example the Shutdown mode on a family without it, the SMPS on a device with the LDO only, a Stop
sub-mode or a retention option the device does not have).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Pwr_Get_ModuleVersion`, `Pwr_Init`, `Pwr_Deinit`, `Pwr_Task`, `Pwr_Get_DefaultConfig` |
| Voltage scaling | `Pwr_Set_VoltageScale`, `Pwr_Get_VoltageScale`, `Pwr_Set_RegulatorMode` |
| Low-power modes | `Pwr_Set_SleepEnter`, `Pwr_Set_StopEnter`, `Pwr_Set_StandbyEnter`, `Pwr_Set_ShutdownEnter` |
| Wake-up | `Pwr_Set_WakeUpPin`, `Pwr_Get_WakeUpSource`, `Pwr_Get_StandbyFlag`, `Pwr_Set_WakeUpFlagClear` |
| Retention | `Pwr_Set_SramRetention`, `Pwr_Get_SramRetention`, `Pwr_Set_IoRetention` |
| Supervision | `Pwr_Set_PvdConfig`, `Pwr_Set_PvdActive`, `Pwr_Set_PvdInactive`, `Pwr_Set_AvdConfig`, `Pwr_Get_SupplyState` |
| Backup domain and supplies | `Pwr_Set_BackupAccessActive`, `Pwr_Set_BackupAccessInactive`, `Pwr_Set_VbatCharging`, `Pwr_Set_UsbSupplyActive`, `Pwr_Set_Io2SupplyActive` |
| Interrupts | `Pwr_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |

```c
pwr_ModuleVersion_t Pwr_Get_ModuleVersion        ( void );

pwr_RequestState_t  Pwr_Init                     ( const pwr_Config_t * const pwrConfig );
pwr_RequestState_t  Pwr_Deinit                   ( void );
void                Pwr_Task                     ( void );
pwr_RequestState_t  Pwr_Get_DefaultConfig        ( pwr_Config_t * const pwrConfig );

pwr_RequestState_t  Pwr_Set_VoltageScale         ( pwr_VoltageScale_t voltageScale );
pwr_RequestState_t  Pwr_Get_VoltageScale         ( pwr_VoltageScale_t * const voltageScale );

pwr_RequestState_t  Pwr_Set_SleepEnter           ( pwr_WakeUpEvent_t wakeUpEvent );
pwr_RequestState_t  Pwr_Set_StopEnter            ( pwr_StopMode_t stopMode );
pwr_RequestState_t  Pwr_Set_StandbyEnter         ( void );
pwr_RequestState_t  Pwr_Set_ShutdownEnter        ( void );

pwr_RequestState_t  Pwr_Set_WakeUpPin            ( pwr_WakeUpPin_t wakeUpPin, const pwr_WakeUpPinConfig_t * const pinConfig );
pwr_RequestState_t  Pwr_Get_WakeUpSource         ( pwr_WakeUpSource_t * const wakeUpSource );
pwr_RequestState_t  Pwr_Set_WakeUpFlagClear      ( void );

pwr_RequestState_t  Pwr_Set_SramRetention        ( pwr_SramId_t sramId, pwr_FunctionState_t retention );

pwr_RequestState_t  Pwr_Set_PvdConfig            ( const pwr_PvdConfig_t * const pvdConfig );
pwr_RequestState_t  Pwr_Set_PvdActive            ( void );
pwr_RequestState_t  Pwr_Get_SupplyState          ( pwr_Supply_t supply, pwr_SupplyState_t * const supplyState );

pwr_RequestState_t  Pwr_Set_BackupAccessActive   ( void );
pwr_RequestState_t  Pwr_Set_BackupAccessInactive ( void );
pwr_RequestState_t  Pwr_Set_VbatCharging         ( pwr_VbatCharging_t vbatCharging );
```

Every function returns `PWR_REQUEST_OK` / `PWR_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `pwr_Config_t` | Voltage scale at the start, regulator mode, the wake-up pins, the PVD configuration, the `PreEnterCallback`, `PostExitCallback`, `PvdCallback`, interrupt priority |
| `pwr_VoltageScale_t` | `PWR_VOLTAGE_SCALE_0` ... - only the scales of the device |
| `pwr_StopMode_t` | `PWR_STOP_0` ... - only the Stop sub-modes of the device |
| `pwr_WakeUpPin_t`, `pwr_WakeUpPinConfig_t` | `PWR_WAKEUP_PIN_1` ...; polarity (rising / falling) and pull (none / up / down) |
| `pwr_WakeUpSource_t` | Bit mask: pin 1 ... n, RTC alarm / wake-up timer, tamper, a peripheral of the family, the standby flag |
| `pwr_Supply_t`, `pwr_SupplyState_t` | `PWR_SUPPLY_USB`, `PWR_SUPPLY_IO2`, `PWR_SUPPLY_ANALOG`, `PWR_SUPPLY_BACKUP`; valid / not valid |

---

## Usage

### Raising the performance before a faster clock

```c
#include "Pwr_Port.h"

(void)Pwr_Set_VoltageScale( PWR_VOLTAGE_SCALE_0 );             /* the intermediate scales are walked by the module */
/* now the RCC module may switch the system clock to the maximum frequency */
```

### Stop mode with a wake-up pin and a periodic RTC wake-up

```c
static void App_BeforeStop( void ) { /* stop the peripherals that must not run */ }
static void App_AfterStop( void )  { /* start them again, the RCC restored the clocks */ }

pwr_Config_t pwrConfig;
pwr_WakeUpPinConfig_t buttonPin = { .Polarity = PWR_WAKEUP_RISING, .Pull = PWR_PULL_DOWN };

(void)Pwr_Get_DefaultConfig( &pwrConfig );
pwrConfig.PreEnterCallback = App_BeforeStop;
pwrConfig.PostExitCallback = App_AfterStop;
(void)Pwr_Init( &pwrConfig );

(void)Pwr_Set_WakeUpPin( PWR_WAKEUP_PIN_2, &buttonPin );
/* the Rtc module arms the periodic wake-up timer */

(void)Pwr_Set_StopEnter( PWR_STOP_1 );                         /* returns after the wake-up */
```

### Standby and the reason of the restart

```c
pwr_WakeUpSource_t wakeUpSource = 0u;
pwr_FunctionState_t standbyFlag = PWR_FUNCTION_INACTIVE;

(void)Pwr_Get_StandbyFlag( &standbyFlag );

if( PWR_FUNCTION_ACTIVE == standbyFlag )
{
    (void)Pwr_Get_WakeUpSource( &wakeUpSource );        /* which pin / RTC event brought the MCU back */
    (void)Pwr_Set_WakeUpFlagClear();
}
else
{
    /* A power-on or a reset, not a wake-up from Standby */
}
```

---

## Behaviour and limits

- **Sequence and time.** The change of the voltage scale and the wake-up from Stop / Standby take time (tens of
  microseconds up to milliseconds); the module waits for the ready flags with a timeout and reports the timeout as an error.
  The restart after Standby / Shutdown is a reset: the module cannot return.
- **Clocks.** The system clock after a Stop mode is the one the family restarts with (the internal oscillator); the RCC
  module restores the configured clocks in the `PostExitCallback` sequence. Leaving the power mode with the old high
  frequency and a lower voltage scale is a hardware fault - the application orders the calls as above.
- **Wake-up.** An event that is not armed before the entry does not wake the MCU; an event that is pending at the entry makes
  the entry return at once. The flags of the wake-up sources are sticky and must be cleared.
- **Debugger.** The low-power modes interact with the debug connection (the clocks stopped in Stop, the debug domain);
  the DbgMcu module keeps the debug alive in the low-power modes during the development.
- **Backup domain.** The access is reference counted; a module that forgets to release it keeps the registers writable.
  Reset of the backup domain is the task of the RCC module.
- **Not part of the first version:** the dynamic voltage and frequency scaling policies, the autonomous mode of the
  peripherals, the power state machine of a dual-core device (STM32H7 domains), the battery voltage measurement (the Adc
  module measures `VBAT`).

---

## Dependencies

- **Rcc** - the clock restore after a low-power mode, the backup domain reset, the reset source, the clock needed for the PWR registers
- **Nvic** - the deep sleep bit and the wait-for-interrupt entry of the core, the PVD / AVD interrupts
- **Exti** - the wake-up lines of the supervisors and of the peripherals on the families that route them through the EXTI
- **Gpio** - the wake-up pins in the required mode
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC, EXTI and GPIO configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the walk through the intermediate voltage scales for every pair of the
  scales, the ready timeouts, the entry sequences of every low-power mode (the order of the register writes, the hooks), the
  rejection of the modes and the retention options the device does not have, the reference counting of the backup
  access, the PVD configuration and callbacks, the wake-up flag handling.
- **Integration tests** (target, probe-rs): the voltage scale change under the load of a running loop, the wake-up from
  Sleep and Stop by a timer and by a pin, the reason of the wake-up after Standby, the PVD callback with a lowered
  supply, the consumption in the modes (measured by the test board where it is available).

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `PWR_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Pwr_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Pwr_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`pwr_FunctionState_t`, `pwr_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `PWR_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Pwr_Port.h`, `Pwr_Types.h` | Public interface (identical in all families) |
| `Pwr*.c`, `Pwr.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Pwr_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Pwr_Lib )
```

1. Link `Pwr_Lib` to your CMake library.
2. Include `Pwr_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
