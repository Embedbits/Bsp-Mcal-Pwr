#======================== Unit tests configuration ============================#
# Unit tests of Pwr module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(PWR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${PWR_DIR}/.." ABSOLUTE)

# Tests of the waits and of the read-back verification use the HW model of RegMem (status
# that follows a written register, a bit that does not take the value) - executed serially
# where the model runs in a thread
UnitTesting_Add_Test(
    NAME            Pwr
    TEST_SOURCE     Test_Pwr.c
    UUT_SOURCES     ${PWR_DIR}/Pwr.c
                    ${PWR_DIR}/Pwr_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Exti/Exti_Port.h
                    ${MCAL_DIR}/Core/Scb/Scb_Port.h
    INCLUDE_DIRS    ${PWR_DIR}
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Exti
                    ${MCAL_DIR}/Core/Scb
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
