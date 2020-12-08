*** Settings ***
Suite Setup                             Setup
Suite Teardown                          Teardown
Test Setup                              Reset Emulation
Test Teardown                           Test Teardown
Resource                                ${RENODEKEYWORDS}

*** Variables ***
${TIMER_3_COMPARE_1}                    0x34
${TIMER_3_COMPARE_2}                    0x38
${TIMER_3_COMPARE_3}                    0x3C
${TIMER_3_COMPARE_4}                    0x40

${PWM_MARGIN}                           0x10
${WING_START_VALUE_BASE}                0x1434
${WING_DRS_VALUE_BASE}                  0x6A4

${WING_START_VALUE_MIN}                 ${${WING_START_VALUE_BASE} - ${PWM_MARGIN}}
${WING_START_VALUE_MAX}                 ${${WING_START_VALUE_BASE} + ${PWM_MARGIN}}
${WING_DRS_VALUE_MIN}                   ${${WING_DRS_VALUE_BASE} - ${PWM_MARGIN}}
${WING_DRS_VALUE_MAX}                   ${${WING_DRS_VALUE_BASE} + ${PWM_MARGIN}}

${STARTUP_TIME}                         10
${STARTUP}                              1

*** Keywords ***
Add Files
    Execute Command                     include @peripherals/STM32F0GPIOPort.cs
    Execute Command                     include @peripherals/STM32F0Timer3.cs
    Execute Command                     include @peripherals/STM32F0Timer6.cs

Create Target
    Execute Command                     mach create
    Execute Command                     machine LoadPlatformDescription @platforms/boards/acm_pcba_v1.0.repl
    Execute Command                     sysbus LoadELF @code/RTOS_DRS.elf

*** Test Cases ***
Create Machine
    [Tags]                              Meta tests
    Add Files
    Create Target
    Provides                            ACM

Check Rear Wing Startup Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks startup servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_1}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_START_VALUE_MAX}

Check Left Front Wing Startup Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks startup servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_2}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_START_VALUE_MAX}

Check Right Front Wing Startup Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks startup servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_3}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_START_VALUE_MAX}

Check Rear Wing DRS Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks DRS servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    Execute Command                     sysbus.gpioPortC.DRSButton Press
    Sleep                               1s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_1}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_DRS_VALUE_MAX}

Check Left Front Wing DRS Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks DRS servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    Execute Command                     sysbus.gpioPortC.DRSButton Press
    Sleep                               1s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_2}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_DRS_VALUE_MAX}

Check Right Front Wing DRS Steady-State Value
    [Tags]                              Servo control tests
    [Documentation]                     Checks DRS servo position
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    Execute Command                     sysbus.gpioPortC.DRSButton Press
    Sleep                               1s
    ${pwm}=  Execute Command            sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_3}
    ${pwm_clean}=  Convert To Number    ${pwm}

    Should Be True                      ${pwm_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm_clean} < ${WING_DRS_VALUE_MAX}

Check DRS Deployment And Release
    [Tags]                              Servo control tests
    [Documentation]                     Checks DRS deployment
    Create Target
    Start Emulation
    Sleep                               ${STARTUP_TIME}s
    Execute Command                     sysbus.gpioPortC.DRSButton Press
    Sleep                               1s

    ${pwm1}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_1}
    ${pwm2}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_2}
    ${pwm3}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_3}
    ${pwm1_clean}=  Convert To Number   ${pwm1}
    ${pwm2_clean}=  Convert To Number   ${pwm2}
    ${pwm3_clean}=  Convert To Number   ${pwm3}
    
    Should Be True                      ${pwm1_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm1_clean} < ${WING_DRS_VALUE_MAX}
    Should Be True                      ${pwm2_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm2_clean} < ${WING_DRS_VALUE_MAX}
    Should Be True                      ${pwm3_clean} > ${WING_DRS_VALUE_MIN}
    Should Be True                      ${pwm3_clean} < ${WING_DRS_VALUE_MAX}

    Execute Command                     sysbus.gpioPortC.DRSButton Release
    Sleep                               1s

    ${pwm1}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_1}
    ${pwm2}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_2}
    ${pwm3}=  Execute Command           sysbus.timer3 ReadDoubleWord ${TIMER_3_COMPARE_3}
    ${pwm1_clean}=  Convert To Number   ${pwm1}
    ${pwm2_clean}=  Convert To Number   ${pwm2}
    ${pwm3_clean}=  Convert To Number   ${pwm3}
    
    Should Be True                      ${pwm1_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm1_clean} < ${WING_START_VALUE_MAX}
    Should Be True                      ${pwm2_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm2_clean} < ${WING_START_VALUE_MAX}
    Should Be True                      ${pwm3_clean} > ${WING_START_VALUE_MIN}
    Should Be True                      ${pwm3_clean} < ${WING_START_VALUE_MAX}
