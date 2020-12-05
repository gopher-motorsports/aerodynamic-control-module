*** Settings ***
Suite Setup                   Setup
Suite Teardown                Teardown
Test Setup                    Reset Emulation
Test Teardown                 Test Teardown
Resource                      ${RENODEKEYWORDS}

*** Keywords ***
Create Target
    Execute Command         include @peripherals/STM32F0GPIOPort.cs
    Execute Command         mach create
    Execute Command         machine LoadPlatformDescription @platforms/cpus/stm32f072.repl
    Execute Command         machine LoadPlatformDescriptionFromString "button: Miscellaneous.Button @ gpioPortB 0 { IRQ -> gpioPortB@0 }"
    Execute Command         sysbus LoadELF @code/RTOS_DRS.elf

*** Test Cases ***
Should Handle Button Press
    Create Target
    [Tags]                  Smoke Tests
    Execute Command         emulation CreateLEDTester "lt" sysbus.gpioPortA.UserLED1

    Start Emulation

    Execute Command         lt AssertState True 1
    Execute Command         sysbus.gpioPortB.button Press
    Execute Command         lt AssertState False 1
    Execute Command         sysbus.gpioPortB.button Release
    Execute Command         lt AssertState True 1