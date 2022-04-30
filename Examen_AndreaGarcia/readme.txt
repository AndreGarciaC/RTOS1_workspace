FreeRTOS-project example using FreeRTOS - Event Driven System (EDS) - Project for STM32F429ZI_NUCLEO_144

Example description
Required_SystemCoreClock => 180MHz
SysTick_RateHz => 1000 ticks per second (1mS)

vTaskA
    Simple access controller code (see "Access Controller.jpg" for details).

vTaskB
    Simple access controller code (see "Access Controller.jpg" for details).

vTaskTest
    Simply excites the other tasks (see "Access Controller.jpg" for details).

Also:
toggle LDs STM32F429ZI_NUCLEO_144 board
	Turn OFF LED & Turn ON LED

Special connection requirements
There are no special connection requirements for this example.

Libraries:	lpcxpresso => Examples => LPCOpen

Build procedures:
Visit the LPCOpen quickstart guide at "http://www.lpcware.com/content/project/lpcopen-platform-nxp-lpc-microcontrollers/lpcopen-v200-quickstart-guides"
to get started building LPCOpen projects.
