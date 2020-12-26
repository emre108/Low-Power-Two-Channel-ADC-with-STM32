# Low-Power-Two-Channel-ADC-with-STM32

The direct memory access(DMA) feature has been implemented for two-channel ADC. Digital data can be seen in Serial Monitor via RS-232 or Mobil Application via Bluetooth Module(HC-05). In order to time the tasks, a timer-framework(constructed with SysTick) is used. To have maximum efficiency, the Stop-Mode feature is implemented. 
The device can be woken up by the user(remotely or with a button on the device) or automatically with the Real-Time Clock(RTC). Everything that is happening on the device can be read through LEDs on the board.
