# Automated-Safety-Stand
Automated safety stand for motor bikes


## Target Microcontroller Unit: ATmega328P
* **Datasheet :** https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
* **Product Description Page :** https://www.microchip.com/en-us/product/ATmega328P


<p align="center">
<img src="figures/Atmel-MCU-ATmega328-and-the-Arduino-pin-out_W640.jpg"  align="center" width=500>
</p>

> *The high-performance Microchip picoPower® 8-bit AVR® RISC-based microcontroller combines 32 KB ISP Flash memory with read-while-write capabilities, 1024B EEPROM, 2 KB SRAM, 23 general purpose I/O lines, 32 general purpose working registers, three flexible timer/counters with compare modes, internal and external interrupts, serial programmable USART, a byte-oriented Two-Wire serial interface, SPI serial port, a 6-channel 10-bit A/D converter (8-channels in TQFP and QFN/MLF packages), programmable watchdog timer with internal oscillator, and five software selectable power saving modes. The device operates between 1.8-5.5 volts.By executing powerful instructions in a single clock cycle, the device achieves throughputs approaching one MIPS per MHz, balancing power consumption and processing speed.*


### External intrrupts to identify the moment of switch on: 11.1 Interrupt Vectors in ATmega328P Datasheet 

- 
```
2 0x002     INT0 External interrupt request 0 - PD2
3 0x0004    INT1 External interrupt request 1 - PD3
```


## References

1. [ATmega Interface](https://www.electronicwings.com/avr-atmega/interfaces)
2. [MPU6050 (Gyroscope + Accelerometer + Temperature) interface with AVR ATmega16](https://www.electronicwings.com/avr-atmega/mpu6050-gyroscope-accelerometer-temperature-interface-with-atmega16)
3. [The MPU6050 Explained](https://mjwhite8119.github.io/Robots/mpu6050)
4. [Add multiple files and folders to existing project in Atmel Studio6](https://microchipsupport.force.com/s/article/Add-multiple-files-and-folders-to-existing-project-in-Atmel-Studio6)


# Arduino Project (Chosen)
*Due to lack of complete code reference for MPU6050 to use with ATmega328P*

<p align="center">
<img src="figures/Atmega168PinMap2.png"  align="center" width=500>
</p>


1. [Build a Digital Level with MPU-6050 and Arduino](https://dronebotworkshop.com/mpu-6050-level/)
2. [Arduino Uno to ATmega328 - Shrinking your Arduino Projects](https://youtu.be/Sww1mek5rHU)
3. [Build an Electronic Level with MPU-6050 and Arduino](https://youtu.be/XCyRXMvVSCw)
4. [MPU-6050 6-axis accelerometer/gyroscope - I2Cdevlib documatation](https://www.i2cdevlib.com/devices/mpu6050#source)
5. [jrowberg/i2cdevlib](https://github.com/jrowberg/i2cdevlib)
6. [MPU6050 Class Reference](https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html)
