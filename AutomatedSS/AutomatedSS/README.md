# Pin Allocation of the ATmega328P

|Pin|Purpose|
|:----|:----|
|PC6 | reset|
|PD0 | key on determinant switch( pin will be high if key is on)|
|PD2 | IR digital input|
|PD4 | lift down pin(if this pin is high,stand should be lift down)|
|PB6,PB7| oscillator|
|PD5 | red led(while stand is lifting down and lifting up, this led should on)|
|PD6 | green led (if the stand is in right position, this led is on)|
|PC5 | SCL|
|PC4 | SDL - I2C communication with gyrascope|
|PC3,PC2,PC1,PC0 | stepper outs|