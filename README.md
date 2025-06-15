# KRT Port for PIC 16F628.
Support:
- GPIO Ports A & B Read, Write, Configure
- Pull-up port B Enable, Disable
- Serial UART communiation driver

K-RT Base commands support:
- skst, stopkst: Stop K-ST (Kernel Space scheduler)
- rkst, runkst: Run K-ST (Kernel Space scheduler)
- sys, systat: Read-back systat then reset min/max
- clr, clea: Clear Fault and Alarm Words
- sust, stopust: Stop U-ST (User Space scheduler)
- rust, runust: Run U-ST (User Space scheduler)
- sta, status: Return Status, Alarm and Fault words
- g[n]ci: Set GPIO [n] as digital input
- g[n]co: Set GPIO [n] as digital output
- g[n]cr: Read-back GPIO [n] Configuration
- g[n]cod: Configure GPIO [n] Open Drain (High Z)
- g[n]cpu: Configure GPIO [n] Pull Up

MPLAB X Footprint:
```
16F628 Memory Summary:
    Program space        used   611h (  1553) of   800h words   ( 75.8%)
    Data space           used    61h (    97) of    E0h bytes   ( 43.3%)
    EEPROM space         used     0h (     0) of    80h bytes   (  0.0%)
    Configuration bits   used     1h (     1) of     1h word    (100.0%)
    ID Location space    used     0h (     0) of     4h bytes   (  0.0%)
```
