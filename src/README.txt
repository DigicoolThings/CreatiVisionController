Please read the header comments in main.c source file.

Specifically, this XC8 code was developed in MPLAB X v6.20, also using MCC for correct hardware setup and initialisation (as outlined).

Therefore, to correctly use this source code:  First install MPLAB then create a new project for the appropriate AVR MCU.  
Next, use MCC to generate the initialisation #include code (configuration settings as outlined), and then copy the main.c source file into the new project (overwriting the default created main.c file template).
