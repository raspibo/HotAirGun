EESchema Schematic File Version 2
LIBS:HotAirGun-rescue
LIBS:74xx
LIBS:ac-dc
LIBS:adc-dac
LIBS:Altera
LIBS:analog_devices
LIBS:analog_switches
LIBS:atmel
LIBS:audio
LIBS:Battery_Management
LIBS:bbd
LIBS:Bosch
LIBS:brooktre
LIBS:cmos4000
LIBS:conn
LIBS:Connector
LIBS:contrib
LIBS:cypress
LIBS:dc-dc
LIBS:Decawave
LIBS:device
LIBS:digital-audio
LIBS:Diode
LIBS:Display
LIBS:driver_gate
LIBS:dsp
LIBS:DSP_Microchip_DSPIC33
LIBS:elec-unifil
LIBS:ESD_Protection
LIBS:Espressif
LIBS:FPGA_Actel
LIBS:ftdi
LIBS:gennum
LIBS:Graphic
LIBS:hc11
LIBS:infineon
LIBS:intel
LIBS:interface
LIBS:intersil
LIBS:ir
LIBS:Lattice
LIBS:LED
LIBS:LEM
LIBS:linear
LIBS:Logic_74xgxx
LIBS:Logic_74xx
LIBS:Logic_CMOS_4000
LIBS:Logic_CMOS_IEEE
LIBS:logic_programmable
LIBS:Logic_TTL_IEEE
LIBS:maxim
LIBS:MCU_Microchip_PIC10
LIBS:MCU_Microchip_PIC12
LIBS:MCU_Microchip_PIC16
LIBS:MCU_Microchip_PIC18
LIBS:MCU_Microchip_PIC24
LIBS:MCU_Microchip_PIC32
LIBS:MCU_NXP_Kinetis
LIBS:MCU_NXP_LPC
LIBS:MCU_NXP_S08
LIBS:MCU_Parallax
LIBS:MCU_ST_STM8
LIBS:MCU_ST_STM32
LIBS:MCU_Texas_MSP430
LIBS:Mechanical
LIBS:memory
LIBS:microchip
LIBS:microcontrollers
LIBS:modules
LIBS:Motor
LIBS:motor_drivers
LIBS:motorola
LIBS:Msystem
LIBS:nordicsemi
LIBS:nxp
LIBS:onsemi
LIBS:opto
LIBS:Oscillators
LIBS:philips
LIBS:power
LIBS:Power_Management
LIBS:powerint
LIBS:pspice
LIBS:references
LIBS:regul
LIBS:Relay
LIBS:relays
LIBS:RF_Bluetooth
LIBS:rfcom
LIBS:RFSolutions
LIBS:Sensor_Current
LIBS:Sensor_Humidity
LIBS:sensors
LIBS:silabs
LIBS:siliconi
LIBS:supertex
LIBS:Switch
LIBS:switches
LIBS:texas
LIBS:Transformer
LIBS:Transistor
LIBS:transistors
LIBS:triac_thyristor
LIBS:Valve
LIBS:valves
LIBS:video
LIBS:wiznet
LIBS:Worldsemi
LIBS:Xicor
LIBS:xilinx
LIBS:zetex
LIBS:Zilog
LIBS:arduino
LIBS:HotAirGun-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title "HotAirGun"
Date "2018-12-03"
Rev "V2.0"
Comp "Raspibo"
Comment1 "https://github.com/raspibo/I2cController"
Comment2 ""
Comment3 ""
Comment4 "I2c Controller HAG Configuration"
$EndDescr
$Comp
L MCP23017 U1
U 1 1 58E54BD6
P 6300 2650
F 0 "U1" H 6000 3650 50  0000 C CNN
F 1 "MCP23017" H 6550 1650 50  0000 C CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_LongPads" H 6350 1700 50  0001 L CNN
F 3 "" H 6550 3650 50  0001 C CNN
	1    6300 2650
	1    0    0    -1  
$EndComp
NoConn ~ 6800 2250
$Comp
L LCD16X2 DS1
U 1 1 58ED3CD5
P 2300 1350
F 0 "DS1" H 3178 1315 50  0000 L CNN
F 1 "LCD16X2" H 3178 1224 50  0000 L CNN
F 2 "Display:WC1602A" H 2300 1300 50  0001 C CIN
F 3 "" H 2300 1350 50  0001 C CNN
	1    2300 1350
	1    0    0    -1  
$EndComp
Text GLabel 2550 2000 3    60   Input ~ 0
D4
Text GLabel 2650 1850 3    60   Input ~ 0
D5
Text GLabel 2750 2000 3    60   Input ~ 0
D6
Text GLabel 2850 1850 3    60   Input ~ 0
D7
Text GLabel 1850 2025 3    60   Input ~ 0
RS
Text GLabel 1950 1850 3    60   Input ~ 0
RW
Text GLabel 2050 2075 3    60   Input ~ 0
E
Text GLabel 5499 1850 0    60   Input ~ 0
LED1
Text GLabel 5750 1950 0    60   Input ~ 0
D7
Text GLabel 5500 2050 0    60   Input ~ 0
D6
Text GLabel 2450 2950 0    60   Input ~ 0
LED1
$Comp
L R R1
U 1 1 58ED60AB
P 2600 2950
F 0 "R1" V 2393 2950 50  0000 C CNN
F 1 "470R" V 2484 2950 50  0000 C CNN
F 2 "Discret:R3" V 2530 2950 50  0001 C CNN
F 3 "" H 2600 2950 50  0001 C CNN
	1    2600 2950
	0    1    1    0   
$EndComp
Text GLabel 5750 2150 0    60   Input ~ 0
D5
Text GLabel 5500 2250 0    60   Input ~ 0
D4
Text GLabel 5750 2350 0    60   Input ~ 0
E
Text GLabel 5500 2450 0    60   Input ~ 0
RW
Text GLabel 5750 2550 0    60   Input ~ 0
RS
$Comp
L SW_Push SW1
U 1 1 5C0975EB
P 2000 5600
F 0 "SW1" H 2050 5700 50  0000 L CNN
F 1 "Start" H 2000 5540 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_SMALL" H 2000 5800 50  0001 C CNN
F 3 "" H 2000 5800 50  0001 C CNN
	1    2000 5600
	0    1    1    0   
$EndComp
$Comp
L SW_Push SW2
U 1 1 5C097638
P 2500 5600
F 0 "SW2" H 2550 5700 50  0000 L CNN
F 1 "Stop" H 2500 5540 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_SMALL" H 2500 5800 50  0001 C CNN
F 3 "" H 2500 5800 50  0001 C CNN
	1    2500 5600
	0    1    1    0   
$EndComp
$Comp
L SW_Push SW3
U 1 1 5C0976B1
P 3000 5600
F 0 "SW3" H 3050 5700 50  0000 L CNN
F 1 "WeldCurv" H 3000 5540 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_PUSH_SMALL" H 3000 5800 50  0001 C CNN
F 3 "" H 3000 5800 50  0001 C CNN
	1    3000 5600
	0    1    1    0   
$EndComp
Text GLabel 5450 3050 0    60   Input ~ 0
Start
Text GLabel 5750 3150 0    60   Input ~ 0
Led
Text GLabel 5450 3250 0    60   Input ~ 0
Stop
Text GLabel 5750 3350 0    60   Input ~ 0
Buz
Text GLabel 5450 3450 0    60   Input ~ 0
WeldCurv
$Comp
L Buzzer BZ1
U 1 1 5C09C525
P 4100 5700
F 0 "BZ1" H 4250 5750 50  0000 L CNN
F 1 "Buzzer" H 4250 5650 50  0000 L CNN
F 2 "Buzzers_Beepers:Buzzer_12x9.5RM7.6" V 4075 5800 50  0001 C CNN
F 3 "" V 4075 5800 50  0001 C CNN
	1    4100 5700
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5C09F57D
P 4000 5450
F 0 "R3" V 4080 5450 50  0000 C CNN
F 1 "470R" V 4000 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3930 5450 50  0001 C CNN
F 3 "" H 4000 5450 50  0001 C CNN
	1    4000 5450
	1    0    0    -1  
$EndComp
Text GLabel 2000 5400 1    60   Input ~ 0
Start
Text GLabel 2500 5400 1    60   Input ~ 0
Stop
Text GLabel 4000 5300 1    60   Input ~ 0
Buz
$Comp
L LED D2
U 1 1 5C0A4902
P 7350 2100
F 0 "D2" H 7350 2200 50  0000 C CNN
F 1 "LED" H 7350 2000 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 7350 2100 50  0001 C CNN
F 3 "" H 7350 2100 50  0001 C CNN
	1    7350 2100
	-1   0    0    1   
$EndComp
$Comp
L R R7
U 1 1 5C0A496B
P 7650 2100
F 0 "R7" V 7730 2100 50  0000 C CNN
F 1 "360R" V 7650 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7580 2100 50  0001 C CNN
F 3 "" H 7650 2100 50  0001 C CNN
	1    7650 2100
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X06 J2
U 1 1 5C0A50E0
P 8750 2600
F 0 "J2" H 8750 2950 50  0000 C CNN
F 1 "CONN_01X06" V 8850 2600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 8750 2600 50  0001 C CNN
F 3 "" H 8750 2600 50  0001 C CNN
	1    8750 2600
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5C065E34
P 7450 3450
F 0 "R6" V 7400 3600 50  0000 C CNN
F 1 "10K" V 7450 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7380 3450 50  0001 C CNN
F 3 "" H 7450 3450 50  0001 C CNN
	1    7450 3450
	0    -1   1    0   
$EndComp
$Comp
L R R5
U 1 1 5C065E3A
P 7450 3350
F 0 "R5" V 7500 3500 50  0000 C CNN
F 1 "10K" V 7450 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7380 3350 50  0001 C CNN
F 3 "" H 7450 3350 50  0001 C CNN
	1    7450 3350
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 5C065E40
P 7450 3250
F 0 "R4" V 7400 3400 50  0000 C CNN
F 1 "10K" V 7450 3250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7380 3250 50  0001 C CNN
F 3 "" H 7450 3250 50  0001 C CNN
	1    7450 3250
	0    -1   1    0   
$EndComp
$Comp
L CONN_02X03 J1
U 1 1 5C067A3D
P 7000 4000
F 0 "J1" H 7000 4200 50  0000 C CNN
F 1 "CONN_02X03" H 7000 3800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03" H 7000 2800 50  0001 C CNN
F 3 "" H 7000 2800 50  0001 C CNN
	1    7000 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2850 1850 2850 1850
Wire Wire Line
	2750 1850 2750 2000
Wire Wire Line
	2650 1850 2650 1850
Wire Wire Line
	2550 1850 2550 2000
Wire Wire Line
	2150 1925 2450 1925
Wire Wire Line
	2150 1925 2150 1850
Wire Wire Line
	2250 1850 2250 1925
Connection ~ 2250 1925
Wire Wire Line
	2350 1925 2350 1850
Connection ~ 2300 1925
Wire Wire Line
	2450 1925 2450 1850
Connection ~ 2350 1925
Wire Wire Line
	2300 2000 2300 1925
Wire Wire Line
	1850 2025 1850 1850
Wire Wire Line
	2050 2075 2050 1850
Wire Wire Line
	1950 1850 1950 1850
Wire Wire Line
	5499 1850 5800 1850
Wire Wire Line
	5500 2050 5800 2050
Wire Wire Line
	3050 1850 3050 2750
Wire Wire Line
	1950 2550 1950 2575
Wire Wire Line
	5750 1950 5800 1950
Wire Wire Line
	5750 2550 5800 2550
Wire Wire Line
	5800 2450 5500 2450
Wire Wire Line
	5750 2350 5800 2350
Wire Wire Line
	5800 2250 5500 2250
Wire Wire Line
	5750 2150 5800 2150
Wire Wire Line
	4200 2750 4200 4400
Wire Wire Line
	4200 2750 5800 2750
Wire Wire Line
	4450 3400 4450 2850
Wire Wire Line
	4450 2850 5800 2850
Wire Wire Line
	5800 2950 4650 2950
Wire Wire Line
	4650 2950 4650 3400
Wire Wire Line
	5450 3050 5800 3050
Wire Wire Line
	5800 3150 5750 3150
Wire Wire Line
	5450 3250 5800 3250
Wire Wire Line
	5800 3350 5750 3350
Wire Wire Line
	5800 3450 5450 3450
Connection ~ 3000 5800
Connection ~ 2500 5800
Connection ~ 2000 5800
Connection ~ 4000 5800
Wire Wire Line
	8550 2450 8100 2450
Wire Wire Line
	8100 2450 8100 1850
Wire Wire Line
	8100 1850 6800 1850
Wire Wire Line
	6800 2350 8000 2350
Wire Wire Line
	8000 2100 8000 2550
Wire Wire Line
	8000 2550 8550 2550
Wire Wire Line
	7800 2100 8000 2100
Connection ~ 8000 2350
Wire Wire Line
	7600 3450 7600 3250
Wire Wire Line
	6800 3250 7300 3250
Wire Wire Line
	6800 3350 7300 3350
Wire Wire Line
	7000 3750 7000 3350
Connection ~ 7000 3350
Wire Wire Line
	6800 3450 7300 3450
Wire Wire Line
	6900 4250 7100 4250
Connection ~ 7000 4250
$Comp
L CONN_02X02 J3
U 1 1 5C059BFC
P 9000 3000
F 0 "J3" H 9000 3150 50  0000 C CNN
F 1 "CONN_02X02" H 9000 2850 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x02" H 9000 1800 50  0001 C CNN
F 3 "" H 9000 1800 50  0001 C CNN
	1    9000 3000
	-1   0    0    1   
$EndComp
$Comp
L R R8
U 1 1 5C059D0D
P 9400 2950
F 0 "R8" V 9350 2800 50  0000 C CNN
F 1 "8K2" V 9400 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9330 2950 50  0001 C CNN
F 3 "" H 9400 2950 50  0001 C CNN
	1    9400 2950
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 5C05C8FA
P 9400 3050
F 0 "R9" V 9350 2900 50  0000 C CNN
F 1 "8K2" V 9400 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 9330 3050 50  0001 C CNN
F 3 "" H 9400 3050 50  0001 C CNN
	1    9400 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	9550 2950 9550 3050
Text GLabel 3000 5400 1    60   Input ~ 0
WeldCurv
Wire Wire Line
	6800 2950 8750 2950
Wire Wire Line
	7950 2950 7950 2750
Wire Wire Line
	7950 2750 8550 2750
Wire Wire Line
	8550 2850 8100 2850
Wire Wire Line
	8100 2850 8100 3050
Wire Wire Line
	6800 3050 8750 3050
Text GLabel 8550 2350 0    60   Input ~ 0
Vcc_P
Text GLabel 8550 2650 0    60   Input ~ 0
Gnd_P
Wire Wire Line
	2000 5800 4000 5800
Text GLabel 6300 3650 3    60   Input ~ 0
Gnd_P
Text GLabel 7000 4250 3    60   Input ~ 0
Gnd_P
Text GLabel 1550 1850 3    60   Input ~ 0
Gnd_P
Text GLabel 1600 2750 0    60   Input ~ 0
Gnd_P
Text GLabel 2300 2000 3    60   Input ~ 0
Gnd_P
Text GLabel 3050 3150 3    60   Input ~ 0
Gnd_P
Text GLabel 2800 5800 3    60   Input ~ 0
Gnd_P
Text GLabel 4650 4000 3    60   Input ~ 0
Gnd_P
Text GLabel 4550 3400 1    60   Input ~ 0
Gnd_P
$Comp
L POT RV1
U 1 1 5C065731
P 1750 2750
F 0 "RV1" V 1575 2750 50  0000 C CNN
F 1 "10K" V 1750 2750 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_VishaySpectrol-Econtrim-Type36M" H 1750 2750 50  0001 C CNN
F 3 "" H 1750 2750 50  0001 C CNN
	1    1750 2750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 1850 1750 2600
Text GLabel 6300 1650 1    60   Input ~ 0
Vcc_P
Text GLabel 7600 3350 2    60   Input ~ 0
Vcc_P
Text GLabel 7200 2100 0    60   Input ~ 0
Vcc_P
Text GLabel 9550 3000 2    60   Input ~ 0
Vcc_P
Text GLabel 2950 2000 3    60   Input ~ 0
Vcc_P
Text GLabel 1650 2200 3    60   Input ~ 0
Vcc_P
Wire Wire Line
	1650 2200 1650 1850
Text GLabel 1900 2750 2    60   Input ~ 0
Vcc_P
Wire Wire Line
	2950 2000 2950 1850
$Comp
L LED D1
U 1 1 5C05A29F
P 3500 5650
F 0 "D1" H 3500 5750 50  0000 C CNN
F 1 "LED" H 3500 5550 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 3500 5650 50  0001 C CNN
F 3 "" H 3500 5650 50  0000 C CNN
	1    3500 5650
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 5C05A358
P 3500 5350
F 0 "R2" V 3580 5350 50  0000 C CNN
F 1 "470R" V 3500 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3430 5350 50  0001 C CNN
F 3 "" H 3500 5350 50  0000 C CNN
	1    3500 5350
	1    0    0    -1  
$EndComp
Text GLabel 3500 5200 1    60   Input ~ 0
Led
Connection ~ 7950 2950
Connection ~ 8100 3050
$Comp
L Rotary_Encoder_Switch_V2 SW4
U 1 1 5C094BB1
P 4550 3700
F 0 "SW4" H 4550 3960 50  0000 C CNN
F 1 "Rotary_Encoder_Switch_V2" H 4550 3440 50  0001 C CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 4450 3860 50  0001 C CNN
F 3 "" H 4550 3960 50  0001 C CNN
	1    4550 3700
	0    1    1    0   
$EndComp
$Comp
L Q_NPN_EBC Q1
U 1 1 5C09B046
P 2950 2950
F 0 "Q1" H 3150 3000 50  0000 L CNN
F 1 "Q_NPN_CBE" H 3150 2900 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Rugged_Reverse" H 3150 3050 50  0001 C CNN
F 3 "" H 2950 2950 50  0000 C CNN
	1    2950 2950
	1    0    0    -1  
$EndComp
Connection ~ 3500 5800
Wire Wire Line
	4450 4400 4450 4000
Wire Wire Line
	4200 4400 4450 4400
Wire Wire Line
	6900 3450 6900 3750
Connection ~ 6900 3450
Wire Wire Line
	7100 3750 7100 3250
Connection ~ 7100 3250
$Comp
L CP C5
U 1 1 5CE7B713
P 9400 2100
F 0 "C5" H 9425 2200 50  0000 L CNN
F 1 "10uF" H 9425 2000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P5.00mm" H 9438 1950 50  0001 C CNN
F 3 "" H 9400 2100 50  0001 C CNN
	1    9400 2100
	1    0    0    -1  
$EndComp
Text GLabel 9400 1950 1    60   Input ~ 0
Vcc_P
Text GLabel 9400 2250 3    60   Input ~ 0
Gnd_P
$EndSCHEMATC
