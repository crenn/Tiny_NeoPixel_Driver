EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:BOOST Convertor
LIBS:pnp
LIBS:NeoPixTinyDriver-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "25 jun 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L INDUCTOR L1
U 1 1 53115186
P 10100 2100
F 0 "L1" V 10050 2100 40  0000 C CNN
F 1 "6u8" V 10200 2100 40  0000 C CNN
F 2 "~" H 10100 2100 60  0000 C CNN
F 3 "~" H 10100 2100 60  0000 C CNN
	1    10100 2100
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 53115A6A
P 9800 2600
F 0 "C3" H 9800 2700 40  0000 L CNN
F 1 "10u" H 9806 2515 40  0000 L CNN
F 2 "~" H 9838 2450 30  0000 C CNN
F 3 "~" H 9800 2600 60  0000 C CNN
	1    9800 2600
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 53115A79
P 7050 2300
F 0 "C1" H 7050 2400 40  0000 L CNN
F 1 "47u" H 7056 2215 40  0000 L CNN
F 2 "~" H 7088 2150 30  0000 C CNN
F 3 "~" H 7050 2300 60  0000 C CNN
	1    7050 2300
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 53115A88
P 7350 2300
F 0 "C2" H 7350 2400 40  0000 L CNN
F 1 "2u2" H 7356 2215 40  0000 L CNN
F 2 "~" H 7388 2150 30  0000 C CNN
F 3 "~" H 7350 2300 60  0000 C CNN
	1    7350 2300
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 53115AB5
P 10800 2350
F 0 "R5" V 10880 2350 40  0000 C CNN
F 1 "R" V 10807 2351 40  0000 C CNN
F 2 "~" V 10730 2350 30  0000 C CNN
F 3 "~" H 10800 2350 30  0000 C CNN
	1    10800 2350
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 53115AC4
P 10800 2950
F 0 "R6" V 10880 2950 40  0000 C CNN
F 1 "R" V 10807 2951 40  0000 C CNN
F 2 "~" V 10730 2950 30  0000 C CNN
F 3 "~" H 10800 2950 30  0000 C CNN
	1    10800 2950
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 53115D3D
P 4000 3350
F 0 "SW2" H 4150 3460 50  0000 C CNN
F 1 "SW_PUSH" H 4000 3270 50  0000 C CNN
F 2 "~" H 4000 3350 60  0000 C CNN
F 3 "~" H 4000 3350 60  0000 C CNN
	1    4000 3350
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 53115D4C
P 4000 2900
F 0 "SW1" H 4150 3010 50  0000 C CNN
F 1 "SW_PUSH" H 4000 2820 50  0000 C CNN
F 2 "~" H 4000 2900 60  0000 C CNN
F 3 "~" H 4000 2900 60  0000 C CNN
	1    4000 2900
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR01
U 1 1 53115F31
P 6750 2050
F 0 "#PWR01" H 6750 2140 20  0001 C CNN
F 1 "+5V" H 6750 2140 30  0000 C CNN
F 2 "" H 6750 2050 60  0000 C CNN
F 3 "" H 6750 2050 60  0000 C CNN
	1    6750 2050
	1    0    0    -1  
$EndComp
$Comp
L +1.2V #PWR02
U 1 1 53115F41
P 10650 2000
F 0 "#PWR02" H 10650 2140 20  0001 C CNN
F 1 "+1.2V" H 10650 2110 30  0000 C CNN
F 2 "" H 10650 2000 60  0000 C CNN
F 3 "" H 10650 2000 60  0000 C CNN
	1    10650 2000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR03
U 1 1 53115F5A
P 11000 4850
F 0 "#PWR03" H 11000 4940 20  0001 C CNN
F 1 "+5V" H 11000 4940 30  0000 C CNN
F 2 "" H 11000 4850 60  0000 C CNN
F 3 "" H 11000 4850 60  0000 C CNN
	1    11000 4850
	1    0    0    -1  
$EndComp
Connection ~ 7350 2100
Connection ~ 7050 2100
Wire Wire Line
	9500 2100 9800 2100
Wire Wire Line
	10650 2400 9500 2400
Wire Wire Line
	10650 2000 10650 2400
Connection ~ 9800 2400
Wire Wire Line
	10400 2100 10800 2100
Connection ~ 10650 2100
Wire Wire Line
	10800 2600 10800 2700
Wire Wire Line
	10800 2650 10550 2650
Wire Wire Line
	10550 2650 10550 2300
Wire Wire Line
	10550 2300 9500 2300
Connection ~ 10800 2650
Wire Wire Line
	6900 2000 8350 2000
Wire Wire Line
	8250 2000 8250 1800
Connection ~ 8250 2000
$Comp
L +1.2V #PWR04
U 1 1 531EF037
P 7800 1200
F 0 "#PWR04" H 7800 1340 20  0001 C CNN
F 1 "+1.2V" H 7800 1310 30  0000 C CNN
F 2 "" H 7800 1200 60  0000 C CNN
F 3 "" H 7800 1200 60  0000 C CNN
	1    7800 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 1300 8250 1300
Wire Wire Line
	7800 1300 7800 1200
Wire Wire Line
	8250 1300 8250 1400
Connection ~ 7800 1300
Text Notes 3650 2700 0    60   ~ 0
Power ON/OFF
Text Notes 3700 3150 0    60   ~ 0
Mode Select
$Comp
L GND #PWR05
U 1 1 531EF1D8
P 11000 5500
F 0 "#PWR05" H 11000 5500 30  0001 C CNN
F 1 "GND" H 11000 5430 30  0001 C CNN
F 2 "" H 11000 5500 60  0000 C CNN
F 3 "" H 11000 5500 60  0000 C CNN
	1    11000 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10700 5450 11000 5450
Wire Wire Line
	11000 5450 11000 5500
$Comp
L GND #PWR06
U 1 1 531EF20B
P 10800 3300
F 0 "#PWR06" H 10800 3300 30  0001 C CNN
F 1 "GND" H 10800 3230 30  0001 C CNN
F 2 "" H 10800 3300 60  0000 C CNN
F 3 "" H 10800 3300 60  0000 C CNN
	1    10800 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 531EF239
P 7650 3300
F 0 "#PWR07" H 7650 3300 30  0001 C CNN
F 1 "GND" H 7650 3230 30  0001 C CNN
F 2 "" H 7650 3300 60  0000 C CNN
F 3 "" H 7650 3300 60  0000 C CNN
	1    7650 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 3200 10800 3300
Wire Wire Line
	9800 2800 9800 3250
Wire Wire Line
	9650 3250 10800 3250
Connection ~ 10800 3250
Wire Wire Line
	10700 4950 11000 4950
Wire Wire Line
	11000 4950 11000 4850
Wire Wire Line
	3700 2900 3550 2900
Text GLabel 4750 2900 2    60   Input ~ 0
Power_SIG
Wire Wire Line
	4300 2900 4750 2900
Text GLabel 6200 5450 0    60   Input ~ 0
Power_SIG
Text GLabel 6100 2200 0    60   Input ~ 0
Power_SIG
Text GLabel 7800 1600 0    60   Input ~ 0
MCUPWR_SIG
Wire Wire Line
	7800 1600 7950 1600
Wire Wire Line
	6900 1300 6900 1400
Wire Wire Line
	6900 1800 6900 2450
Wire Wire Line
	4300 3350 4750 3350
Text GLabel 7400 4950 0    60   Input ~ 0
MODE_SIG
Wire Wire Line
	6200 5450 6400 5450
Wire Wire Line
	7350 2500 7350 3200
Wire Wire Line
	7050 2500 7050 3200
Connection ~ 7350 3200
Wire Wire Line
	8350 2400 8050 2400
Wire Wire Line
	8050 2400 8050 3200
Wire Wire Line
	9500 2000 9650 2000
Wire Wire Line
	9650 2000 9650 3250
Connection ~ 9800 3250
Wire Wire Line
	8050 3200 6900 3200
Wire Wire Line
	7650 3200 7650 3300
Connection ~ 7650 3200
$Comp
L CONN_6 P2
U 1 1 53376E3E
P 2600 2500
F 0 "P2" V 2550 2500 60  0000 C CNN
F 1 "CONN_6" V 2650 2500 60  0000 C CNN
F 2 "" H 2600 2500 60  0000 C CNN
F 3 "" H 2600 2500 60  0000 C CNN
	1    2600 2500
	1    0    0    -1  
$EndComp
$Comp
L CONN_3X2 P1
U 1 1 53376E6B
P 2050 1450
F 0 "P1" H 2050 1700 50  0000 C CNN
F 1 "CONN_3X2" V 2050 1500 40  0000 C CNN
F 2 "" H 2050 1450 60  0000 C CNN
F 3 "" H 2050 1450 60  0000 C CNN
	1    2050 1450
	1    0    0    -1  
$EndComp
$Comp
L ATTINY85-S IC1
U 1 1 53376EF4
P 9350 5200
F 0 "IC1" H 8300 5600 60  0000 C CNN
F 1 "ATTINY85-S" H 10200 4800 60  0000 C CNN
F 2 "SO8-200" H 8450 4800 60  0001 C CNN
F 3 "" H 9350 5200 60  0000 C CNN
	1    9350 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 533777B7
P 1650 2300
F 0 "#PWR08" H 1650 2300 30  0001 C CNN
F 1 "GND" H 1650 2230 30  0001 C CNN
F 2 "" H 1650 2300 60  0000 C CNN
F 3 "" H 1650 2300 60  0000 C CNN
	1    1650 2300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR09
U 1 1 533777D0
P 2050 1950
F 0 "#PWR09" H 2050 2040 20  0001 C CNN
F 1 "+5V" H 2050 2040 30  0000 C CNN
F 2 "" H 2050 1950 60  0000 C CNN
F 3 "" H 2050 1950 60  0000 C CNN
	1    2050 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1950 2050 2450
Wire Wire Line
	2050 2450 2250 2450
Wire Wire Line
	2250 2250 1650 2250
Wire Wire Line
	1650 2250 1650 2300
$Comp
L R R3
U 1 1 53378F27
P 6900 2700
F 0 "R3" V 6980 2700 40  0000 C CNN
F 1 "1M" V 6907 2701 40  0000 C CNN
F 2 "~" V 6830 2700 30  0000 C CNN
F 3 "~" H 6900 2700 30  0000 C CNN
	1    6900 2700
	1    0    0    -1  
$EndComp
Connection ~ 6900 2000
Wire Wire Line
	6900 3200 6900 2950
Connection ~ 7050 3200
Text GLabel 8100 2300 0    60   Input ~ 0
LOW_BATT
Wire Wire Line
	8350 2300 8100 2300
Text GLabel 2100 2550 0    60   Input ~ 0
UART_Rx
Text GLabel 1600 2650 0    60   Input ~ 0
UART_Tx
Wire Wire Line
	1600 2650 2250 2650
Wire Wire Line
	2100 2550 2250 2550
NoConn ~ 2250 2350
NoConn ~ 2250 2750
Text GLabel 7400 5350 0    60   Input ~ 0
LOW_BATT
Wire Wire Line
	7400 5350 8000 5350
Text GLabel 7400 5150 0    60   Input ~ 0
STRING_1
Text GLabel 6200 5050 0    60   Input ~ 0
STRING_2
Text GLabel 6200 5250 0    60   Input ~ 0
MCUPWR_SIG
Wire Wire Line
	6200 5250 8000 5250
Connection ~ 6700 5250
$Comp
L GND #PWR014
U 1 1 5337921B
P 6700 5700
F 0 "#PWR014" H 6700 5700 30  0001 C CNN
F 1 "GND" H 6700 5630 30  0001 C CNN
F 2 "" H 6700 5700 60  0000 C CNN
F 3 "" H 6700 5700 60  0000 C CNN
	1    6700 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5650 6700 5700
Wire Wire Line
	7400 5150 8000 5150
Wire Wire Line
	6200 5050 8000 5050
Text GLabel 6200 4700 0    60   Input ~ 0
UART_Tx
Text GLabel 7400 4700 0    60   Input ~ 0
UART_Rx
Wire Wire Line
	6200 4700 6550 4700
Wire Wire Line
	6550 4700 6550 5050
Connection ~ 6550 5050
Wire Wire Line
	7400 4950 8000 4950
Wire Wire Line
	7500 4950 7500 4700
Wire Wire Line
	7500 4700 7400 4700
Connection ~ 7500 4950
Text GLabel 4750 3350 2    60   Input ~ 0
MODE_SIG
$Comp
L GND #PWR015
U 1 1 533794AF
P 3550 3450
F 0 "#PWR015" H 3550 3450 30  0001 C CNN
F 1 "GND" H 3550 3380 30  0001 C CNN
F 2 "" H 3550 3450 60  0000 C CNN
F 3 "" H 3550 3450 60  0000 C CNN
	1    3550 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3350 3550 3350
Wire Wire Line
	3550 2900 3550 3450
NoConn ~ 8350 2200
Text Notes 2100 850  0    60   ~ 0
Vcc
Text Notes 2100 950  0    60   ~ 0
MOSI
Text Notes 2100 1050 0    60   ~ 0
GND
Text Notes 1800 1050 0    60   ~ 0
RST
Text Notes 1800 950  0    60   ~ 0
SCK
Text Notes 1800 850  0    60   ~ 0
MISO
$Comp
L R R4
U 1 1 5337972C
P 7800 4450
F 0 "R4" V 7880 4450 40  0000 C CNN
F 1 "100K" V 7807 4451 40  0000 C CNN
F 2 "~" V 7730 4450 30  0000 C CNN
F 3 "~" H 7800 4450 30  0000 C CNN
	1    7800 4450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR016
U 1 1 53379995
P 7800 4100
F 0 "#PWR016" H 7800 4190 20  0001 C CNN
F 1 "+5V" H 7800 4190 30  0000 C CNN
F 2 "" H 7800 4100 60  0000 C CNN
F 3 "" H 7800 4100 60  0000 C CNN
	1    7800 4100
	1    0    0    -1  
$EndComp
Text GLabel 7550 5550 0    60   Input ~ 0
RESET
Wire Wire Line
	7800 4700 7800 5550
Wire Wire Line
	7800 5450 8000 5450
Wire Wire Line
	7800 5550 7550 5550
Connection ~ 7800 5450
Wire Wire Line
	7800 4200 7800 4100
Text GLabel 1550 1500 0    60   Input ~ 0
RESET
Text GLabel 1550 1300 0    60   Input ~ 0
UART_Tx
Wire Notes Line
	1750 750  2350 750 
Wire Notes Line
	2350 750  2350 1050
Wire Notes Line
	2350 1050 1750 1050
Wire Notes Line
	1750 1050 1750 750 
Wire Notes Line
	2050 750  2050 1050
Wire Notes Line
	1750 850  2350 850 
Wire Notes Line
	1750 950  2350 950 
Text GLabel 2550 1400 2    60   Input ~ 0
UART_Rx
Text GLabel 1050 1400 0    60   Input ~ 0
STRING_1
Wire Wire Line
	1550 1500 1650 1500
Wire Wire Line
	1550 1300 1650 1300
Wire Wire Line
	1050 1400 1650 1400
Wire Wire Line
	2550 1400 2450 1400
$Comp
L +5V #PWR017
U 1 1 53379E4D
P 2550 1200
F 0 "#PWR017" H 2550 1290 20  0001 C CNN
F 1 "+5V" H 2550 1290 30  0000 C CNN
F 2 "" H 2550 1200 60  0000 C CNN
F 3 "" H 2550 1200 60  0000 C CNN
	1    2550 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 53379E53
P 2550 1600
F 0 "#PWR018" H 2550 1600 30  0001 C CNN
F 1 "GND" H 2550 1530 30  0001 C CNN
F 2 "" H 2550 1600 60  0000 C CNN
F 3 "" H 2550 1600 60  0000 C CNN
	1    2550 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1600 2550 1500
Wire Wire Line
	2550 1500 2450 1500
Wire Wire Line
	2550 1200 2550 1300
Wire Wire Line
	2550 1300 2450 1300
Wire Bus Line
	3300 3650 3300 450 
Wire Bus Line
	3300 3650 11250 3650
Wire Bus Line
	3300 2350 5400 2350
Wire Bus Line
	3300 2950 450  2950
Text Notes 650  650  0    100  ~ 20
Programming Interfaces
Text Notes 3450 700  0    100  ~ 20
WS281x Strings
Text Notes 3400 2550 0    100  ~ 20
User Buttons
Text Notes 5500 650  0    100  ~ 20
Main BOOST Supplier
Text Notes 5500 3850 0    100  ~ 20
Microcontroller
Wire Notes Line
	5400 450  5400 5850
Wire Notes Line
	5400 5850 11250 5850
$Comp
L CONN_2 P7
U 1 1 5337A1CC
P 4600 4275
F 0 "P7" V 4550 4275 40  0000 C CNN
F 1 "CONN_2" V 4650 4275 40  0000 C CNN
F 2 "" H 4600 4275 60  0000 C CNN
F 3 "" H 4600 4275 60  0000 C CNN
	1    4600 4275
	1    0    0    -1  
$EndComp
$Comp
L +1.2V #PWR019
U 1 1 5337A1D9
P 4100 4075
F 0 "#PWR019" H 4100 4215 20  0001 C CNN
F 1 "+1.2V" H 4100 4185 30  0000 C CNN
F 2 "" H 4100 4075 60  0000 C CNN
F 3 "" H 4100 4075 60  0000 C CNN
	1    4100 4075
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5337A1DF
P 4100 4475
F 0 "#PWR020" H 4100 4475 30  0001 C CNN
F 1 "GND" H 4100 4405 30  0001 C CNN
F 2 "" H 4100 4475 60  0000 C CNN
F 3 "" H 4100 4475 60  0000 C CNN
	1    4100 4475
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4075 4100 4175
Wire Wire Line
	4100 4175 4250 4175
Wire Wire Line
	4100 4475 4100 4375
Wire Wire Line
	4100 4375 4250 4375
Text Notes 3350 3850 0    100  ~ 20
Battery Supply
Wire Notes Line
	3300 3650 3300 4750
Wire Notes Line
	3300 4750 5400 4750
$Comp
L TPS61027 U1
U 1 1 5337A38E
P 8650 1900
F 0 "U1" H 8700 1950 60  0000 C CNN
F 1 "TPS61027" H 8950 1250 60  0000 C CNN
F 2 "~" H 8650 1700 60  0000 C CNN
F 3 "~" H 8650 1700 60  0000 C CNN
	1    8650 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 2200 9650 2200
Connection ~ 9650 2200
$Comp
L R R2
U 1 1 538C5F19
P 6650 1300
F 0 "R2" V 6730 1300 40  0000 C CNN
F 1 "1M" V 6657 1301 40  0000 C CNN
F 2 "~" V 6580 1300 30  0000 C CNN
F 3 "~" H 6650 1300 30  0000 C CNN
	1    6650 1300
	0    -1   -1   0   
$EndComp
Connection ~ 3550 3350
$Comp
L R R1
U 1 1 538C6010
P 6400 1950
F 0 "R1" V 6480 1950 40  0000 C CNN
F 1 "1K" V 6407 1951 40  0000 C CNN
F 2 "~" V 6330 1950 30  0000 C CNN
F 3 "~" H 6400 1950 30  0000 C CNN
	1    6400 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	6400 2200 6100 2200
Wire Wire Line
	6750 2050 6750 2100
Wire Wire Line
	6750 2100 8350 2100
Wire Wire Line
	6400 1300 6400 1700
Wire Wire Line
	6600 1600 6400 1600
Connection ~ 6400 1600
$Comp
L CONN_1 P3
U 1 1 538F0F60
P 4000 5250
F 0 "P3" H 4080 5250 40  0000 L CNN
F 1 "CONN_1" H 4000 5305 30  0001 C CNN
F 2 "" H 4000 5250 60  0000 C CNN
F 3 "" H 4000 5250 60  0000 C CNN
	1    4000 5250
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P4
U 1 1 538F0F6F
P 4000 5400
F 0 "P4" H 4080 5400 40  0000 L CNN
F 1 "CONN_1" H 4000 5455 30  0001 C CNN
F 2 "" H 4000 5400 60  0000 C CNN
F 3 "" H 4000 5400 60  0000 C CNN
	1    4000 5400
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P5
U 1 1 538F0F7E
P 4000 5550
F 0 "P5" H 4080 5550 40  0000 L CNN
F 1 "CONN_1" H 4000 5605 30  0001 C CNN
F 2 "" H 4000 5550 60  0000 C CNN
F 3 "" H 4000 5550 60  0000 C CNN
	1    4000 5550
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P6
U 1 1 538F0F8D
P 4000 5700
F 0 "P6" H 4080 5700 40  0000 L CNN
F 1 "CONN_1" H 4000 5755 30  0001 C CNN
F 2 "" H 4000 5700 60  0000 C CNN
F 3 "" H 4000 5700 60  0000 C CNN
	1    4000 5700
	1    0    0    -1  
$EndComp
Text GLabel 3550 5550 0    60   Input ~ 0
MCUPWR_SIG
$Comp
L +1.2V #PWR021
U 1 1 538F0F9B
P 3700 5150
F 0 "#PWR021" H 3700 5290 20  0001 C CNN
F 1 "+1.2V" H 3700 5260 30  0000 C CNN
F 2 "" H 3700 5150 60  0000 C CNN
F 3 "" H 3700 5150 60  0000 C CNN
	1    3700 5150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR022
U 1 1 538F0FA1
P 3500 5150
F 0 "#PWR022" H 3500 5240 20  0001 C CNN
F 1 "+5V" H 3500 5240 30  0000 C CNN
F 2 "" H 3500 5150 60  0000 C CNN
F 3 "" H 3500 5150 60  0000 C CNN
	1    3500 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5250 3700 5250
Wire Wire Line
	3700 5250 3700 5150
Wire Wire Line
	3850 5400 3500 5400
Wire Wire Line
	3500 5400 3500 5150
Wire Wire Line
	3850 5550 3550 5550
$Comp
L GND #PWR023
U 1 1 538F1128
P 3700 5800
F 0 "#PWR023" H 3700 5800 30  0001 C CNN
F 1 "GND" H 3700 5730 30  0001 C CNN
F 2 "" H 3700 5800 60  0000 C CNN
F 3 "" H 3700 5800 60  0000 C CNN
	1    3700 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5800 3700 5700
Wire Wire Line
	3700 5700 3850 5700
$Comp
L MBT3906 Q1
U 1 1 538F1F3F
P 6600 5450
F 0 "Q1" H 6600 5300 60  0000 R CNN
F 1 "MBT3906" H 6600 5600 60  0000 R CNN
F 2 "~" H 6600 5450 60  0000 C CNN
F 3 "~" H 6600 5450 60  0000 C CNN
	1    6600 5450
	1    0    0    -1  
$EndComp
$Comp
L MBT3906 Q1
U 2 1 538F1F4E
P 6800 1600
F 0 "Q1" H 6800 1450 60  0000 R CNN
F 1 "MBT3906" H 6800 1750 60  0000 R CNN
F 2 "~" H 6800 1600 60  0000 C CNN
F 3 "~" H 6800 1600 60  0000 C CNN
	2    6800 1600
	1    0    0    -1  
$EndComp
$Comp
L MOS_N Q3
U 1 1 53AAA67A
P 8150 1600
F 0 "Q3" H 8160 1770 60  0000 R CNN
F 1 "MOS_N" H 8160 1450 60  0000 R CNN
F 2 "~" H 8150 1600 60  0000 C CNN
F 3 "~" H 8150 1600 60  0000 C CNN
	1    8150 1600
	1    0    0    -1  
$EndComp
$EndSCHEMATC