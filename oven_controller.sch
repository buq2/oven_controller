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
LIBS:buq2-kicad-components
LIBS:oven_controller-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BUQ2-ATXMEGA128A4U-A IC1
U 1 1 54BE8389
P 4550 3300
F 0 "IC1" H 3800 4500 40  0000 L BNN
F 1 "BUQ2-ATXMEGA128A4U-A" H 4900 1950 40  0000 L BNN
F 2 "buq2:BUQ2-TQFP44" H 4550 3300 35  0000 C CIN
F 3 "" H 4550 3300 60  0000 C CNN
	1    4550 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 54BE83D9
P 4550 4900
F 0 "#PWR01" H 4550 4900 30  0001 C CNN
F 1 "GND" H 4550 4830 30  0001 C CNN
F 2 "" H 4550 4900 60  0000 C CNN
F 3 "" H 4550 4900 60  0000 C CNN
	1    4550 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 4900 4550 4800
Wire Wire Line
	4400 4800 4700 4800
Wire Wire Line
	4700 4800 4700 4700
Wire Wire Line
	4600 4700 4600 4800
Connection ~ 4600 4800
Wire Wire Line
	4500 4800 4500 4700
Connection ~ 4550 4800
Wire Wire Line
	4400 4800 4400 4700
Connection ~ 4500 4800
$Comp
L C C3
U 1 1 54BE845D
P 4250 1400
F 0 "C3" H 4250 1500 40  0000 L CNN
F 1 "100n" H 4256 1315 40  0000 L CNN
F 2 "buq2:SM0603" H 4288 1250 30  0001 C CNN
F 3 "" H 4250 1400 60  0000 C CNN
	1    4250 1400
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 54BE84AE
P 4500 1400
F 0 "C4" H 4500 1500 40  0000 L CNN
F 1 "100n" H 4506 1315 40  0000 L CNN
F 2 "buq2:SM0603" H 4538 1250 30  0001 C CNN
F 3 "" H 4500 1400 60  0000 C CNN
	1    4500 1400
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 54BE84CB
P 4750 1400
F 0 "C5" H 4750 1500 40  0000 L CNN
F 1 "100n" H 4756 1315 40  0000 L CNN
F 2 "buq2:SM0603" H 4788 1250 30  0001 C CNN
F 3 "" H 4750 1400 60  0000 C CNN
	1    4750 1400
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 54BE84F5
P 5000 1400
F 0 "C6" H 5000 1500 40  0000 L CNN
F 1 "100n" H 5006 1315 40  0000 L CNN
F 2 "buq2:SM0603" H 5038 1250 30  0001 C CNN
F 3 "" H 5000 1400 60  0000 C CNN
	1    5000 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2000 4450 1800
Wire Wire Line
	4250 1800 5000 1800
Wire Wire Line
	4250 1800 4250 1600
Wire Wire Line
	4500 1800 4500 1600
Connection ~ 4450 1800
Wire Wire Line
	4750 1800 4750 1600
Connection ~ 4500 1800
Wire Wire Line
	5000 1800 5000 1600
Connection ~ 4750 1800
Wire Wire Line
	4550 2000 4550 1800
Connection ~ 4550 1800
Wire Wire Line
	4650 2000 4650 1800
Connection ~ 4650 1800
Wire Wire Line
	4850 2000 4850 1800
Connection ~ 4850 1800
Wire Wire Line
	4250 1200 5000 1200
Connection ~ 4500 1200
Connection ~ 4750 1200
$Comp
L GND #PWR02
U 1 1 54BE863C
P 4650 950
F 0 "#PWR02" H 4650 950 30  0001 C CNN
F 1 "GND" H 4650 880 30  0001 C CNN
F 2 "" H 4650 950 60  0000 C CNN
F 3 "" H 4650 950 60  0000 C CNN
	1    4650 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 950  4650 1200
Connection ~ 4650 1200
Text Label 4250 1800 2    60   ~ 0
VDD
$Comp
L TAG-CONNECT-ATMEL-PDI-CONNECTOR U3
U 1 1 54BE86A1
P 2050 1400
F 0 "U3" H 2050 1000 60  0000 C CNN
F 1 "TAG-CONNECT-ATMEL-PDI-CONNECTOR" H 2050 1850 60  0000 C CNN
F 2 "buq2:TAG-CONNECT6" H 2100 1250 60  0001 C CNN
F 3 "" H 2100 1250 60  0000 C CNN
	1    2050 1400
	1    0    0    -1  
$EndComp
Text Label 3650 2300 2    60   ~ 0
PDI_CLK
Text Label 3650 2400 2    60   ~ 0
PDI_DATA
Text Label 1500 1250 2    60   ~ 0
PDI_DATA
Text Label 1500 1550 2    60   ~ 0
PDI_CLK
Text Label 2550 1250 0    60   ~ 0
VDD
$Comp
L GND #PWR03
U 1 1 54BE87A9
P 2650 1550
F 0 "#PWR03" H 2650 1550 30  0001 C CNN
F 1 "GND" H 2650 1480 30  0001 C CNN
F 2 "" H 2650 1550 60  0000 C CNN
F 3 "" H 2650 1550 60  0000 C CNN
	1    2650 1550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2650 1550 2550 1550
NoConn ~ 2550 1400
NoConn ~ 1500 1400
Text Label 3650 4200 2    60   ~ 0
D-
Text Label 3650 4300 2    60   ~ 0
D+
Text Label 6350 3700 0    60   ~ 0
CSn_FLASH
Text Label 6350 4100 0    60   ~ 0
MOSI
Text Label 6350 4300 0    60   ~ 0
SCK
Text Label 6350 4200 0    60   ~ 0
MISO
$Comp
L MAX6675ISA+ U2
U 1 1 54BE90F4
P 1650 6850
F 0 "U2" H 1650 6350 60  0000 C CNN
F 1 "MAX6675ISA+" H 1650 7250 60  0000 C CNN
F 2 "buq2:8-SOIC" H 1700 6600 60  0001 C CNN
F 3 "" H 1700 6600 60  0000 C CNN
	1    1650 6850
	1    0    0    -1  
$EndComp
NoConn ~ 2100 6650
Text Label 1150 7100 2    60   ~ 0
VDD
Text Label 1250 6950 2    60   ~ 0
T+0
Text Label 1250 6800 2    60   ~ 0
T-0
$Comp
L GND #PWR04
U 1 1 54BE9181
P 1100 6650
F 0 "#PWR04" H 1100 6650 30  0001 C CNN
F 1 "GND" H 1100 6580 30  0001 C CNN
F 2 "" H 1100 6650 60  0000 C CNN
F 3 "" H 1100 6650 60  0000 C CNN
	1    1100 6650
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 6650 1250 6650
Text Label 2100 6950 0    60   ~ 0
nCS_THERMO0
Text Label 6350 3900 0    60   ~ 0
nCS_THERMO0
Text Label 2100 6800 0    60   ~ 0
MISO
Text Label 2100 7100 0    60   ~ 0
SCK
$Comp
L BUQ2-USB-MICRO-B CON1
U 1 1 54BE94B8
P 1750 3850
F 0 "CON1" H 1500 4300 60  0000 C CNN
F 1 "BUQ2-USB-MICRO-B" H 1700 3350 60  0000 C CNN
F 2 "buq2:USB-MICRO-B-FCI-10118194" H 1800 3150 60  0001 C CNN
F 3 "http://portal.fciconnect.com/Comergent//fci/drawing/10118193.pdf" H 2200 3250 60  0001 C CNN
F 4 "609-4616-1-ND" H 1900 4400 60  0001 C CNN "Digikey"
	1    1750 3850
	1    0    0    -1  
$EndComp
$Comp
L VR VR1
U 1 1 54BE9510
P 750 3450
F 0 "VR1" V 810 3404 40  0000 C TNN
F 1 "VR" V 750 3450 40  0000 C CNN
F 2 "buq2:SM0603" H 750 3450 60  0001 C CNN
F 3 "" H 750 3450 60  0000 C CNN
	1    750  3450
	1    0    0    -1  
$EndComp
$Comp
L VR VR2
U 1 1 54BE9560
P 750 4100
F 0 "VR2" V 810 4054 40  0000 C TNN
F 1 "VR" V 750 4100 40  0000 C CNN
F 2 "buq2:SM0603" H 750 4100 60  0001 C CNN
F 3 "" H 750 4100 60  0000 C CNN
	1    750  4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 3700 750  3700
Wire Wire Line
	1200 3850 750  3850
$Comp
L GND #PWR05
U 1 1 54BE9612
P 750 4450
F 0 "#PWR05" H 750 4450 30  0001 C CNN
F 1 "GND" H 750 4380 30  0001 C CNN
F 2 "" H 750 4450 60  0000 C CNN
F 3 "" H 750 4450 60  0000 C CNN
	1    750  4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  4450 750  4350
$Comp
L GND #PWR06
U 1 1 54BE965D
P 750 3100
F 0 "#PWR06" H 750 3100 30  0001 C CNN
F 1 "GND" H 750 3030 30  0001 C CNN
F 2 "" H 750 3100 60  0000 C CNN
F 3 "" H 750 3100 60  0000 C CNN
	1    750  3100
	-1   0    0    1   
$EndComp
Wire Wire Line
	750  3100 750  3200
Text Label 750  3700 2    60   ~ 0
D-
Text Label 750  3850 2    60   ~ 0
D+
$Comp
L GND #PWR07
U 1 1 54BE9755
P 1050 4150
F 0 "#PWR07" H 1050 4150 30  0001 C CNN
F 1 "GND" H 1050 4080 30  0001 C CNN
F 2 "" H 1050 4150 60  0000 C CNN
F 3 "" H 1050 4150 60  0000 C CNN
	1    1050 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 4150 1200 4150
NoConn ~ 1200 4000
$Comp
L GND #PWR08
U 1 1 54BE97A4
P 2400 3850
F 0 "#PWR08" H 2400 3850 30  0001 C CNN
F 1 "GND" H 2400 3780 30  0001 C CNN
F 2 "" H 2400 3850 60  0000 C CNN
F 3 "" H 2400 3850 60  0000 C CNN
	1    2400 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2400 3850 2300 3850
Text Label 1200 3550 2    60   ~ 0
VBUS
$Comp
L FLASH_MEMORY U1
U 1 1 54BE9965
P 1450 5350
F 0 "U1" H 1450 5650 60  0000 C CNN
F 1 "FLASH_MEMORY" H 1500 5000 60  0000 C CNN
F 2 "buq2:8-SOIC" H 1350 5200 60  0001 C CNN
F 3 "" H 1350 5200 60  0000 C CNN
	1    1450 5350
	1    0    0    -1  
$EndComp
Text Label 950  5200 2    60   ~ 0
CSn_FLASH
Text Label 950  5300 2    60   ~ 0
MISO
$Comp
L C C2
U 1 1 54BE9A83
P 2100 5000
F 0 "C2" H 2100 5100 40  0000 L CNN
F 1 "100n" H 2106 4915 40  0000 L CNN
F 2 "buq2:SM0603" H 2138 4850 30  0001 C CNN
F 3 "" H 2100 5000 60  0000 C CNN
	1    2100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 5200 2100 5200
Text Label 2100 5200 0    60   ~ 0
VDD
$Comp
L GND #PWR09
U 1 1 54BE9B06
P 2100 4750
F 0 "#PWR09" H 2100 4750 30  0001 C CNN
F 1 "GND" H 2100 4680 30  0001 C CNN
F 2 "" H 2100 4750 60  0000 C CNN
F 3 "" H 2100 4750 60  0000 C CNN
	1    2100 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 4800 2100 4750
Text Label 2000 5400 0    60   ~ 0
SCK
Text Label 2000 5500 0    60   ~ 0
MOSI
$Comp
L GND #PWR010
U 1 1 54BE9BF6
P 850 5500
F 0 "#PWR010" H 850 5500 30  0001 C CNN
F 1 "GND" H 850 5430 30  0001 C CNN
F 2 "" H 850 5500 60  0000 C CNN
F 3 "" H 850 5500 60  0000 C CNN
	1    850  5500
	0    1    1    0   
$EndComp
Wire Wire Line
	850  5500 950  5500
Wire Wire Line
	950  5400 750  5400
Wire Wire Line
	750  5400 750  5850
Wire Wire Line
	750  5850 2350 5850
Wire Wire Line
	2350 5850 2350 5300
Wire Wire Line
	2350 5300 2000 5300
Wire Wire Line
	2100 5200 2100 5300
Connection ~ 2100 5300
$Comp
L C C1
U 1 1 54BE9E8A
P 1200 7300
F 0 "C1" H 1200 7400 40  0000 L CNN
F 1 "100n" H 1206 7215 40  0000 L CNN
F 2 "buq2:SM0603" H 1238 7150 30  0001 C CNN
F 3 "" H 1200 7300 60  0000 C CNN
	1    1200 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 7100 1250 7100
Connection ~ 1200 7100
$Comp
L GND #PWR011
U 1 1 54BE9FE1
P 1200 7600
F 0 "#PWR011" H 1200 7600 30  0001 C CNN
F 1 "GND" H 1200 7530 30  0001 C CNN
F 2 "" H 1200 7600 60  0000 C CNN
F 3 "" H 1200 7600 60  0000 C CNN
	1    1200 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 7600 1200 7500
$Comp
L CONN_2 P1
U 1 1 54BEA1D6
P 4100 7550
F 0 "P1" V 4050 7550 40  0000 C CNN
F 1 "CONN_THERMO0" V 4150 7550 40  0000 C CNN
F 2 "buq2:PIN_HEADER_2" H 4100 7550 60  0001 C CNN
F 3 "" H 4100 7550 60  0000 C CNN
	1    4100 7550
	0    1    1    0   
$EndComp
Text Label 4000 7200 2    60   ~ 0
T-0
Text Label 4200 7200 0    60   ~ 0
T+0
$Comp
L LS013B4DN04 U5
U 1 1 54BEA902
P 10250 1500
F 0 "U5" H 10600 350 60  0000 C CNN
F 1 "LS013B4DN04" H 10550 2300 60  0000 C CNN
F 2 "buq2:FPC-CONN-10-FCI-TOP-CONTACTS" H 10250 1500 60  0001 C CNN
F 3 "" H 10250 1500 60  0000 C CNN
	1    10250 1500
	1    0    0    -1  
$EndComp
Text Label 10000 1000 2    60   ~ 0
SCK
Text Label 10000 1150 2    60   ~ 0
MOSI
Text Label 6350 3600 0    60   ~ 0
CS_DISPLAY
Text Label 10000 1300 2    60   ~ 0
CS_DISPLAY
Text Label 9050 1600 2    60   ~ 0
VDD
$Comp
L C C8
U 1 1 54BEACE1
P 9150 1800
F 0 "C8" H 9150 1900 40  0000 L CNN
F 1 "100n" H 9156 1715 40  0000 L CNN
F 2 "buq2:SM0603" H 9188 1650 30  0001 C CNN
F 3 "" H 9150 1800 60  0000 C CNN
	1    9150 1800
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 54BEAD22
P 9350 1950
F 0 "C9" H 9350 2050 40  0000 L CNN
F 1 "1u" H 9356 1865 40  0000 L CNN
F 2 "buq2:SM0603" H 9388 1800 30  0001 C CNN
F 3 "" H 9350 1950 60  0000 C CNN
	1    9350 1950
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 54BEAD65
P 9650 2100
F 0 "C10" H 9650 2200 40  0000 L CNN
F 1 "1u" H 9656 2015 40  0000 L CNN
F 2 "buq2:SM0603" H 9688 1950 30  0001 C CNN
F 3 "" H 9650 2100 60  0000 C CNN
	1    9650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1600 10000 1600
Connection ~ 9150 1600
Wire Wire Line
	10000 1750 9350 1750
Wire Wire Line
	9650 1900 10000 1900
Wire Wire Line
	10000 2350 10000 2200
Wire Wire Line
	9150 2350 10000 2350
Wire Wire Line
	9650 2300 9650 2550
Wire Wire Line
	9350 2350 9350 2150
Connection ~ 9650 2350
Wire Wire Line
	9150 2350 9150 2000
Connection ~ 9350 2350
$Comp
L GND #PWR012
U 1 1 54BEB23C
P 9900 1450
F 0 "#PWR012" H 9900 1450 30  0001 C CNN
F 1 "GND" H 9900 1380 30  0001 C CNN
F 2 "" H 9900 1450 60  0000 C CNN
F 3 "" H 9900 1450 60  0000 C CNN
	1    9900 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 1450 10000 1450
$Comp
L GND #PWR013
U 1 1 54BEB2B3
P 9900 2050
F 0 "#PWR013" H 9900 2050 30  0001 C CNN
F 1 "GND" H 9900 1980 30  0001 C CNN
F 2 "" H 9900 2050 60  0000 C CNN
F 3 "" H 9900 2050 60  0000 C CNN
	1    9900 2050
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 2050 10000 2050
$Comp
L GND #PWR014
U 1 1 54BEB3F5
P 9650 2550
F 0 "#PWR014" H 9650 2550 30  0001 C CNN
F 1 "GND" H 9650 2480 30  0001 C CNN
F 2 "" H 9650 2550 60  0000 C CNN
F 3 "" H 9650 2550 60  0000 C CNN
	1    9650 2550
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 54BEB6B2
P 3200 7550
F 0 "P2" V 3150 7550 40  0000 C CNN
F 1 "CONN_POWER" V 3250 7550 40  0000 C CNN
F 2 "buq2:PIN_HEADER_2" H 3200 7550 60  0001 C CNN
F 3 "" H 3200 7550 60  0000 C CNN
	1    3200 7550
	0    1    1    0   
$EndComp
Text Label 3100 7200 2    60   ~ 0
VBUS
$Comp
L LD1117-SOT-223 U4
U 1 1 54BEB9AE
P 5750 6950
F 0 "U4" H 5750 6450 60  0000 C CNN
F 1 "LD1117-SOT-223" H 5750 7200 60  0000 C CNN
F 2 "buq2:SOT-223" H 5700 6950 60  0001 C CNN
F 3 "" H 5700 6950 60  0000 C CNN
	1    5750 6950
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 54BEBB30
P 6400 7150
F 0 "C7" H 6400 7250 40  0000 L CNN
F 1 "4.7u" H 6406 7065 40  0000 L CNN
F 2 "buq2:SM0805" H 6438 7000 30  0001 C CNN
F 3 "" H 6400 7150 60  0000 C CNN
	1    6400 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6950 6400 6950
$Comp
L GND #PWR015
U 1 1 54BEBBBE
P 6100 7500
F 0 "#PWR015" H 6100 7500 30  0001 C CNN
F 1 "GND" H 6100 7430 30  0001 C CNN
F 2 "" H 6100 7500 60  0000 C CNN
F 3 "" H 6100 7500 60  0000 C CNN
	1    6100 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 7500 6100 7350
Wire Wire Line
	5750 7350 6400 7350
Connection ~ 6100 7350
$Comp
L FUSE F1
U 1 1 54BEBD00
P 4950 6950
F 0 "F1" H 5050 7000 40  0000 C CNN
F 1 "FUSE" H 4850 6900 40  0000 C CNN
F 2 "buq2:RESETTABLE_FUSE_BOURNS" H 4950 6950 60  0001 C CNN
F 3 "" H 4950 6950 60  0000 C CNN
	1    4950 6950
	1    0    0    -1  
$EndComp
Text Label 4700 6950 2    60   ~ 0
VBUS
Text Label 6400 6950 0    60   ~ 0
VDD
Text Label 8800 4150 2    60   ~ 0
BTN0
Text Label 8800 4800 2    60   ~ 0
BTN1
Text Label 6350 2500 0    60   ~ 0
BTN1
Text Label 6350 2700 0    60   ~ 0
BTN2
Text Label 6350 2300 0    60   ~ 0
BTN0
$Comp
L LED D1
U 1 1 54BECE59
P 8700 3050
F 0 "D1" H 8700 3150 50  0000 C CNN
F 1 "LED" H 8700 2950 50  0000 C CNN
F 2 "buq2:SM0603" H 8700 3050 60  0001 C CNN
F 3 "" H 8700 3050 60  0000 C CNN
	1    8700 3050
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 54BECF0E
P 9150 3050
F 0 "R1" V 9230 3050 40  0000 C CNN
F 1 "4k" V 9157 3051 40  0000 C CNN
F 2 "buq2:SM0603" V 9080 3050 30  0001 C CNN
F 3 "" H 9150 3050 30  0000 C CNN
	1    9150 3050
	0    1    1    0   
$EndComp
$Comp
L GND #PWR016
U 1 1 54BECF95
P 9450 3050
F 0 "#PWR016" H 9450 3050 30  0001 C CNN
F 1 "GND" H 9450 2980 30  0001 C CNN
F 2 "" H 9450 3050 60  0000 C CNN
F 3 "" H 9450 3050 60  0000 C CNN
	1    9450 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9450 3050 9400 3050
$Comp
L LED D2
U 1 1 54BED2DA
P 8700 3300
F 0 "D2" H 8700 3400 50  0000 C CNN
F 1 "LED" H 8700 3200 50  0000 C CNN
F 2 "buq2:SM0603" H 8700 3300 60  0001 C CNN
F 3 "" H 8700 3300 60  0000 C CNN
	1    8700 3300
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 54BED2E0
P 9150 3300
F 0 "R2" V 9230 3300 40  0000 C CNN
F 1 "4k" V 9157 3301 40  0000 C CNN
F 2 "buq2:SM0603" V 9080 3300 30  0001 C CNN
F 3 "" H 9150 3300 30  0000 C CNN
	1    9150 3300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR017
U 1 1 54BED2E6
P 9450 3300
F 0 "#PWR017" H 9450 3300 30  0001 C CNN
F 1 "GND" H 9450 3230 30  0001 C CNN
F 2 "" H 9450 3300 60  0000 C CNN
F 3 "" H 9450 3300 60  0000 C CNN
	1    9450 3300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9450 3300 9400 3300
$Comp
L LED D3
U 1 1 54BED34B
P 8700 3550
F 0 "D3" H 8700 3650 50  0000 C CNN
F 1 "LED" H 8700 3450 50  0000 C CNN
F 2 "buq2:SM0603" H 8700 3550 60  0001 C CNN
F 3 "" H 8700 3550 60  0000 C CNN
	1    8700 3550
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 54BED351
P 9150 3550
F 0 "R3" V 9230 3550 40  0000 C CNN
F 1 "4k" V 9157 3551 40  0000 C CNN
F 2 "buq2:SM0603" V 9080 3550 30  0001 C CNN
F 3 "" H 9150 3550 30  0000 C CNN
	1    9150 3550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR018
U 1 1 54BED357
P 9450 3550
F 0 "#PWR018" H 9450 3550 30  0001 C CNN
F 1 "GND" H 9450 3480 30  0001 C CNN
F 2 "" H 9450 3550 60  0000 C CNN
F 3 "" H 9450 3550 60  0000 C CNN
	1    9450 3550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9450 3550 9400 3550
Text Label 8500 3050 2    60   ~ 0
LD0
Text Label 8500 3300 2    60   ~ 0
LD1
Text Label 8500 3550 2    60   ~ 0
LD2
Text Label 6350 2400 0    60   ~ 0
LD0
Text Label 6350 2600 0    60   ~ 0
LD1
Text Label 6350 2800 0    60   ~ 0
LD2
$Comp
L PWR_FLAG #FLG019
U 1 1 54BEE9A8
P 5200 6950
F 0 "#FLG019" H 5200 7045 30  0001 C CNN
F 1 "PWR_FLAG" H 5200 7130 30  0000 C CNN
F 2 "" H 5200 6950 60  0000 C CNN
F 3 "" H 5200 6950 60  0000 C CNN
	1    5200 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 1600 9650 1900
Connection ~ 9650 1750
Connection ~ 9650 1600
$Comp
L PWR_FLAG #FLG020
U 1 1 54BEF10E
P 6300 7400
F 0 "#FLG020" H 6300 7495 30  0001 C CNN
F 1 "PWR_FLAG" H 6300 7580 30  0000 C CNN
F 2 "" H 6300 7400 60  0000 C CNN
F 3 "" H 6300 7400 60  0000 C CNN
	1    6300 7400
	-1   0    0    1   
$EndComp
Wire Wire Line
	6300 7400 6300 7350
Connection ~ 6300 7350
NoConn ~ 6350 3250
NoConn ~ 6350 3350
NoConn ~ 6350 3150
NoConn ~ 6350 3450
NoConn ~ 6350 3000
NoConn ~ 6350 2900
NoConn ~ 3650 3150
NoConn ~ 3650 3250
NoConn ~ 3650 3350
NoConn ~ 3650 2900
NoConn ~ 3650 3000
NoConn ~ 3650 3450
NoConn ~ 3650 3900
NoConn ~ 3650 4000
NoConn ~ 3650 4100
$Comp
L SW_PUSH_4_PIN U6
U 1 1 54BF1AB8
P 9200 4150
F 0 "U6" H 9200 3900 60  0000 C CNN
F 1 "SW_PUSH_4_PIN" H 9200 4400 60  0000 C CNN
F 2 "buq2:PUSHBUTTON_4x4_5x5_ROTATED_4_PIN" H 9350 4150 60  0001 C CNN
F 3 "" H 9350 4150 60  0000 C CNN
	1    9200 4150
	1    0    0    -1  
$EndComp
Text Label 8800 5450 2    60   ~ 0
BTN2
Wire Wire Line
	8850 4050 8850 4250
Wire Wire Line
	8850 4150 8800 4150
Connection ~ 8850 4150
Wire Wire Line
	9550 4250 9550 4050
Wire Wire Line
	9550 4150 9600 4150
Connection ~ 9550 4150
$Comp
L GND #PWR021
U 1 1 54BF223A
P 9600 4150
F 0 "#PWR021" H 9600 4150 30  0001 C CNN
F 1 "GND" H 9600 4080 30  0001 C CNN
F 2 "" H 9600 4150 60  0000 C CNN
F 3 "" H 9600 4150 60  0000 C CNN
	1    9600 4150
	0    -1   -1   0   
$EndComp
$Comp
L SW_PUSH_4_PIN U7
U 1 1 54BF23C7
P 9200 4800
F 0 "U7" H 9200 4550 60  0000 C CNN
F 1 "SW_PUSH_4_PIN" H 9200 5050 60  0000 C CNN
F 2 "buq2:PUSHBUTTON_4x4_5x5_ROTATED_4_PIN" H 9350 4800 60  0001 C CNN
F 3 "" H 9350 4800 60  0000 C CNN
	1    9200 4800
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_4_PIN U8
U 1 1 54BF243E
P 9200 5450
F 0 "U8" H 9200 5200 60  0000 C CNN
F 1 "SW_PUSH_4_PIN" H 9200 5700 60  0000 C CNN
F 2 "buq2:PUSHBUTTON_4x4_5x5_ROTATED_4_PIN" H 9350 5450 60  0001 C CNN
F 3 "" H 9350 5450 60  0000 C CNN
	1    9200 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 5350 8850 5550
Wire Wire Line
	8850 5450 8800 5450
Connection ~ 8850 5450
Wire Wire Line
	9550 5550 9550 5350
Wire Wire Line
	9550 4900 9550 4700
Wire Wire Line
	9550 4800 9600 4800
Connection ~ 9550 4800
Wire Wire Line
	9550 5450 9600 5450
Connection ~ 9550 5450
$Comp
L GND #PWR022
U 1 1 54BF2761
P 9600 5450
F 0 "#PWR022" H 9600 5450 30  0001 C CNN
F 1 "GND" H 9600 5380 30  0001 C CNN
F 2 "" H 9600 5450 60  0000 C CNN
F 3 "" H 9600 5450 60  0000 C CNN
	1    9600 5450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR023
U 1 1 54BF2794
P 9600 4800
F 0 "#PWR023" H 9600 4800 30  0001 C CNN
F 1 "GND" H 9600 4730 30  0001 C CNN
F 2 "" H 9600 4800 60  0000 C CNN
F 3 "" H 9600 4800 60  0000 C CNN
	1    9600 4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8850 4900 8850 4700
Wire Wire Line
	8850 4800 8800 4800
Connection ~ 8850 4800
$Comp
L GND #PWR024
U 1 1 54BF66A7
P 3300 7150
F 0 "#PWR024" H 3300 7150 30  0001 C CNN
F 1 "GND" H 3300 7080 30  0001 C CNN
F 2 "" H 3300 7150 60  0000 C CNN
F 3 "" H 3300 7150 60  0000 C CNN
	1    3300 7150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 7150 3300 7200
$Comp
L GND #PWR025
U 1 1 54BF685A
P 3750 6000
F 0 "#PWR025" H 3750 6000 30  0001 C CNN
F 1 "GND" H 3750 5930 30  0001 C CNN
F 2 "" H 3750 6000 60  0000 C CNN
F 3 "" H 3750 6000 60  0000 C CNN
	1    3750 6000
	-1   0    0    1   
$EndComp
Text Label 3650 6100 1    60   ~ 0
RELAY0
Text Label 3650 3600 2    60   ~ 0
RELAY0
$Comp
L CONN_4 P3
U 1 1 54BFB3CC
P 3600 6450
F 0 "P3" V 3550 6450 50  0000 C CNN
F 1 "CONN_4" V 3650 6450 50  0000 C CNN
F 2 "buq2:PIN_HEADER_4" H 3600 6450 60  0001 C CNN
F 3 "" H 3600 6450 60  0000 C CNN
	1    3600 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 6000 3750 6100
Text Label 3550 6100 1    60   ~ 0
RELAY1
Text Label 3450 6100 1    60   ~ 0
RELAY2
Text Label 3650 3700 2    60   ~ 0
RELAY1
Text Label 3650 3800 2    60   ~ 0
RELAY2
$Comp
L MAX6675ISA+ U9
U 1 1 54BF378E
P 5250 5600
F 0 "U9" H 5250 5100 60  0000 C CNN
F 1 "MAX6675ISA+" H 5250 6000 60  0000 C CNN
F 2 "buq2:8-SOIC" H 5300 5350 60  0001 C CNN
F 3 "" H 5300 5350 60  0000 C CNN
	1    5250 5600
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 54BF384C
P 4600 6050
F 0 "C11" H 4600 6150 40  0000 L CNN
F 1 "100n" H 4606 5965 40  0000 L CNN
F 2 "buq2:SM0603" H 4638 5900 30  0001 C CNN
F 3 "" H 4600 6050 60  0000 C CNN
	1    4600 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 54BF38B3
P 4600 6350
F 0 "#PWR026" H 4600 6350 30  0001 C CNN
F 1 "GND" H 4600 6280 30  0001 C CNN
F 2 "" H 4600 6350 60  0000 C CNN
F 3 "" H 4600 6350 60  0000 C CNN
	1    4600 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6350 4600 6250
Wire Wire Line
	4600 5850 4850 5850
Text Label 4850 5550 2    60   ~ 0
T-1
Text Label 4850 5700 2    60   ~ 0
T+1
Text Label 5700 5550 0    60   ~ 0
MISO
Text Label 5700 5700 0    60   ~ 0
nCS_THERMO1
Text Label 5700 5850 0    60   ~ 0
SCK
NoConn ~ 5700 5400
$Comp
L GND #PWR027
U 1 1 54BF3CB5
P 4700 5400
F 0 "#PWR027" H 4700 5400 30  0001 C CNN
F 1 "GND" H 4700 5330 30  0001 C CNN
F 2 "" H 4700 5400 60  0000 C CNN
F 3 "" H 4700 5400 60  0000 C CNN
	1    4700 5400
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 5400 4850 5400
Text Label 6350 3800 0    60   ~ 0
nCS_THERMO1
NoConn ~ 6350 4000
$Comp
L CONN_2 P4
U 1 1 54BF48F3
P 4800 7650
F 0 "P4" V 4750 7650 40  0000 C CNN
F 1 "CONN_THERMO1" V 4850 7650 40  0000 C CNN
F 2 "buq2:PIN_HEADER_2" H 4800 7650 60  0001 C CNN
F 3 "" H 4800 7650 60  0000 C CNN
	1    4800 7650
	0    1    1    0   
$EndComp
Text Label 4700 7300 2    60   ~ 0
T-1
Text Label 4900 7300 0    60   ~ 0
T+1
Text Label 4600 5850 2    60   ~ 0
VDD
$EndSCHEMATC
