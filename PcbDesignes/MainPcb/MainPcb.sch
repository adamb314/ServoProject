EESchema Schematic File Version 4
LIBS:MainPcb-cache
EELAYER 30 0
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
L MainPcb-rescue:Itsybitsy-New_Library U1
U 1 1 5D6691BB
P 1300 2150
F 0 "U1" H 1700 2315 50  0000 C CNN
F 1 "Itsybitsy" H 1700 2224 50  0000 C CNN
F 2 "Package_DIP:DIP-28_600" H 2050 2200 50  0001 C CNN
F 3 "" H 2050 2200 50  0001 C CNN
	1    1300 2150
	1    0    0    -1  
$EndComp
$Comp
L MainPcb-rescue:TB6612FNG-New_Library U2
U 1 1 5D66A310
P 8300 2150
F 0 "U2" H 8725 2315 50  0000 C CNN
F 1 "TB6612FNG" H 8725 2224 50  0000 C CNN
F 2 "Package_DIP:DIP-16_600_ELL" H 8900 2200 50  0001 C CNN
F 3 "" H 8900 2200 50  0001 C CNN
	1    8300 2150
	-1   0    0    -1  
$EndComp
$Comp
L MainPcb-rescue:CurrentSensor-New_Library U3
U 1 1 5D66AD20
P 8150 3400
F 0 "U3" H 8425 3565 50  0000 C CNN
F 1 "CurrentSensor" H 8425 3474 50  0000 C CNN
F 2 "Package_DIP:DIP-7_600_ELL" H 8800 3450 50  0001 C CNN
F 3 "" H 8800 3450 50  0001 C CNN
	1    8150 3400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8400 2450 8550 2450
Wire Wire Line
	8400 2350 8450 2350
Wire Wire Line
	8450 2350 8450 1900
Wire Wire Line
	8400 2550 8550 2550
Wire Wire Line
	8550 2550 8550 2850
Wire Wire Line
	8400 2650 8450 2650
Wire Wire Line
	8450 2650 8450 2750
Wire Wire Line
	8450 2750 8400 2750
Wire Wire Line
	8450 2750 8450 3600
Wire Wire Line
	8450 3600 8250 3600
Connection ~ 8450 2750
Wire Wire Line
	8450 1900 7200 1900
Wire Wire Line
	7200 1900 7200 2250
Wire Wire Line
	7200 2250 7350 2250
Wire Wire Line
	7350 2550 7200 2550
Wire Wire Line
	7200 2550 7200 2250
Connection ~ 7200 2250
Wire Wire Line
	7350 2850 7200 2850
Wire Wire Line
	7200 2850 7200 2550
Connection ~ 7200 2550
Wire Wire Line
	7350 2450 7300 2450
Wire Wire Line
	7300 2450 7300 2650
Wire Wire Line
	7300 2650 7350 2650
Wire Wire Line
	7350 2350 7250 2350
Wire Wire Line
	7250 2350 7250 2750
Wire Wire Line
	7250 2750 7350 2750
Wire Wire Line
	7250 3800 7500 3800
Wire Wire Line
	7500 3900 7250 3900
Wire Wire Line
	7250 3900 7250 3800
Connection ~ 7250 2350
Wire Wire Line
	7500 3600 7400 3600
Wire Wire Line
	7400 3600 7400 3350
$Comp
L Diode:1N4001 D1
U 1 1 5D66F4E1
P 6200 1100
F 0 "D1" V 6246 1179 50  0000 L CNN
F 1 "1N4001" V 6155 1179 50  0000 L CNN
F 2 "Diode_THT:D_DO-34_SOD68_P7.62mm_Horizontal" H 6200 925 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 6200 1100 50  0001 C CNN
	1    6200 1100
	0    1    -1   0   
$EndComp
Wire Wire Line
	2200 2250 2350 2250
$Comp
L power:+3.3V #PWR0103
U 1 1 5D686FC5
P 6850 3700
F 0 "#PWR0103" H 6850 3550 50  0001 C CNN
F 1 "+3.3V" H 6865 3873 50  0000 C CNN
F 2 "" H 6850 3700 50  0001 C CNN
F 3 "" H 6850 3700 50  0001 C CNN
	1    6850 3700
	-1   0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0107
U 1 1 5D69F0B1
P 6850 1550
F 0 "#PWR0107" H 6850 1400 50  0001 C CNN
F 1 "+BATT" H 6865 1723 50  0000 C CNN
F 2 "" H 6850 1550 50  0001 C CNN
F 3 "" H 6850 1550 50  0001 C CNN
	1    6850 1550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8650 2250 8400 2250
$Comp
L power:+BATT #PWR0108
U 1 1 5D6A92A8
P 5800 1050
F 0 "#PWR0108" H 5800 900 50  0001 C CNN
F 1 "+BATT" H 5815 1223 50  0000 C CNN
F 2 "" H 5800 1050 50  0001 C CNN
F 3 "" H 5800 1050 50  0001 C CNN
	1    5800 1050
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5D6AAD2A
P 6000 750
F 0 "J2" V 5873 462 50  0000 R CNN
F 1 "Conn_01x04" V 5964 462 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x04_P3.81mm_Drill1.2mm" H 6000 750 50  0001 C CNN
F 3 "~" H 6000 750 50  0001 C CNN
	1    6000 750 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5800 1050 5800 950 
Wire Wire Line
	5800 950  5900 950 
Wire Wire Line
	8550 2850 8400 2850
Connection ~ 7200 1900
Wire Wire Line
	1200 2350 1000 2350
Wire Wire Line
	1000 2350 1000 1900
Wire Wire Line
	1200 3450 1100 3450
Wire Wire Line
	1100 3450 1100 3850
Wire Wire Line
	1200 3350 1000 3350
Wire Wire Line
	1000 3350 1000 3950
Wire Wire Line
	1200 3250 900  3250
Wire Wire Line
	900  3250 900  4050
Wire Wire Line
	1200 3150 800  3150
Wire Wire Line
	800  3150 800  4150
Wire Wire Line
	3750 2000 2350 2000
Wire Wire Line
	2350 2000 2350 2250
Wire Wire Line
	3600 2350 3600 2250
Wire Wire Line
	2200 2350 3600 2350
Wire Wire Line
	5250 2150 5250 1900
Wire Wire Line
	5750 2150 6550 2150
Wire Wire Line
	6550 2150 6550 1550
Wire Wire Line
	6550 1550 6850 1550
Wire Wire Line
	8550 1650 7800 1650
Wire Wire Line
	6650 1650 6650 2250
Wire Wire Line
	6650 2250 5750 2250
Wire Wire Line
	6750 2350 6750 2550
Wire Wire Line
	6750 2550 5750 2550
Wire Wire Line
	6750 2350 7250 2350
Wire Wire Line
	2650 2650 2650 2450
Wire Wire Line
	2650 2450 3600 2450
Wire Wire Line
	3600 2450 3600 2550
Wire Wire Line
	2200 2650 2650 2650
Wire Wire Line
	1100 3850 2950 3850
Wire Wire Line
	1000 3950 2900 3950
Wire Wire Line
	900  4050 2850 4050
Wire Wire Line
	800  4150 2800 4150
Wire Wire Line
	3800 3450 3800 2650
Wire Wire Line
	3800 2650 3600 2650
Wire Wire Line
	2200 3450 3800 3450
Wire Wire Line
	3750 3550 3750 2750
Wire Wire Line
	3750 2750 3600 2750
Wire Wire Line
	2200 3550 3750 3550
Wire Wire Line
	4150 1900 4150 2850
Wire Wire Line
	4150 2850 3600 2850
Wire Wire Line
	4100 2350 4100 2950
Wire Wire Line
	4100 2950 3600 2950
Connection ~ 3600 2350
Wire Wire Line
	3750 2000 3750 2150
Wire Wire Line
	3750 2150 3600 2150
Wire Wire Line
	2950 1900 4150 1900
Wire Wire Line
	1000 1900 2950 1900
Connection ~ 2950 1900
Wire Wire Line
	2950 1900 2950 2150
Wire Wire Line
	2950 2150 3100 2150
Wire Wire Line
	5750 2650 6200 2650
Wire Wire Line
	6200 2650 6200 1250
Wire Wire Line
	5750 2750 6100 2750
Wire Wire Line
	6100 2750 6100 950 
Wire Wire Line
	8550 1650 8550 2450
Wire Wire Line
	6850 1550 8650 1550
Wire Wire Line
	8650 1550 8650 2250
Connection ~ 6850 1550
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 5D80CB38
P 5350 3700
F 0 "J6" H 5430 3742 50  0000 L CNN
F 1 "Conn_01x01" H 5430 3651 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Pad1.5mm_Drill1mm" H 5350 3700 50  0001 C CNN
F 3 "~" H 5350 3700 50  0001 C CNN
	1    5350 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 5D80CBAF
P 5350 4000
F 0 "J7" H 5430 4042 50  0000 L CNN
F 1 "Conn_01x01" H 5430 3951 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Pad1.5mm_Drill1mm" H 5350 4000 50  0001 C CNN
F 3 "~" H 5350 4000 50  0001 C CNN
	1    5350 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 5D81111A
P 5350 4350
F 0 "J8" H 5430 4392 50  0000 L CNN
F 1 "Conn_01x01" H 5430 4301 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Pad1.5mm_Drill1mm" H 5350 4350 50  0001 C CNN
F 3 "~" H 5350 4350 50  0001 C CNN
	1    5350 4350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J9
U 1 1 5D815687
P 5350 4700
F 0 "J9" H 5430 4742 50  0000 L CNN
F 1 "Conn_01x01" H 5430 4651 50  0000 L CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Pad1.5mm_Drill1mm" H 5350 4700 50  0001 C CNN
F 3 "~" H 5350 4700 50  0001 C CNN
	1    5350 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4000 5150 4000
Wire Wire Line
	5000 4350 5150 4350
Wire Wire Line
	4950 4700 5150 4700
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J1
U 1 1 5D83791E
P 3300 2150
F 0 "J1" H 3350 2367 50  0001 C CNN
F 1 "Conn_02x02_Odd_Even" H 3350 2276 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 3300 2150 50  0001 C CNN
F 3 "~" H 3300 2150 50  0001 C CNN
	1    3300 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J3
U 1 1 5D837975
P 3300 2750
F 0 "J3" H 3350 3167 50  0001 C CNN
F 1 "Conn_02x05_Odd_Even" H 3350 3076 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 3300 2750 50  0001 C CNN
F 3 "~" H 3300 2750 50  0001 C CNN
	1    3300 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J4
U 1 1 5D84BF83
P 5450 2150
F 0 "J4" H 5500 2367 50  0001 C CNN
F 1 "Conn_02x02_Odd_Even" H 5500 2276 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 5450 2150 50  0001 C CNN
F 3 "~" H 5450 2150 50  0001 C CNN
	1    5450 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J5
U 1 1 5D851177
P 5450 2750
F 0 "J5" H 5500 3167 50  0001 C CNN
F 1 "Conn_02x05_Odd_Even" H 5500 3076 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 5450 2750 50  0001 C CNN
F 3 "~" H 5450 2750 50  0001 C CNN
	1    5450 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2350 4100 2350
Wire Wire Line
	6500 2850 6500 3700
Wire Wire Line
	6500 3700 6850 3700
Wire Wire Line
	5750 2850 6500 2850
Wire Wire Line
	5750 2950 6400 2950
Wire Wire Line
	6400 2950 6400 3800
Wire Wire Line
	6400 3800 7250 3800
Connection ~ 7250 3800
$Comp
L power:GNDPWR #PWR0101
U 1 1 5D896D06
P 7800 1650
F 0 "#PWR0101" H 7800 1450 50  0001 C CNN
F 1 "GNDPWR" H 7804 1496 50  0000 C CNN
F 2 "" H 7800 1600 50  0001 C CNN
F 3 "" H 7800 1600 50  0001 C CNN
	1    7800 1650
	1    0    0    -1  
$EndComp
Connection ~ 7800 1650
Wire Wire Line
	6650 1650 7800 1650
$Comp
L power:GND #PWR0102
U 1 1 5D8A0F70
P 6400 3800
F 0 "#PWR0102" H 6400 3550 50  0001 C CNN
F 1 "GND" H 6405 3627 50  0000 C CNN
F 2 "" H 6400 3800 50  0001 C CNN
F 3 "" H 6400 3800 50  0001 C CNN
	1    6400 3800
	1    0    0    -1  
$EndComp
Connection ~ 6400 3800
$Comp
L power:+3.3VP #PWR0104
U 1 1 5D8A0FB4
P 7200 1900
F 0 "#PWR0104" H 7350 1850 50  0001 C CNN
F 1 "+3.3VP" H 7220 2043 50  0000 C CNN
F 2 "" H 7200 1900 50  0001 C CNN
F 3 "" H 7200 1900 50  0001 C CNN
	1    7200 1900
	1    0    0    -1  
$EndComp
Connection ~ 6850 3700
Wire Wire Line
	6850 3700 7500 3700
Wire Wire Line
	5250 1900 7200 1900
Wire Wire Line
	6650 1650 6000 1650
Wire Wire Line
	6000 1650 6000 950 
Connection ~ 6650 1650
Wire Wire Line
	2700 2750 2700 2250
Wire Wire Line
	2700 2250 3100 2250
Wire Wire Line
	2200 2750 2700 2750
Wire Wire Line
	2800 4150 2800 2550
Wire Wire Line
	2800 2550 3100 2550
Wire Wire Line
	2850 4050 2850 2650
Wire Wire Line
	2850 2650 3100 2650
Wire Wire Line
	2900 3950 2900 2750
Wire Wire Line
	2900 2750 3100 2750
Wire Wire Line
	2950 3850 2950 2850
Wire Wire Line
	2950 2850 3100 2850
Wire Wire Line
	1200 2750 600  2750
Wire Wire Line
	600  2750 600  4350
Wire Wire Line
	600  4350 3050 4350
Wire Wire Line
	3050 4350 3050 2950
Wire Wire Line
	3050 2950 3100 2950
Wire Wire Line
	7300 2450 5250 2450
Wire Wire Line
	5250 2450 5250 2250
Connection ~ 7300 2450
Wire Wire Line
	4950 2550 5250 2550
Wire Wire Line
	4950 2550 4950 4700
Wire Wire Line
	5000 2650 5000 4350
Wire Wire Line
	5000 2650 5250 2650
Wire Wire Line
	5050 2750 5050 4000
Wire Wire Line
	5050 2750 5250 2750
Wire Wire Line
	5100 2850 5100 3700
Wire Wire Line
	5100 2850 5250 2850
Wire Wire Line
	5100 3700 5150 3700
Wire Wire Line
	5250 3350 5250 2950
Wire Wire Line
	5250 3350 7400 3350
$EndSCHEMATC
