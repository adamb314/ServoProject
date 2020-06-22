EESchema Schematic File Version 4
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
L Sensor_Proximity:ITR8307 U1
U 1 1 5EEF8A34
P 3600 3150
F 0 "U1" H 3600 3467 50  0000 C CNN
F 1 "ITR8307" H 3600 3376 50  0000 C CNN
F 2 "OptoDevice:Everlight_ITR8307" H 3600 2950 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/ITR8307.pdf" H 3600 3250 50  0001 C CNN
	1    3600 3150
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Proximity:ITR8307 U2
U 1 1 5EEF93BB
P 4850 3150
F 0 "U2" H 4850 3467 50  0000 C CNN
F 1 "ITR8307" H 4850 3376 50  0000 C CNN
F 2 "OptoDevice:Everlight_ITR8307" H 4850 2950 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/ITR8307.pdf" H 4850 3250 50  0001 C CNN
	1    4850 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5EEFA858
P 4200 3900
F 0 "#PWR?" H 4200 3650 50  0001 C CNN
F 1 "GND" H 4205 3727 50  0000 C CNN
F 2 "" H 4200 3900 50  0001 C CNN
F 3 "" H 4200 3900 50  0001 C CNN
	1    4200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5EEFAFC1
P 4200 2150
F 0 "#PWR?" H 4200 2000 50  0001 C CNN
F 1 "+3.3V" H 4215 2323 50  0000 C CNN
F 2 "" H 4200 2150 50  0001 C CNN
F 3 "" H 4200 2150 50  0001 C CNN
	1    4200 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5EEFDC57
P 3400 2500
F 0 "R1" H 3470 2546 50  0000 L CNN
F 1 "120" H 3470 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3330 2500 50  0001 C CNN
F 3 "~" H 3400 2500 50  0001 C CNN
	1    3400 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5EEFF9B5
P 3900 2500
F 0 "R2" H 3970 2546 50  0000 L CNN
F 1 "8.2K" H 3970 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3830 2500 50  0001 C CNN
F 3 "~" H 3900 2500 50  0001 C CNN
	1    3900 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5EEFFEA8
P 4550 2500
F 0 "R3" H 4620 2546 50  0000 L CNN
F 1 "120" H 4620 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4480 2500 50  0001 C CNN
F 3 "~" H 4550 2500 50  0001 C CNN
	1    4550 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5EF000F1
P 5100 2500
F 0 "R4" H 5170 2546 50  0000 L CNN
F 1 "8.2K" H 5170 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5030 2500 50  0001 C CNN
F 3 "~" H 5100 2500 50  0001 C CNN
	1    5100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3250 4550 3900
Wire Wire Line
	4550 3900 4200 3900
Wire Wire Line
	3300 3250 3300 3900
Wire Wire Line
	3300 3900 3900 3900
Connection ~ 4200 3900
Wire Wire Line
	3300 3050 3300 2650
Wire Wire Line
	3300 2650 3400 2650
Wire Wire Line
	3400 2350 3400 2150
Wire Wire Line
	3400 2150 3900 2150
Wire Wire Line
	4550 3050 4550 2650
Wire Wire Line
	4550 2350 4550 2150
Wire Wire Line
	4550 2150 4200 2150
Connection ~ 4200 2150
Wire Wire Line
	5150 3250 5150 3900
Wire Wire Line
	5150 3900 4550 3900
Connection ~ 4550 3900
Wire Wire Line
	3900 3250 3900 3900
Wire Wire Line
	3900 3900 4200 3900
Connection ~ 3900 3900
Wire Wire Line
	5150 3050 5150 2850
Wire Wire Line
	5150 2650 5100 2650
Wire Wire Line
	5100 2350 5100 2150
Wire Wire Line
	5100 2150 4550 2150
Connection ~ 4550 2150
Wire Wire Line
	3900 3050 3900 2700
Wire Wire Line
	3900 2350 3900 2150
Connection ~ 3900 2150
Wire Wire Line
	3900 2150 4200 2150
Wire Wire Line
	3900 2700 6200 2700
Connection ~ 3900 2700
Wire Wire Line
	3900 2700 3900 2650
Wire Wire Line
	5150 2850 6200 2850
Connection ~ 5150 2850
Wire Wire Line
	5150 2850 5150 2650
Text GLabel 6200 2700 2    50   Output ~ 0
A4
Text GLabel 6200 2850 2    50   Output ~ 0
A5
$EndSCHEMATC
