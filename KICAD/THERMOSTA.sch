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
L power:+5V #PWR08
U 1 1 5F3CBB19
P 1700 2350
F 0 "#PWR08" H 1700 2200 50  0001 C CNN
F 1 "+5V" H 1715 2523 50  0000 C CNN
F 2 "" H 1700 2350 50  0001 C CNN
F 3 "" H 1700 2350 50  0001 C CNN
	1    1700 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 2350 1700 2450
Wire Wire Line
	1800 2450 1800 2350
Wire Wire Line
	1800 2350 1700 2350
Connection ~ 1700 2350
$Comp
L Device:Crystal_Small Y1
U 1 1 5F3CD062
P 3900 4200
F 0 "Y1" V 3854 4288 50  0000 L CNN
F 1 "8MHZ" V 3945 4288 50  0000 L CNN
F 2 "Crystal:Crystal_HC49-4H_Vertical" H 3900 4200 50  0001 C CNN
F 3 "~" H 3900 4200 50  0001 C CNN
	1    3900 4200
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5F3CD9D8
P 3700 3950
F 0 "C3" H 3792 3996 50  0000 L CNN
F 1 "22pF" H 3792 3905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3700 3950 50  0001 C CNN
F 3 "~" H 3700 3950 50  0001 C CNN
	1    3700 3950
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5F3CDEEC
P 3700 4450
F 0 "C4" H 3792 4496 50  0000 L CNN
F 1 "22pF" H 3792 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3700 4450 50  0001 C CNN
F 3 "~" H 3700 4450 50  0001 C CNN
	1    3700 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	3800 4450 3900 4450
Wire Wire Line
	3800 3950 3900 3950
Wire Wire Line
	3900 3950 3900 4100
$Comp
L power:GND #PWR015
U 1 1 5F3CE401
P 3500 4200
F 0 "#PWR015" H 3500 3950 50  0001 C CNN
F 1 "GND" H 3505 4027 50  0000 C CNN
F 2 "" H 3500 4200 50  0001 C CNN
F 3 "" H 3500 4200 50  0001 C CNN
	1    3500 4200
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 4200 3500 3950
Wire Wire Line
	3500 3950 3600 3950
Wire Wire Line
	3600 4450 3500 4450
Wire Wire Line
	3500 4450 3500 4200
Connection ~ 3500 4200
$Comp
L Relay:SANYOU_SRD_Form_C K1
U 1 1 5F594A5D
P 8350 4050
F 0 "K1" H 8780 4096 50  0000 L CNN
F 1 "SANYOU_SRD_Form_C" H 7900 3500 50  0000 L CNN
F 2 "Relay_THT:Relay_SPDT_SANYOU_SRD_Series_Form_C" H 8800 4000 50  0001 L CNN
F 3 "http://www.sanyourelay.ca/public/products/pdf/SRD.pdf" H 8350 4050 50  0001 C CNN
	1    8350 4050
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4007 D3
U 1 1 5F5963A1
P 7550 4050
F 0 "D3" V 7504 4129 50  0000 L CNN
F 1 "1N4007" V 7595 4129 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 7550 3875 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 7550 4050 50  0001 C CNN
	1    7550 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	7550 3900 7550 3650
Wire Wire Line
	7550 3650 8150 3650
Wire Wire Line
	8150 3650 8150 3750
Wire Wire Line
	7550 4200 7550 4450
Wire Wire Line
	7550 4450 8150 4450
Wire Wire Line
	8150 4450 8150 4350
$Comp
L Connector:Conn_01x03_Male J3
U 1 1 5F59990B
P 9350 4050
F 0 "J3" H 9322 3982 50  0000 R CNN
F 1 "HEAT SWITCH" H 9322 4073 50  0000 R CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type055_RT01503HDWU_1x03_P5.00mm_Horizontal" H 9350 4050 50  0001 C CNN
F 3 "~" H 9350 4050 50  0001 C CNN
	1    9350 4050
	-1   0    0    1   
$EndComp
Wire Wire Line
	8550 4350 8950 4350
Wire Wire Line
	8950 4350 8950 4050
Wire Wire Line
	8950 4050 9150 4050
Wire Wire Line
	8650 3750 9050 3750
Wire Wire Line
	9050 3750 9050 4150
Wire Wire Line
	9050 4150 9150 4150
Wire Wire Line
	8450 3750 8450 3650
Wire Wire Line
	8450 3650 9100 3650
Wire Wire Line
	9100 3650 9100 3950
Wire Wire Line
	9100 3950 9150 3950
$Comp
L Transistor_BJT:BC547 Q3
U 1 1 5F5A010A
P 6900 4450
F 0 "Q3" H 7091 4496 50  0000 L CNN
F 1 "BC547" H 7091 4405 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7100 4375 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 6900 4450 50  0001 L CNN
	1    6900 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5F5A3332
P 6450 4450
F 0 "R3" V 6243 4450 50  0000 C CNN
F 1 "R" V 6334 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6380 4450 50  0001 C CNN
F 3 "~" H 6450 4450 50  0001 C CNN
	1    6450 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	6600 4450 6700 4450
$Comp
L power:GND #PWR011
U 1 1 5F5A88A2
P 7000 4800
F 0 "#PWR011" H 7000 4550 50  0001 C CNN
F 1 "GND" H 7005 4627 50  0000 C CNN
F 2 "" H 7000 4800 50  0001 C CNN
F 3 "" H 7000 4800 50  0001 C CNN
	1    7000 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 4650 7000 4800
Wire Wire Line
	7000 4250 7000 4200
Wire Wire Line
	7000 4200 7550 4200
Connection ~ 7550 4200
$Comp
L power:+5V #PWR010
U 1 1 5F5D6BC5
P 7550 3550
F 0 "#PWR010" H 7550 3400 50  0001 C CNN
F 1 "+5V" H 7565 3723 50  0000 C CNN
F 2 "" H 7550 3550 50  0001 C CNN
F 3 "" H 7550 3550 50  0001 C CNN
	1    7550 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3550 7550 3650
Connection ~ 7550 3650
Text GLabel 3550 3250 2    50   Input ~ 0
D7
Wire Wire Line
	3550 3250 3250 3250
Text GLabel 3550 3150 2    50   Input ~ 0
D6
Wire Wire Line
	3550 3150 3100 3150
Text GLabel 3550 3050 2    50   Input ~ 0
D5
Wire Wire Line
	3550 3050 2950 3050
Text GLabel 3550 2950 2    50   Input ~ 0
D4
Wire Wire Line
	3550 2950 2300 2950
$Comp
L power:+5V #PWR03
U 1 1 5F63395B
P 8250 1400
F 0 "#PWR03" H 8250 1250 50  0001 C CNN
F 1 "+5V" H 8265 1573 50  0000 C CNN
F 2 "" H 8250 1400 50  0001 C CNN
F 3 "" H 8250 1400 50  0001 C CNN
	1    8250 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 1400 8250 1500
$Comp
L Device:R_POT RV1
U 1 1 5F636D39
P 8250 1650
F 0 "RV1" H 8180 1696 50  0000 R CNN
F 1 "R_POT" H 8180 1605 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Bourns_3266W_Vertical" H 8250 1650 50  0001 C CNN
F 3 "~" H 8250 1650 50  0001 C CNN
	1    8250 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5F639EB8
P 8250 1900
F 0 "#PWR06" H 8250 1650 50  0001 C CNN
F 1 "GND" H 8255 1727 50  0000 C CNN
F 2 "" H 8250 1900 50  0001 C CNN
F 3 "" H 8250 1900 50  0001 C CNN
	1    8250 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 1900 8250 1800
$Comp
L Relay:SANYOU_SRD_Form_C K2
U 1 1 5F644E48
P 8550 5550
F 0 "K2" H 8980 5596 50  0000 L CNN
F 1 "SANYOU_SRD_Form_C" H 8100 5000 50  0000 L CNN
F 2 "Relay_THT:Relay_SPDT_SANYOU_SRD_Series_Form_C" H 9000 5500 50  0001 L CNN
F 3 "http://www.sanyourelay.ca/public/products/pdf/SRD.pdf" H 8550 5550 50  0001 C CNN
	1    8550 5550
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4007 D4
U 1 1 5F644E4E
P 7750 5550
F 0 "D4" V 7704 5629 50  0000 L CNN
F 1 "1N4007" V 7795 5629 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 7750 5375 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 7750 5550 50  0001 C CNN
	1    7750 5550
	0    1    1    0   
$EndComp
Wire Wire Line
	7750 5400 7750 5150
Wire Wire Line
	7750 5150 8350 5150
Wire Wire Line
	8350 5150 8350 5250
Wire Wire Line
	7750 5700 7750 5950
Wire Wire Line
	7750 5950 8350 5950
Wire Wire Line
	8350 5950 8350 5850
$Comp
L Connector:Conn_01x03_Male J4
U 1 1 5F644E5A
P 9550 5550
F 0 "J4" H 9522 5482 50  0000 R CNN
F 1 "HUM SWITCH" H 9522 5573 50  0000 R CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type055_RT01503HDWU_1x03_P5.00mm_Horizontal" H 9550 5550 50  0001 C CNN
F 3 "~" H 9550 5550 50  0001 C CNN
	1    9550 5550
	-1   0    0    1   
$EndComp
Wire Wire Line
	8750 5850 9150 5850
Wire Wire Line
	9150 5850 9150 5550
Wire Wire Line
	9150 5550 9350 5550
Wire Wire Line
	8850 5250 9250 5250
Wire Wire Line
	9250 5250 9250 5650
Wire Wire Line
	9250 5650 9350 5650
Wire Wire Line
	8650 5250 8650 5150
Wire Wire Line
	8650 5150 9300 5150
Wire Wire Line
	9300 5150 9300 5450
Wire Wire Line
	9300 5450 9350 5450
$Comp
L Transistor_BJT:BC547 Q4
U 1 1 5F644E6A
P 7100 5950
F 0 "Q4" H 7291 5996 50  0000 L CNN
F 1 "BC547" H 7291 5905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7300 5875 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 7100 5950 50  0001 L CNN
	1    7100 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5F644E70
P 6650 5950
F 0 "R4" V 6443 5950 50  0000 C CNN
F 1 "R" V 6534 5950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 6580 5950 50  0001 C CNN
F 3 "~" H 6650 5950 50  0001 C CNN
	1    6650 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 5950 6900 5950
$Comp
L power:GND #PWR014
U 1 1 5F644E77
P 7200 6300
F 0 "#PWR014" H 7200 6050 50  0001 C CNN
F 1 "GND" H 7205 6127 50  0000 C CNN
F 2 "" H 7200 6300 50  0001 C CNN
F 3 "" H 7200 6300 50  0001 C CNN
	1    7200 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 6150 7200 6300
Wire Wire Line
	7200 5750 7200 5700
Wire Wire Line
	7200 5700 7750 5700
Connection ~ 7750 5700
$Comp
L power:+5V #PWR012
U 1 1 5F644E82
P 7750 5050
F 0 "#PWR012" H 7750 4900 50  0001 C CNN
F 1 "+5V" H 7765 5223 50  0000 C CNN
F 2 "" H 7750 5050 50  0001 C CNN
F 3 "" H 7750 5050 50  0001 C CNN
	1    7750 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 5050 7750 5150
Connection ~ 7750 5150
Text GLabel 6350 5950 0    50   Input ~ 0
RH
Wire Wire Line
	6500 5950 6350 5950
Text GLabel 6200 4450 0    50   Input ~ 0
RT
Wire Wire Line
	6200 4450 6300 4450
Text GLabel 2400 3650 2    50   Input ~ 0
RT
$Comp
L Device:Thermistor_NTC TH1
U 1 1 5F64DA19
P 6200 1100
F 0 "TH1" H 6298 1146 50  0000 L CNN
F 1 "Thermistor_NTC" H 6298 1055 50  0000 L CNN
F 2 "Connector_JST:JST_EH_B2B-EH-A_1x02_P2.50mm_Vertical" H 6200 1150 50  0001 C CNN
F 3 "~" H 6200 1150 50  0001 C CNN
	1    6200 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F64F220
P 6200 1650
F 0 "R2" V 5993 1650 50  0000 C CNN
F 1 "R" V 6084 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6130 1650 50  0001 C CNN
F 3 "~" H 6200 1650 50  0001 C CNN
	1    6200 1650
	-1   0    0    1   
$EndComp
Wire Wire Line
	6200 1500 6200 1350
$Comp
L power:GND #PWR05
U 1 1 5F65217F
P 6200 1900
F 0 "#PWR05" H 6200 1650 50  0001 C CNN
F 1 "GND" H 6205 1727 50  0000 C CNN
F 2 "" H 6200 1900 50  0001 C CNN
F 3 "" H 6200 1900 50  0001 C CNN
	1    6200 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 1900 6200 1850
$Comp
L power:+5V #PWR01
U 1 1 5F65498B
P 6200 800
F 0 "#PWR01" H 6200 650 50  0001 C CNN
F 1 "+5V" H 6215 973 50  0000 C CNN
F 2 "" H 6200 800 50  0001 C CNN
F 3 "" H 6200 800 50  0001 C CNN
	1    6200 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 800  6200 850 
Text GLabel 6050 1350 0    50   Input ~ 0
T0
Wire Wire Line
	6050 1350 6200 1350
Connection ~ 6200 1350
Wire Wire Line
	6200 1350 6200 1250
$Comp
L Connector:Conn_01x03_Male J2
U 1 1 5F65AB3C
P 7400 950
F 0 "J2" H 7372 974 50  0000 R CNN
F 1 "WHTM-02A" H 7372 883 50  0000 R CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 7400 950 50  0001 C CNN
F 3 "~" H 7400 950 50  0001 C CNN
	1    7400 950 
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7200 850  6200 850 
Connection ~ 6200 850 
Wire Wire Line
	6200 850  6200 950 
Wire Wire Line
	7200 1050 7100 1050
Wire Wire Line
	7100 1050 7100 1850
Wire Wire Line
	7100 1850 6600 1850
Connection ~ 6200 1850
Wire Wire Line
	6200 1850 6200 1800
Text GLabel 7050 950  0    50   Input ~ 0
H0
Wire Wire Line
	7050 950  7200 950 
$Comp
L Regulator_Switching:LM2596S-5 U1
U 1 1 5F67239A
P 3850 1000
F 0 "U1" H 3850 1367 50  0000 C CNN
F 1 "LM2596S-5" H 3850 1276 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 3900 750 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2596.pdf" H 3850 1000 50  0001 C CNN
	1    3850 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 1100 3200 1100
$Comp
L Device:CP_Small C1
U 1 1 5F68390B
P 2750 1100
F 0 "C1" H 2838 1146 50  0000 L CNN
F 1 "680uF" H 2838 1055 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_5x5.3" H 2750 1100 50  0001 C CNN
F 3 "~" H 2750 1100 50  0001 C CNN
	1    2750 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1000 2750 900 
Wire Wire Line
	2750 900  3350 900 
$Comp
L Device:CP_Small C2
U 1 1 5F68A771
P 5150 1350
F 0 "C2" H 5238 1396 50  0000 L CNN
F 1 "220uF" H 5238 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_5x5.3" H 5150 1350 50  0001 C CNN
F 3 "~" H 5150 1350 50  0001 C CNN
	1    5150 1350
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N5822 D2
U 1 1 5F68B8A9
P 4500 1500
F 0 "D2" V 4454 1579 50  0000 L CNN
F 1 "1N5822" V 4545 1579 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 4500 1325 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88526/1n5820.pdf" H 4500 1500 50  0001 C CNN
	1    4500 1500
	0    1    1    0   
$EndComp
Wire Wire Line
	4500 1350 4500 1100
Wire Wire Line
	4500 1100 4350 1100
$Comp
L Device:L_Small L1
U 1 1 5F68F957
P 4800 1100
F 0 "L1" V 4900 1150 50  0000 C CNN
F 1 "33uH" V 4650 1100 50  0000 C CNN
F 2 "Inductor_SMD:L_6.3x6.3_H3" H 4800 1100 50  0001 C CNN
F 3 "~" H 4800 1100 50  0001 C CNN
	1    4800 1100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4700 1100 4500 1100
Connection ~ 4500 1100
Wire Wire Line
	5150 1100 5150 1250
Wire Wire Line
	5150 1450 5150 1700
Wire Wire Line
	5150 1700 4500 1700
Wire Wire Line
	4500 1700 4500 1650
Wire Wire Line
	3200 1700 3850 1700
Wire Wire Line
	3200 1100 3200 1700
Connection ~ 4500 1700
Wire Wire Line
	3850 1300 3850 1700
Connection ~ 3850 1700
Wire Wire Line
	3850 1700 4500 1700
Wire Wire Line
	2750 1700 3200 1700
Wire Wire Line
	2750 1200 2750 1700
Connection ~ 3200 1700
$Comp
L power:+5V #PWR02
U 1 1 5F6AEE05
P 5250 1100
F 0 "#PWR02" H 5250 950 50  0001 C CNN
F 1 "+5V" V 5265 1228 50  0000 L CNN
F 2 "" H 5250 1100 50  0001 C CNN
F 3 "" H 5250 1100 50  0001 C CNN
	1    5250 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	5250 1100 5150 1100
Connection ~ 5150 1100
Wire Wire Line
	4900 1100 5150 1100
Wire Wire Line
	4350 900  5150 900 
Wire Wire Line
	5150 900  5150 1100
$Comp
L dk_Barrel-Power-Connectors:PJ-102A J1
U 1 1 5F6C0520
P 1450 900
F 0 "J1" H 1383 1125 50  0000 C CNN
F 1 "12V-INPUT" H 1383 1034 50  0000 C CNN
F 2 "digikey-footprints:Barrel_Jack_5.5mmODx2.1mmID_PJ-102A" H 1650 1100 60  0001 L CNN
F 3 "https://www.cui.com/product/resource/digikeypdf/pj-102a.pdf" H 1650 1200 60  0001 L CNN
F 4 "CP-102A-ND" H 1650 1300 60  0001 L CNN "Digi-Key_PN"
F 5 "PJ-102A" H 1650 1400 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 1650 1500 60  0001 L CNN "Category"
F 7 "Barrel - Power Connectors" H 1650 1600 60  0001 L CNN "Family"
F 8 "https://www.cui.com/product/resource/digikeypdf/pj-102a.pdf" H 1650 1700 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/cui-inc/PJ-102A/CP-102A-ND/275425" H 1650 1800 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN PWR JACK 2X5.5MM SOLDER" H 1650 1900 60  0001 L CNN "Description"
F 11 "CUI Inc." H 1650 2000 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1650 2100 60  0001 L CNN "Status"
	1    1450 900 
	1    0    0    -1  
$EndComp
Connection ~ 2750 900 
$Comp
L power:GND #PWR04
U 1 1 5F6C565F
P 3850 1800
F 0 "#PWR04" H 3850 1550 50  0001 C CNN
F 1 "GND" H 3855 1627 50  0000 C CNN
F 2 "" H 3850 1800 50  0001 C CNN
F 3 "" H 3850 1800 50  0001 C CNN
	1    3850 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 1800 3850 1700
$Comp
L MCU_Microchip_ATmega:ATmega328P-AU U3
U 1 1 5F701799
P 1700 3950
F 0 "U3" H 1700 2361 50  0000 C CNN
F 1 "ATmega328P-AU" H 1100 2450 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 1700 3950 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 1700 3950 50  0001 C CNN
	1    1700 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5F718107
P 1700 5600
F 0 "#PWR013" H 1700 5350 50  0001 C CNN
F 1 "GND" H 1705 5427 50  0000 C CNN
F 2 "" H 1700 5600 50  0001 C CNN
F 3 "" H 1700 5600 50  0001 C CNN
	1    1700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5600 1700 5450
Text GLabel 4000 4450 2    50   Input ~ 0
X1
Wire Wire Line
	4000 3950 3900 3950
Connection ~ 3900 3950
Wire Wire Line
	3900 4450 3900 4300
Wire Wire Line
	4000 4450 3900 4450
Connection ~ 3900 4450
Text GLabel 2500 3350 2    50   Input ~ 0
X1
Wire Wire Line
	2500 3350 2300 3350
Text GLabel 2500 3450 2    50   Input ~ 0
X2
Wire Wire Line
	2500 3450 2300 3450
Text GLabel 2400 2850 2    50   Input ~ 0
E
Wire Wire Line
	2400 2850 2300 2850
Text GLabel 2400 2750 2    50   Input ~ 0
RS
Wire Wire Line
	2400 2750 2300 2750
Wire Wire Line
	1750 900  1550 900 
Wire Wire Line
	2150 900  2300 900 
$Comp
L Device:Q_PMOS_GDS Q1
U 1 1 5F6A1D5D
P 1950 1000
F 0 "Q1" V 2292 1000 50  0000 C CNN
F 1 "IPN70R360P7S " V 2201 1000 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223" H 2150 1100 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/196/Infineon-IPN70R360P7S-DS-v02_02-EN-1709872.pdf" H 1950 1000 50  0001 C CNN
	1    1950 1000
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F6B2AEE
P 1950 1450
F 0 "R1" H 1850 1450 50  0000 C CNN
F 1 "100K" H 2100 1450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1880 1450 50  0001 C CNN
F 3 "~" H 1950 1450 50  0001 C CNN
	1    1950 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	1950 1300 1950 1250
Wire Wire Line
	1950 1600 1950 1700
Wire Wire Line
	1950 1700 2750 1700
Connection ~ 2750 1700
$Comp
L Diode:BZT52Bxx D1
U 1 1 5F6BD82D
P 2300 1100
F 0 "D1" V 2254 1179 50  0000 L CNN
F 1 "BZT52B9V1-G" V 2500 900 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 2300 925 50  0001 C CNN
F 3 "https://eu.mouser.com/datasheet/2/427/bzt52g-1767393.pdf" H 2300 1100 50  0001 C CNN
	1    2300 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 1250 1950 1250
Connection ~ 1950 1250
Wire Wire Line
	1950 1250 1950 1200
Wire Wire Line
	2300 950  2300 900 
Connection ~ 2300 900 
Wire Wire Line
	2300 900  2750 900 
Wire Wire Line
	2400 3650 2300 3650
Text GLabel 2400 3750 2    50   Input ~ 0
RH
Wire Wire Line
	2400 3750 2300 3750
$Comp
L Connector_Generic:Conn_01x06 J5
U 1 1 5F7080ED
P 4750 2700
F 0 "J5" H 4830 2692 50  0000 L CNN
F 1 "SPI" H 4830 2601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 4750 2700 50  0001 C CNN
F 3 "~" H 4750 2700 50  0001 C CNN
	1    4750 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5F7086CF
P 4450 3100
F 0 "#PWR019" H 4450 2850 50  0001 C CNN
F 1 "GND" H 4455 2927 50  0000 C CNN
F 2 "" H 4450 3100 50  0001 C CNN
F 3 "" H 4450 3100 50  0001 C CNN
	1    4450 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 3100 4450 3000
Wire Wire Line
	4450 3000 4550 3000
$Comp
L power:+5V #PWR016
U 1 1 5F70ED25
P 4450 2400
F 0 "#PWR016" H 4450 2250 50  0001 C CNN
F 1 "+5V" H 4465 2573 50  0000 C CNN
F 2 "" H 4450 2400 50  0001 C CNN
F 3 "" H 4450 2400 50  0001 C CNN
	1    4450 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 2400 4450 2500
Wire Wire Line
	4450 2500 4550 2500
Text GLabel 4450 2900 0    50   Input ~ 0
RESET
Wire Wire Line
	4450 2900 4550 2900
Text GLabel 4450 2800 0    50   Input ~ 0
CLK_
Text GLabel 4450 2700 0    50   Input ~ 0
MISO
Text GLabel 4450 2600 0    50   Input ~ 0
MOSI
Wire Wire Line
	4450 2600 4550 2600
Wire Wire Line
	4450 2700 4550 2700
Wire Wire Line
	4450 2800 4550 2800
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 5F72F3A7
P 5950 2700
F 0 "J6" H 6030 2692 50  0000 L CNN
F 1 "UART" H 6030 2601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5950 2700 50  0001 C CNN
F 3 "~" H 5950 2700 50  0001 C CNN
	1    5950 2700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 5F72FC30
P 5700 2500
F 0 "#PWR017" H 5700 2350 50  0001 C CNN
F 1 "+5V" H 5715 2673 50  0000 C CNN
F 2 "" H 5700 2500 50  0001 C CNN
F 3 "" H 5700 2500 50  0001 C CNN
	1    5700 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2500 5700 2600
Wire Wire Line
	5700 2600 5750 2600
$Comp
L power:GND #PWR018
U 1 1 5F7366F0
P 5700 2950
F 0 "#PWR018" H 5700 2700 50  0001 C CNN
F 1 "GND" H 5705 2777 50  0000 C CNN
F 2 "" H 5700 2950 50  0001 C CNN
F 3 "" H 5700 2950 50  0001 C CNN
	1    5700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2950 5700 2900
Wire Wire Line
	5700 2900 5750 2900
Text GLabel 5700 2700 0    50   Input ~ 0
TX
Wire Wire Line
	5700 2700 5750 2700
Text GLabel 5700 2800 0    50   Input ~ 0
RX
Wire Wire Line
	5700 2800 5750 2800
Text GLabel 2400 4450 2    50   Input ~ 0
RX
Wire Wire Line
	2400 4450 2300 4450
Text GLabel 2400 4550 2    50   Input ~ 0
TX
Wire Wire Line
	2400 4550 2300 4550
Wire Wire Line
	1100 2750 950  2750
Wire Wire Line
	950  2750 950  2350
Wire Wire Line
	950  2350 1700 2350
$Comp
L Connector_Generic:Conn_01x10 J7
U 1 1 5F768C71
P 9550 1750
F 0 "J7" H 9630 1742 50  0000 L CNN
F 1 "LCD16L02" H 9630 1651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 9550 1750 50  0001 C CNN
F 3 "~" H 9550 1750 50  0001 C CNN
	1    9550 1750
	1    0    0    -1  
$EndComp
Text GLabel 9250 2250 0    50   Input ~ 0
D7
Text GLabel 9250 2150 0    50   Input ~ 0
D6
Text GLabel 9250 2050 0    50   Input ~ 0
D5
Text GLabel 9250 1950 0    50   Input ~ 0
D4
Wire Wire Line
	9250 1950 9350 1950
Wire Wire Line
	9250 2050 9350 2050
Wire Wire Line
	9250 2150 9350 2150
Wire Wire Line
	9250 2250 9350 2250
Text GLabel 9200 1850 0    50   Input ~ 0
E
Wire Wire Line
	9200 1850 9350 1850
Text GLabel 9200 1650 0    50   Input ~ 0
RS
Wire Wire Line
	9200 1650 9350 1650
$Comp
L power:GND #PWR09
U 1 1 5F7AF968
P 9150 1350
F 0 "#PWR09" H 9150 1100 50  0001 C CNN
F 1 "GND" H 9155 1177 50  0000 C CNN
F 2 "" H 9150 1350 50  0001 C CNN
F 3 "" H 9150 1350 50  0001 C CNN
	1    9150 1350
	0    1    1    0   
$EndComp
Wire Wire Line
	9150 1350 9300 1350
$Comp
L power:+5V #PWR07
U 1 1 5F7B7D98
P 8800 1300
F 0 "#PWR07" H 8800 1150 50  0001 C CNN
F 1 "+5V" H 8815 1473 50  0000 C CNN
F 2 "" H 8800 1300 50  0001 C CNN
F 3 "" H 8800 1300 50  0001 C CNN
	1    8800 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 1300 8800 1450
Wire Wire Line
	8800 1450 9350 1450
Text GLabel 8750 1550 0    50   Input ~ 0
VEE
Wire Wire Line
	8750 1550 8850 1550
Wire Wire Line
	9300 1750 9300 1350
Wire Wire Line
	9300 1750 9350 1750
Connection ~ 9300 1350
Wire Wire Line
	9300 1350 9350 1350
Wire Wire Line
	8400 1650 8850 1650
Wire Wire Line
	8850 1650 8850 1550
Connection ~ 8850 1550
Wire Wire Line
	8850 1550 9350 1550
$Comp
L Device:R R5
U 1 1 5F8334BA
P 6600 1600
F 0 "R5" V 6393 1600 50  0000 C CNN
F 1 "R" V 6484 1600 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 6530 1600 50  0001 C CNN
F 3 "~" H 6600 1600 50  0001 C CNN
	1    6600 1600
	-1   0    0    1   
$EndComp
Wire Wire Line
	6600 1750 6600 1850
Connection ~ 6600 1850
Wire Wire Line
	6600 1850 6200 1850
Wire Wire Line
	6600 1450 6600 1350
Wire Wire Line
	6600 1350 6200 1350
Text GLabel 4000 3950 2    50   Input ~ 0
X2
Text GLabel 2400 4250 2    50   Input ~ 0
RESET
Wire Wire Line
	2400 4250 2300 4250
Text GLabel 2950 3500 3    50   Input ~ 0
MOSI
Text GLabel 3100 3500 3    50   Input ~ 0
MISO
Text GLabel 3250 3500 3    50   Input ~ 0
CLK_
Wire Wire Line
	2950 3500 2950 3050
Connection ~ 2950 3050
Wire Wire Line
	2950 3050 2300 3050
Wire Wire Line
	3100 3500 3100 3150
Connection ~ 3100 3150
Wire Wire Line
	3100 3150 2300 3150
Wire Wire Line
	3250 3500 3250 3250
Connection ~ 3250 3250
Wire Wire Line
	3250 3250 2300 3250
$Comp
L Switch:SW_Push SW1
U 1 1 5F6BF039
P 3900 6300
F 0 "SW1" H 3900 6585 50  0000 C CNN
F 1 "B+" H 3900 6494 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H8mm" H 3900 6500 50  0001 C CNN
F 3 "~" H 3900 6500 50  0001 C CNN
	1    3900 6300
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 5F6D6337
P 3900 6600
F 0 "SW3" H 3900 6500 50  0000 C CNN
F 1 "B-" H 3900 6794 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H8mm" H 3900 6800 50  0001 C CNN
F 3 "~" H 3900 6800 50  0001 C CNN
	1    3900 6600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 5F6D658B
P 4350 6000
F 0 "#PWR0101" H 4350 5850 50  0001 C CNN
F 1 "+5V" H 4365 6173 50  0000 C CNN
F 2 "" H 4350 6000 50  0001 C CNN
F 3 "" H 4350 6000 50  0001 C CNN
	1    4350 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 6300 4350 6300
Wire Wire Line
	4350 6300 4350 6000
Wire Wire Line
	4100 6600 4350 6600
Wire Wire Line
	4350 6600 4350 6300
Connection ~ 4350 6300
$Comp
L Device:R R8
U 1 1 5F6EF68C
P 3600 6850
F 0 "R8" H 3700 6950 50  0000 C CNN
F 1 "10K" H 3700 6850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3530 6850 50  0001 C CNN
F 3 "~" H 3600 6850 50  0001 C CNN
	1    3600 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3600 6700 3600 6600
Wire Wire Line
	3600 6600 3700 6600
$Comp
L Device:R R7
U 1 1 5F6F8C85
P 3250 6850
F 0 "R7" H 3400 6950 50  0000 C CNN
F 1 "10K" H 3400 6850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 3180 6850 50  0001 C CNN
F 3 "~" H 3250 6850 50  0001 C CNN
	1    3250 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 6700 3250 6300
Wire Wire Line
	3250 6300 3700 6300
$Comp
L power:GND #PWR0102
U 1 1 5F701A5E
P 3250 7100
F 0 "#PWR0102" H 3250 6850 50  0001 C CNN
F 1 "GND" H 3255 6927 50  0000 C CNN
F 2 "" H 3250 7100 50  0001 C CNN
F 3 "" H 3250 7100 50  0001 C CNN
	1    3250 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 7100 3250 7050
Wire Wire Line
	3600 7000 3600 7050
Wire Wire Line
	3600 7050 3250 7050
Connection ~ 3250 7050
Wire Wire Line
	3250 7050 3250 7000
Text GLabel 3150 6300 0    50   Input ~ 0
B+
Wire Wire Line
	3150 6300 3250 6300
Connection ~ 3250 6300
Text GLabel 3150 6600 0    50   Input ~ 0
B-
Wire Wire Line
	3150 6600 3600 6600
Connection ~ 3600 6600
$Comp
L Switch:SW_Push_SPDT SW2
U 1 1 5F726608
P 5200 6400
F 0 "SW2" H 5200 6685 50  0000 C CNN
F 1 "SW_Push_SPDT" H 5200 6594 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 5200 6400 50  0001 C CNN
F 3 "~" H 5200 6400 50  0001 C CNN
	1    5200 6400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5F727BCA
P 4850 6800
F 0 "R6" H 5000 6850 50  0000 C CNN
F 1 "10K" H 5000 6750 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 4780 6800 50  0001 C CNN
F 3 "~" H 4850 6800 50  0001 C CNN
	1    4850 6800
	-1   0    0    1   
$EndComp
Connection ~ 3600 7050
Wire Wire Line
	4850 6950 4850 7050
Wire Wire Line
	3600 7050 4850 7050
Wire Wire Line
	4850 6650 4850 6400
Wire Wire Line
	4850 6400 5000 6400
Text GLabel 4800 6400 0    50   Input ~ 0
PAGE
Wire Wire Line
	4800 6400 4850 6400
Connection ~ 4850 6400
$Comp
L power:+5V #PWR0103
U 1 1 5F762B91
P 5600 6200
F 0 "#PWR0103" H 5600 6050 50  0001 C CNN
F 1 "+5V" H 5615 6373 50  0000 C CNN
F 2 "" H 5600 6200 50  0001 C CNN
F 3 "" H 5600 6200 50  0001 C CNN
	1    5600 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 6200 5600 6300
Wire Wire Line
	5600 6300 5400 6300
Wire Wire Line
	5400 6500 5500 6500
Wire Wire Line
	5500 6500 5500 7050
Wire Wire Line
	5500 7050 4850 7050
Connection ~ 4850 7050
Wire Wire Line
	1550 1100 1600 1100
Wire Wire Line
	1600 1100 1600 1700
Wire Wire Line
	1600 1700 1950 1700
Connection ~ 1950 1700
Wire Wire Line
	1550 1000 1600 1000
Wire Wire Line
	1600 1000 1600 1100
Connection ~ 1600 1100
Text Label 8900 1750 0    50   ~ 0
RW
Wire Wire Line
	8900 1750 9300 1750
Connection ~ 9300 1750
Text GLabel 2450 4050 2    50   Input ~ 0
T0
Wire Wire Line
	2450 4050 2300 4050
Text GLabel 2450 4150 2    50   Input ~ 0
H0
Wire Wire Line
	2450 4150 2300 4150
$EndSCHEMATC