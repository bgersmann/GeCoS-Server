# GeCoS-Server
Python GeCoS Server Script. Ansprechbar per WebSocket


Befehlaufbau:
Befehl in geschweiften klammern verpackt: {}
Trennzeichen: ;
Erste Info: Funktion (3Buchstaben)
2. Info: BUS 0-2
3. Info: Modul Adresse
n. Sachinformation (z.B.: Port Status)
Antwort mit Befehl + Status

Bsp.: Output setzen, alle Ausgänge einschalten: 
Befehl:     {SOM;1;0x24;65535;}
Antwort:    {SOM;1;0x24;65535;OK}

Es werden nur Module ausgelesen die bei der Modulsuche(MOD) gefunden wurde. 

Funktionen:
"SAI" = Status All IN -> Liest alle Eingangsmodule und sendet aktuellen Status
"SAO" = Status All Out -> Liest alle Ausgangsmodule und sendet aktuellen Status
"MOD" = Modulsuche -> Sucht nach Modulen und Antwortet mit Moduladressen
"SPWM" = Status All PWM -> Liest alle PWM Module aus und sendet aktuellen Status
"SRGBW" = Status All RGBW -> Liest alle RGBW Module aus und sendet aktuellen Status
"SOM" = "Set Output Module" -> INT Big = port A; Litte = Port B
"PWM" = "Set PWM Module" -> {PWM;I2C-Kanal;Adresse;Kanal;Status;Wert} Status=0/1 (0=Aus,1=Ein), Wert=0-100
"RGBW" = "Set RGBW Module" -> {RGBW;I2C-Kanal;Adresse;StatusW;StatusRGB;R;G;B;W} Status=0/1 (0=Aus,1=Ein), R/G/B/W=0-100
"SAM" = "Status Analog Module" -> {SAM;0;0x69;AnalogChannel;Resolution;Amplifier}

Kanal 0-2

MOD - Antworten
{MOD;0;0x24;OUT}    -> 16Out erkannt
{MOD;0;0x20;IN}     -> 16In erkannt
{MOD;0;0x50;PWM}    -> PWM erkannt
{MOD;0;0x58;RGBW}   -> RGBW erkannt
{MOD;0;0x68;ANA}    -> Analog erkannt
{MOD;0;0x05;UNB}    -> Unbekanntes i2c device

