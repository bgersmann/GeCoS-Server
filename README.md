# GeCoS-Server
Python GeCoS Server Script. Ansprechbar per WebSocket<br>


### Befehlsaufbau:<br>
Befehl in geschweiften klammern verpackt: ```{}``` <br>
Trennzeichen: ```;``` <br>
Erste Info: Funktion (x-Buchstaben) <br>
2. Info: BUS 0-2 <br>
3. Info: Modul Adresse <br> 
n. Sachinformation (z.B.: Port Status) <br> 
Antwort mit Befehl + Status <br>

Bsp.: Output setzen, alle Ausgänge einschalten:  <br> 
Befehl:     ```{SOM;1;0x24;65535}``` <br>
Antwort:    ```{SOM;1;0x24;65535;OK}``` <br>

Es werden nur Module ausgelesen die bei der Modulsuche(MOD) gefunden wurde. <br>

### Funktionen:<br>
"SAI" = Status All IN -> Liest alle Eingangsmodule und sendet aktuellen Status<br>
"SAO" = Status All Out -> Liest alle Ausgangsmodule und sendet aktuellen Status<br>
"MOD" = Modulsuche -> Sucht nach Modulen und Antwortet mit Moduladressen<br>
"SPWM" = Status All PWM -> Liest alle PWM Module aus und sendet aktuellen Status<br>
"SRGBW" = Status All RGBW -> Liest alle RGBW Module aus und sendet aktuellen Status<br>
"SOM" = "Set Output Module" -> INT Big = port A; Litte = Port B<br>
"PWM" = "Set PWM Module" -> {PWM;I2C-Kanal;Adresse;PWMKanal;Status;Wert} Status=0/1 (0=Aus,1=Ein), Wert=0-4095<br>
"RGBW" = "Set RGBW Module" -> {RGBW;I2C-Kanal;Adresse;RGBWKanal;StatusRGB;StatusW;R;G;B;W} Status=0/1 (0=Aus,1=Ein), R/G/B/W=0-4095 <br>
"SAM" = "Status Analog Module" -> {SAM;0;0x69;AnalogChannel;Resolution;Amplifier}  <br>
"RRTC" = Read RTC  -> {RRTC} -> {RRTC;TT;MM;JJJJ;HH;MM;SS;OK}  <br>
"SRTC" = Set RTC    ->  {SRTC;TT;MM;JJJJ;HH;MM;SS;TEMP} <br>
"OWS" = One Wire Suche -> {OWS} <br>
"OWV" = One Wire Values -> {OWV;OWDevice ID}  (```{OWV;28-610119138fdf1b}``` -> ```{OWV;28-610119138fdf1b;23.568;OK}```) <br>
"OWC" = One Wire Config -> {OWC;OWDevice ID;Werte}  (```{OWC;28-610119138fdf1b;31}``` -> ```{OWC;28-610119138fdf1b;31;OK}```) -> Typ abhängig! <br> 
Kanal 0-2<br>

### MOD - Antworten<br>
```{MOD;0;0x24;OUT}```    -> 16Out erkannt<br>
```{MOD;0;0x20;IN}```     -> 16In erkannt<br>
```{MOD;0;0x50;PWM}```    -> PWM erkannt<br>
```{MOD;0;0x58;RGBW}```   -> RGBW erkannt<br>
```{MOD;0;0x68;ANA}```    -> Analog erkannt<br>
```{MOD;0;0x05;UNB}```    -> Unbekanntes i2c device<br>

## OWS -Antwort <br>
```{OWS;28-610119138fdf1b}}``` -> DS18B20<br>
```{OWS;10-460008037049d3}```   -> DS18S20<br>
```{OWS;3a-1d000000455f51}``` -> DS2413<br>
```{OWS;3b-910cfc09590677}``` -> MAX31850<br>


## OWC - Optionen <br>
FamilyCode ```28```:<br>
Resolution: ```31``` -> 9Bit, ```63```-> 10Bit, ```95```-> 11Bit, ```127```-> 12Bit<br>
```{OWC;28-610119138fdf1b;127}``` -> ```{OWC;28-610119138fdf1b;127;OK}```<br>
FamilyCode ```3a```:<br>
Value: ```0``` -> IOA+IOB = LOW, ```1```-> IOA = HIGH;IOB = LOW, ```2```-> IOA = LOW; IOB = High, ```3```-> IOA+IOB = High<br>
```{OWC;28-610119138fdf1b;3}``` -> ```{OWC;28-610119138fdf1b;3;OK}```<br>

### Download und Einrichten:<br>
Download des git Verzeichnisses:<br>
`git clone https://github.com/bgersmann/GeCoS-Server`<br>
Kopiere Symcon.py zu /usr/local/bin/<br>
`sudo mv /home/pi/GeCoS-Server/Symcon.py /usr/local/bin/`<br>
`sudo rm -r GeCoS-Server`<br>

Erstellen des Services:<br>
`sudo nano /lib/systemd/system/gecos.service`<br>
Folgenden Inhalt einfügen: <br>
```
[Unit]
Description=GeCoS WebService
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/local/bin/Symcon.py
Restart=on-abort

[Install]
WantedBy=multi-user.target
```
Einrichten und Starten des Services: <br>
`sudo chmod 644 /lib/systemd/system/gecos.service`<br>
`chmod +x /usr/local/bin/Symcon.py`<br>
`sudo systemctl daemon-reload`<br>
`sudo systemctl enable gecos.service`<br>
`sudo systemctl start gecos.service`<br>
`sudo systemctl status gecos.service`<br>
`sudo systemctl restart gecos.service`<br>
`sudo systemctl stop gecos.service`<br>



