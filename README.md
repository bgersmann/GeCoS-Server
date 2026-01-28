# GeCoS-Server

Python GeCoS Server Script - Gebäudecontrol System Server

Steuert I2C-basierte Ein-/Ausgangsmodule über TCP/IP

## Features

- I2C-Multiplexer Unterstützung (TCA9548A)
- TCP-basierte Kommunikation
- OneWire-Unterstützung (DS18B20, DS18S20, DS2413, MAX31850)
- RTC-Integration (DS1307/DS3231)
- PWM-Steuerung (PCA9685)
- RGBW-LED-Steuerung
- Analog-Eingänge (MCP3424)
- 16 Ein-/Ausgänge pro Modul (MCP23017)

## Anforderungen

- Python 3.6+
- Raspberry Pi mit I2C aktiviert
- Benötigte Python-Pakete:
  - smbus
  - argparse (Standard-Bibliothek)
  - threading (Standard-Bibliothek)

## Befehlsaufbau

**Parameter:**
- `--debug` oder `-d` → Zeigt Debug-Informationen

**Befehlsstruktur:**
- Befehl in geschweiften Klammern: `{}`
- Trennzeichen: `;`
- 1. Info: Funktion (Buchstaben-Code)
- 2. Info: I2C-Bus (0-2)
- 3. Info: Modul-Adresse
- n. Info: Sachinformation (z.B. Port-Status)
- Antwort: Befehl + Status

**Beispiel - Output setzen:**
```
Befehl:  {SOM;1;0x24;65535}
Antwort: {SOM;1;0x24;65535;OK}
```

> **Hinweis:** Es werden nur Module ausgelesen, die bei der Modulsuche (`MOD`) gefunden wurden.

## Funktionen

### Allgemeine Befehle

| Befehl | Beschreibung | Beispiel |
|--------|-------------|----------|
| `MOD` | Modulsuche - Sucht nach allen I2C-Modulen | `{MOD}` |
| `SAI` | Status All IN - Liest alle Eingangsmodule | `{SAI}` |
| `SAO` | Status All OUT - Liest alle Ausgangsmodule | `{SAO}` |
| `SPWM` | Status All PWM - Liest alle PWM-Module | `{SPWM}` |
| `SRGBW` | Status All RGBW - Liest alle RGBW-Module | `{SRGBW}` |

### Modul-Steuerung

**Output-Modul (16 Ausgänge):**
```
{SOM;Kanal;Adresse;Wert}
Beispiel: {SOM;0;0x24;65535}  // Alle Ausgänge EIN
```

**PWM-Modul:**
```
{PWM;Kanal;Adresse;PWM-Kanal;Status;Wert}
- Status: 0=Aus, 1=Ein
- Wert: 0-4095
Beispiel: {PWM;0;0x50;0;1;2048}
```

**RGBW-Modul:**
```
{RGBW;Kanal;Adresse;RGBW-Kanal;StatusRGB;StatusW;R;G;B;W}
- Status: 0=Aus, 1=Ein
- R/G/B/W: 0-4095
Beispiel: {RGBW;0;0x58;0;1;1;4095;0;0;4095}
```

**Analog-Modul:**
```
{SAM;Kanal;Adresse;Analog-Kanal;Resolution;Amplifier}
Beispiel: {SAM;0;0x69;0;3;0}
```

### RTC (Real-Time Clock)

**RTC lesen:**
```
{RRTC}
Antwort: {RRTC;TT;MM;JJJJ;HH;MM;SS;Temp;OK}
```

**RTC setzen:**
```
{SRTC;TT;MM;JJJJ;HH;MM;SS}
Beispiel: {SRTC;22;10;2025;14;30;00}
```

### OneWire-Befehle

**OneWire-Suche:**
```
{OWS}
Antwort: {OWS;Device-ID} (mehrfach)
         {OWS;END}
```

**OneWire-Wert lesen:**
```
{OWV;Device-ID}
Beispiel: {OWV;28-610119138fdf1b}
Antwort:  {OWV;28-610119138fdf1b;23.568;OK}
```

**OneWire-Konfiguration:**
```
{OWC;Device-ID;Wert}
Beispiel: {OWC;28-610119138fdf1b;127}
Antwort:  {OWC;28-610119138fdf1b;127;OK}
```

## Modulerkennung (MOD-Antworten)

```
{MOD;Kanal;Adresse;Typ}
```

| Antwort | Beschreibung |
|---------|-------------|
| `{MOD;0;0x24;OUT}` | 16-Bit Output-Modul (MCP23017) |
| `{MOD;0;0x20;IN}` | 16-Bit Input-Modul (MCP23017) |
| `{MOD;0;0x50;PWM}` | PWM-Modul (PCA9685) |
| `{MOD;0;0x58;RGBW}` | RGBW-Modul (PCA9685) |
| `{MOD;0;0x68;ANA}` | Analog-Modul (MCP3424) |
| `{MOD;0;0x05;UNB}` | Unbekanntes I2C-Gerät |
| `{MOD;0;0;END}` | Suche beendet |

## OneWire-Geräte

### Unterstützte Geräte

| Family Code | Gerät | Beispiel-ID |
|-------------|-------|-------------|
| `28` | DS18B20 | `{OWS;28-610119138fdf1b}` |
| `10` | DS18S20 | `{OWS;10-460008037049d3}` |
| `3a` | DS2413 | `{OWS;3a-1d000000455f51}` |
| `3b` | MAX31850 | `{OWS;3b-910cfc09590677}` |

### OneWire-Konfiguration (OWC)

**DS18B20 (Family Code 28) - Auflösung:**
- `31` → 9 Bit
- `63` → 10 Bit
- `95` → 11 Bit
- `127` → 12 Bit (Standard)

```
{OWC;28-610119138fdf1b;127}
```

**DS2413 (Family Code 3a) - Ausgänge:**
- `0` → IOA+IOB = LOW
- `1` → IOA = HIGH, IOB = LOW
- `2` → IOA = LOW, IOB = HIGH
- `3` → IOA+IOB = HIGH

```
{OWC;3a-1d000000455f51;3}
```

## Download und Einrichten:<br>
Download des git Verzeichnisses:<br>
`git clone https://github.com/bgersmann/GeCoS-Server`<br>
Kopiere GeCoS-Server.py zu /usr/local/bin/<br>
`sudo mv /home/pi/GeCoS-Server/GeCoS-Server.py /usr/local/bin/`<br>
`sudo mv /home/pi/GeCoS-Server/Mux.py /usr/local/bin/`<br>
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
ExecStart=/usr/bin/python3 /usr/local/bin/GeCoS-Server.py
Restart=on-abort

[Install]
WantedBy=multi-user.target
```
Einrichten und Starten des Services: <br>
`sudo chmod 644 /lib/systemd/system/gecos.service`<br>
`sudo chmod +x /usr/local/bin/GeCoS-Server.py`<br>
`sudo systemctl daemon-reload`<br>
`sudo systemctl enable gecos.service`<br>
`sudo systemctl start gecos.service`<br>
`sudo systemctl status gecos.service`<br>
`sudo systemctl restart gecos.service`<br>
`sudo systemctl stop gecos.service`<br>



### Alternativ:<br>
`sudo wget -O /usr/local/bin/GeCoS-Server.py https://raw.githubusercontent.com/bgersmann/GeCoS-Server/master/GeCoS-Server.py`<br>
`sudo wget -O /usr/local/bin/Mux.py https://raw.githubusercontent.com/bgersmann/GeCoS-Server/master/Mux.py`<br>
`sudo wget -O /lib/systemd/system/gecos.service https://raw.githubusercontent.com/bgersmann/GeCoS-Server/master/gecos.service`<br>
`sudo chmod 644 /lib/systemd/system/gecos.service`<br>
`sudo chmod +x /usr/local/bin/GeCoS-Server.py`<br>
`sudo systemctl daemon-reload`<br>
`sudo systemctl enable gecos.service`<br>
`sudo systemctl start gecos.service`<br>