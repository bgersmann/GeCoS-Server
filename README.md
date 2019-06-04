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
"SAP" = Status All PWM -> Liest alle PWM Module aus und sendet aktuellen Status
"SOM" = "Set Output Module" -> INT Big = port A; Litte = Port B
"SPM" = "Set PWM Module"
"SAM" = "Set Analog Module"

Kanal 0-2