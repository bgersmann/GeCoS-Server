#!/usr/bin/env python3
# encoding=utf-8
"""
GeCoS-Server
Steuert I2C-basierte Ein-/Ausgangsmodule über TCP/IP
"""

import smbus
import time
import sys
import logging
from datetime import datetime
import socket
import threading
import configparser
import os
import argparse
from typing import Optional, List
from Mux import multiplex

# Logging konfigurieren
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Status Variable 16IN 1x pro Bus mit 8 Werten
stat_in = {
    0: [0] * 8,
    1: [0] * 8,
    2: [0] * 8
}

# Globale Variablen
print_debug = False
status_ow = 1
cl_socket: Optional[socket.socket] = None
cl_ip = ""

# Module-Arrays pro Kanal
modules = {
    'in': {0: [], 1: [], 2: []},
    'out': {0: [], 1: [], 2: []},
    'pwm': {0: [], 1: [], 2: []},
    'rgbw': {0: [], 1: [], 2: []},
    'ana': {0: [], 1: [], 2: []}
}

I2C_ADR_DS2482 = 0x18  # Adresse DS2482

# DS2482 - OneWire-Bridge-Klasse
class DS2482:
    """
    DS2482 OneWire-zu-I2C-Bridge Klasse
    
    Ermöglicht die Kommunikation mit OneWire-Geräten über I2C
    """
    
    I2C_ADDR = I2C_ADR_DS2482
    
    def __init__(self):
        self._bus = smbus.SMBus(bus)
        self._owDeviceAddress = [0, 0]
        self._owTripletDirection = 1
        self._owTripletFirstBit = 0
        self._owTripletSecondBit = 0
        self._owLastDevice = 0
        self._owLastDiscrepancy = 0

    def OWSearchBus(self):
        """Sucht alle OneWire-Geräte auf dem Bus"""
        try:
            while self.OWSearch() == 1:
            #if (self.OWSearch()):
                device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
                befehl="{OWS;"
                befehl +="{0}".format(device)
                befehl+="}"
                send_tcp(befehl)
                log("Gerät gefanden: " + str(device), "INFO")
            befehl="{OWS;END}"
            send_tcp(befehl)
        except Exception as e:
            befehl="{OWS;Fehler}"
            send_tcp(befehl)
            log(f"Fehler bei OWS Suche: {e}", "ERROR")

    def DS2482Reset(self):
        """Setzt den DS2482 zurück"""
        self._bus.write_byte(self.I2C_ADDR, 0xF0)

    def OWStatusRegister(self):
        """Liest das Status-Register"""
        self._bus.write_byte_data(self.I2C_ADDR, 0xE1, 0xF0)
        e = self._bus.read_byte(self.I2C_ADDR)
        return e

    def OWReset(self):
        """Setzt den OneWire-Bus zurück"""
        self._bus.write_byte(self.I2C_ADDR, 0xB4)  # 1Wire Reset
        loopcount=0
        data=""    
        while (True):
            loopcount+=1
            data=self.OWStatusRegister() 
            if (data is None):
                #Fehler beim Lesen
                return 0
            else:
                if (data & 0x01):
                    #1Wire belegt
                    if (loopcount>100):
                        return 0
                else:
                    if (data & 0x04):
                        #Short detect bit
                        return 0
                    if (data & 0x02):
                        #Presense-Pulse Detect bit
                        break
                    else:
                        #Keine OW geräte gefunden
                        return 0
        return 1


    def OWWriteByte(self,byte):
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xF0)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte_data(self.I2C_ADDR,0xA5, byte)
        loopcount=0
        while (True):
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break   
        return 0


    def OWReadByte(self):
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xF0)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte(self.I2C_ADDR,0x96)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xE1)
        data=self._bus.read_byte(self.I2C_ADDR)
        if(data is None):
                #Fehler
                return -1
        return data
                



    
    def OWTriplet(self):
        if (self._owTripletDirection > 0):
            self._owTripletDirection = 0xFF
        self._bus.write_byte_data(self.I2C_ADDR, 0x78,self._owTripletDirection)
        loopcount = 0
        while (True):
            loopcount+=1
            data =self.OWStatusRegister()
            if (data is None):
                return -1
            else:
                if (data & 0x01):
                    if (loopcount > 100):
                        return -1
                else:
                    if (data & 0x20):
                        self._owTripletFirstBit = 1
                    else:
                        self._owTripletFirstBit = 0
                    if (data & 0x40):
                        self._owTripletSecondBit = 1
                    else:
                        self._owTripletSecondBit = 0
                    if (data & 0x80):
                        self._owTripletDirection = 1
                    else:
                        self._owTripletDirection = 0
                    return 1


    def OWSearch(self):
        #global owDeviceAddress
        self._bitNumber=1
        self._lastZero=0
        self._deviceAddress4ByteIndex=1 #Fill last 4 bytes first, data from onewire comes LSB first.
        self._deviceAddress4ByteMask=1
        
        if (self._owLastDevice):
            #Letzte adresse:
            self._owLastDevice=0
            self._owLastDiscrepancy=0
            self._owDeviceAddress[0] = 0xFFFFFFFF
            self._owDeviceAddress[1] = 0xFFFFFFFF
        else:
            if not (self.OWReset()):
                self._owLastDiscrepancy = 0
                return 0
        
            self.OWWriteByte(0xF0)
            while (self._deviceAddress4ByteIndex > -1):
                if (self._bitNumber < self._owLastDiscrepancy):
                    if (self._owDeviceAddress[self._deviceAddress4ByteIndex] & self._deviceAddress4ByteMask):
                        self._owTripletDirection = 1
                    else:
                        self._owTripletDirection = 0
                elif (self._bitNumber == self._owLastDiscrepancy): #if equal to last pick 1, if not pick 0
                    self._owTripletDirection = 1
                else:
                    self._owTripletDirection = 0
                
                if not (self.OWTriplet()):
                    return 0

                if (self._owTripletFirstBit==0 and self._owTripletSecondBit==0 and self._owTripletDirection==0):
                    self._lastZero = self._bitNumber
                if (self._owTripletFirstBit==1 and self._owTripletSecondBit==1):
                    break
                if (self._owTripletDirection==1):
                    self._owDeviceAddress[self._deviceAddress4ByteIndex] = self._owDeviceAddress[self._deviceAddress4ByteIndex] | self._deviceAddress4ByteMask
                else:
                    self._owDeviceAddress[self._deviceAddress4ByteIndex] = self._owDeviceAddress[self._deviceAddress4ByteIndex] & (~self._deviceAddress4ByteMask)
                self._bitNumber+=1 #Counter hochsetzen
                self._deviceAddress4ByteMask = (self._deviceAddress4ByteMask << 1) & 0xFFFFFFFF #shift the bit mask left
                if (self._deviceAddress4ByteMask == 0): #if the mask is 0 then go to other address block and reset mask to first bit
                    self._deviceAddress4ByteIndex=self._deviceAddress4ByteIndex-1
                    self._deviceAddress4ByteMask = 1

            if (self._bitNumber == 65): #if the search was successful then
                self._owLastDiscrepancy = self._lastZero
                if (self._owLastDiscrepancy==0):
                    self._owLastDevice = 1
                else:
                    self._owLastDevice = 0
                
                #serialnumber=owDeviceAddress[0][0]<<32 | owDeviceAddress[0][1]
                if (self.OWCheckCRC()):
                    #CRC OK
                    return 1
                else:
                    #CRC NICHT OK
                    return 0
        self._owLastDiscrepancy = 0
        self._owLastDevice = 0
        return 0


    def OWCheckCRC(self):
        crc = 0
        da32bit= self._owDeviceAddress[1]
        for j in range(0,4):
            crc = self.AddCRC(da32bit & 0xFF, crc)
            da32bit = da32bit >> 8 #Shift right 8 bits
        da32bit = self._owDeviceAddress[0]
        for j in range(0,3):
            crc = self.AddCRC(da32bit & 0xFF, crc)
            da32bit = da32bit >> 8 #Shift right 8 bits
        if ((da32bit & 0xFF) == crc): #last byte of address should match CRC of other 7 bytes
            return 1 #match
        return 0 #bad CRC


    def AddCRC(self,inbyte, crc):
        for j in range(0,8):
            mix = (crc ^ inbyte) & 0x01
            crc = crc >> 1
            if (mix):
                crc = crc ^ 0x8C
            inbyte = inbyte >> 1
        return crc


    def OWSelect(self):
        self.OWWriteByte(0x55) #Issue the Match ROM command
        #for i in range(1,-1,-1):
        da32bit = self._owDeviceAddress[1]
        for j in range(0,4):
            self.OWWriteByte(da32bit & 0xFF) #Send lowest byte
            da32bit = da32bit >> 8 #Shift right 8 bits
        da32bit = self._owDeviceAddress[0]
        for j2 in range(0,4):
            self.OWWriteByte(da32bit & 0xFF) #Send lowest byte
            da32bit = da32bit >> 8 #Shift right 8 bits

    def OWSelectAdress(self,OWAdr):
        #"28-a601183074cbff" -> a601183074cbff28
        try:
            x = OWAdr.split("-")
            tmp2 = "0x" + x[1] + x[0]
            tmp = int(tmp2, 16)
            self._owDeviceAddress[1] = tmp & 0xFFFFFFFF
            self._owDeviceAddress[0] = tmp >> 32
            return True
        except (ValueError, IndexError) as e:
            log(f"Fehler beim OneWire Adresse einstellen: {e}", "ERROR")
            return False

    def DS2413OWSetConfig(self,data):
        try:
            self.OWSelect()
            self.OWWriteByte(0x5a)
            self.OWWriteByte(data)
            data = ~data&0xFF
            self.OWWriteByte(data)
            return True
        except Exception as e:
            log(f"Fehler beim DS2413 Config setzen: {e}", "ERROR")
            return False

    def DS18B20OWSetConfig(self,res):
        try:
            # 31, 63, 95, 127 9/10/11/12Bit
            self.OWSelect()
            self.OWWriteByte(78)
            self.OWWriteByte(0)
            self.OWWriteByte(0)
            self.OWWriteByte(res)
            return True
        except Exception as e:
            log(f"Fehler beim DS18B20 Config setzen: {e}", "ERROR")
            return False
        

    def DS18B20OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x28): #Ist ein DS18B20
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.760) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte

            data = [0,0,0,0,0,0,0,0,0]
            for i in range(0,9):                
                data[i] = self.OWReadByte()
               
            crc = 0
            for j in range(0,8):
                crc = self.AddCRC(data[j], crc)
            
            if data[8] != crc:
                 celsius=-85
                 return celsius

            raw = (data[1] << 8) | data[0]
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment
            cfg = data[4] & 0x60
            if (cfg == 0x60):
                raw=raw
                #nix tun
            elif (cfg == 0x40):
                #raw = raw & 0xFFFE
                raw = raw << 1
            elif (cfg == 0x20):
                #raw = raw & 0xFFFC
                raw = raw << 2
            else:
                #raw = raw & 0xFFF8
                raw = raw << 3

            celsius = raw / 16.0
            if (SignBit):
                celsius = celsius * (-1)
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log(f"Fehler 1Wire DS18B20 {device}: {e}","ERROR")
        finally:
            return celsius

    def DS2413GetState(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x3a): #Ist ein DS2413
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0xF5) #Starte Messung
                    result = self.OWReadByte()
        except Exception as e:
            result=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log(f"Fehler 1Wire DS2413 {device}: {e}","ERROR")
        finally:
            return result


    def MAX31850OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x3B): #Ist ein MAX31850
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.100) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte

            data = [0,0,0,0]
            for i in range(0,4):
                data[i] = self.OWReadByte()

            raw = (data[1] << 8) | data[0] & 0xFC
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment

            if (data[0]&0X01==1): # Auf fehler prüfen
                celsius=-85
                device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
                log("Device: " + str(device) + " Temp: " + str(celsius),"ERROR")
                return celsius

            celsius= raw * 0.0625
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log(f"Fehler 1Wire MAX31850 {device}: {e}","ERROR")
        finally:
            return celsius



    def DS18S20OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x10): #Ist ein DS18S20
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.750) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte


            data = [0,0]
            for i in range(0,2):
                data[i] = self.OWReadByte()

            raw = (data[1] << 8) | data[0]
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment
            
            celsius = raw / 2.0
            if (SignBit):
                celsius = celsius * (-1)
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log(f"Fehler 1Wire DS18S20 {device}: {e}","ERROR")
        finally:
            return celsius


#RTC:
def _bcd_to_int(x):
    # Decode 2x4 bit BCD to byte value
    return int((x//16)*10 + x%16)

def _int_to_bcd(x):
    # Encode byte value to BCD
    return int((x//10)*16 + x%10)

#http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-RTC/DS1307_lib.py
class DS1307():
    DS_REG_SECONDS = 0x00
    DS_REG_MINUTES = 0x01
    DS_REG_HOURS   = 0x02
    DS_REG_DOW     = 0x03
    DS_REG_DAY     = 0x04
    DS_REG_MONTH   = 0x05
    DS_REG_YEAR    = 0x06
    DS_REG_CONTROL = 0x07
    DS_REG_TEMP_HSB = 0x11
    DS_REG_TEMP_LSB = 0x12
    

    def __init__(self, bus, addr=0x68):
        self._bus = bus #smbus.SMBus(twi)
        self._addr = addr

    def _read_seconds(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_SECONDS))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_SECONDS))
    
    def _read_minutes(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_MINUTES))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_MINUTES))

    def _read_hours(self):
        d = self._bus.readByteData(3,self._addr, self.DS_REG_HOURS)
        #d = self._bus.read_byte_data(self._addr, self.DS_REG_HOURS)
        if (d == 0x64):    # 12-Std.-Modus
            if ((d & 0b00100000) > 0):
                # Umrechnen auf 24-Std.-Modus
                return _bcd_to_int(d & 0x3F) + 12
        return _bcd_to_int(d & 0x3F)

    def _read_dow(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_DOW))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_DOW))

    def _read_day(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_DAY))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_DAY))

    def _read_month(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_MONTH)&0b01111111)
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_MONTH)&0b01111111)

    def _read_year(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_YEAR))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_YEAR))

    def read_temp(self):
        byte_tmsb = self._bus.readByteData(3,self._addr,self.DS_REG_TEMP_HSB)
        byte_tlsb = bin(self._bus.readByteData(3,self._addr,self.DS_REG_TEMP_LSB))[2:].zfill(8)
        # byte_tmsb = self._bus.read_byte_data(self._addr,self.DS_REG_TEMP_HSB)
        # byte_tlsb = bin(self._bus.read_byte_data(self._addr,self.DS_REG_TEMP_LSB))[2:].zfill(8)
        return byte_tmsb+int(byte_tlsb[0])*2**(-1)+int(byte_tlsb[1])*2**(-2)

    def read_all(self):
        # Gibt eine Liste zurueck: (year, month, day, dow, hours, minutes, seconds).
        return (self._read_year(), self._read_month(), self._read_day(),
               self._read_dow(), self._read_hours(), self._read_minutes(),
               self._read_seconds())

    def read_str(self, century=20):
        # Gibt einen Datum/Zeit-String im Format 'YYYY-DD-MM HH:MM:SS' zurueck.
        return '%04d-%02d-%02d %02d:%02d:%02d' % (century*100 + self._read_year(),
               self._read_month(), self._read_day(), self._read_hours(),
               self._read_minutes(), self._read_seconds())

    def read_datetime(self, century=20, tzinfo=None):
        # Gibt ein datetime.datetime Objekt zurueck.
        return datetime(century*100 + self._read_year(),
               self._read_month(), self._read_day(), self._read_hours(),
               self._read_minutes(), self._read_seconds(), 0, tzinfo=tzinfo)

    def set_clock(self, century=20):
        # Liest einen Datum/Zeit-String im Format 'MMDDhhmmYYss' aus der RTC 
        # und setzt das Systemdatum mittels date-Kommando.
        cmd = 'sudo date %02d%02d%02d%02d%04d.%02d' % (self._read_month(),
              self._read_day(), self._read_hours(), self._read_minutes(),
              century*100 + self._read_year(), self._read_seconds())
        os.system(cmd)


    def write_all(self, seconds=None, minutes=None, hours=None, dow=None,
                  day=None, month=None, year=None):
        # Setzt Datum und Uhrzeit der RTC, jedoch nur die nicht-None-Werte.
        # Prueft auf Einhaltung der zulaessigen Wertegrenzen:
        #        seconds [0-59], minutes [0-59], hours [0-23],
        #        dow [1-7], day [1-31], month [1-12], year [0-99].
        if seconds is not None:
            if seconds < 0 or seconds > 59:
                raise ValueError('Seconds out of range [0-59].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_SECONDS, _int_to_bcd(seconds))

        if minutes is not None:
            if minutes < 0 or minutes > 59:
                raise ValueError('Minutes out of range [0-59].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_MINUTES, _int_to_bcd(minutes))

        if hours is not None:
            if hours < 0 or hours > 23:
                raise ValueError('Hours out of range [0-23].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_HOURS, _int_to_bcd(hours))

        if year is not None:
            if year < 0 or year > 99:
                raise ValueError('Year out of range [0-99].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_YEAR, _int_to_bcd(year))

        if month is not None:
            if month < 1 or month > 12:
                raise ValueError('Month out of range [1-12].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_MONTH, _int_to_bcd(month))

        if day is not None:
            if day < 1 or day > 31:
                raise ValueError('Day out of range [1-31].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_DAY, _int_to_bcd(day))

        if dow is not None:
            if dow < 1 or dow > 7:
                raise ValueError('DOW out of range [1-7].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_DOW, _int_to_bcd(dow))

    def write_datetime(self, dto):
        # Setzt Datum/Zeit der RTC aus dem Inhalt eines datetime.datetime-Objekts.
        # isoweekday() liefert: Montag = 1, Dienstag = 2, ..., Sonntag = 7;
        # RTC braucht: Sonntag = 1, Montag = 2, ..., Samstag = 7
        wd = dto.isoweekday() + 1 # 1..7 -> 2..8
        if wd == 8:               # Sonntag
            wd = 1
        self.write_all(dto.second, dto.minute, dto.hour, wd,
                       dto.day, dto.month, dto.year % 100)

    def write_now(self):
        # Aequivalent zu write_datetime(datetime.datetime.now()).
        self.write_datetime(datetime.now())


#Konfiguration schreiben, wenn nicht vorhanden, anlegen, sonst gewünschte Daten hinzufügen/Anpassen
def configSchreiben(bereich,wert1, wert2):
    config = configparser.ConfigParser()
    config.read('Config.cfg')
    if config.has_section('Allgemein') != True:
        config['Allgemein'] = {'IP':'127.0.0.1','Port':'8000',
                            'StartZeit':str(datetime.now())}
        
    if bereich=='Allgemein':       
        if config.has_option('Allgemein','StartZeit'):
            config.set('Allgemein','StartZeit',str(datetime.now()))
        else:                    
            config['Allgemein'] = {'StartZeit':str(datetime.now())}
    else:
        if config.has_section(bereich):
            if config.has_option(bereich,wert1):
                config.set(bereich,wert1,wert2)
            else:
                config[bereich][wert1] = wert2
        else:
            config.add_section(bereich)
            if config.has_option(bereich,wert1):
                config.set(bereich,wert1,wert2)
            else:
                config[bereich][wert1] = wert2                    
    with open('Config.cfg','w') as configfile:
        config.write(configfile)
        configfile.close

def _check_OW() -> bool:
    """
    Prüft den OneWire-Bus-Status mit Timeout
    
    Returns:
        bool: True wenn verfügbar, sonst False
    """
    global status_ow
    i_cnt = 0
    while True:
        if status_ow == 1:
            return True
        else:
            i_cnt += 1
            if i_cnt >= 10000:
                log(f"OW Status: {status_ow}", "ERROR")
                return False
            time.sleep(0.001)
    return False

def set_output_konfig(kanal,adresse):
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig","ERROR")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    #Konfiguration als Ausgangsmodul:
    try:        
        plexer.writeByteData(kanal,adresse,bankAKonfig,outputKonfig)
        plexer.writeByteData(kanal,adresse,bankBKonfig,outputKonfig)   
        log("Adresse: " +str(hex(adresse)) + " - Port A + B als Output gesetzt")
    except Exception as e:
        log(f"Fehler beim Output konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")

def set_pwm_konfig(kanal, adresse):
    if adresse <0x50 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    try:
        log("Adresse: {0} - PWM Konfig gesetzt".format(hex(adresse)))
        
        #Mode1 = sleep  Register 0  Wert = 16
        plexer.writeByteData(kanal,adresse,0x00,0x10)
        #prescale: round((25.000.000/(4096*Freuqnz))-1) Frequenz aus Konfig lesen!
        prescale=round((25000000/(4096*freqStd))-1)
        plexer.writeByteData(kanal,adresse,0xFE,prescale)
        #mode1 = sleep Register 0  Wert=32
        plexer.writeByteData(kanal,adresse,0x00,0x20)
        #mode2 = Ausgang Register 1  Wert = 4
        plexer.writeByteData(kanal,adresse,0x01,0x04)        
    except Exception as e:
        log(f"Fehler beim PWM konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")
    
def set_input_konfig(kanal,adresse):
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig","ERROR")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    #Konfiguration als Eingangsmodul:
    try:
        plexer.writeByteData(kanal,adresse,bankAKonfig,inputKonfig)
        plexer.writeByteData(kanal,adresse,bankBKonfig,inputKonfig)
        plexer.writeByteData(kanal,adresse,IOCONA,0x44)
        plexer.writeByteData(kanal,adresse,IOCONB,0x44)
        plexer.writeByteData(kanal,adresse,DEFVALA,0x00)
        plexer.writeByteData(kanal,adresse,DEFVALB,0x00)
        plexer.writeByteData(kanal,adresse,INTCONA,0x00)
        plexer.writeByteData(kanal,adresse,INTCONB,0x00)
        plexer.writeByteData(kanal,adresse,GPPUA,0x00)
        plexer.writeByteData(kanal,adresse,GPPUB,0x00)
        plexer.writeByteData(kanal,adresse,IPOLA,0x00)
        plexer.writeByteData(kanal,adresse,IPOLB,0x00)
        plexer.writeByteData(kanal,adresse,GPINTENA,0xFF)
        plexer.writeByteData(kanal,adresse,GPINTENB,0xFF)
        log("Adresse:{0} - Port A + B als Input gesetzt".format(hex(adresse)),"INFO")
    except Exception as e:
        log(f"Fehler beim Input konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")

 


def send_tcp(data: str) -> None:
    """
    Sendet Daten über TCP-Socket
    
    Args:
        data: Zu sendende Nachricht als String
    """
    global cl_socket
    global cl_ip
    try:
        if cl_socket and hasattr(cl_socket, 'send'):
            res = cl_socket.send(data.encode())
        else:
            log(f"TCP-Nicht verbunden: {data}", "ERROR")
            return None            
    except IOError as err:
        log(f"TCP-Fehler: {err} ; {data}", "ERROR")
        return
    
    # Daten prüfen:
    if int(res) == len(data.encode()):
        log(f"TCP-Gesendet: {cl_ip} : {data}")
    else:
        log(f"TCP-Fehler. Gesendet: {len(data)} Empfangen: {res}", "ERROR")
        log(f"TCP-Fehlerdaten: {data}", "ERROR")

def handle_tcp_connection() -> None:
    """
    Verwaltet TCP-Verbindungen und verarbeitet eingehende Befehle
    """
    global cl_socket
    global cl_ip
    con_closed = False
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_socket.bind(("0.0.0.0", miniServerPort))
    tcp_socket.listen(5)
    (cl_socket, (cl_ip, _)) = tcp_socket.accept()
    log(f"Verbunden: {cl_ip}")
    while True:
        gecos_in_data = ""
        data = ""
        arr = ""
        while data[-1:] != "}":
            try:
                #Testen ob noch verbunden? 
                blk = cl_socket.recv(1).decode("utf-8")
                if len(blk) == 0:
                    con_closed = True
                    break 
            except Exception as e:
                log(f"Fehler beim Empfangen: {e}", "ERROR")
            data += blk
        log(f"Empfangen: {cl_ip} : {data}")
        gecos_in_data = data[:-1]
        data = ""
        if len(gecos_in_data) > 0:
            if gecos_in_data[0] == "{":
                gecos_in_data = gecos_in_data.replace("{", "")
                if gecos_in_data == "MOD":
                    modulSuche()
                if gecos_in_data == "OWS":
                    thread_OW_Search()
                elif gecos_in_data == "SAI":
                    interrutpKanal(intKanal0)
                    interrutpKanal(intKanal1)
                    interrutpKanal(intKanal2)
                elif gecos_in_data == "SPWM":
                    pwmAll()
                elif gecos_in_data == "SRGBW":
                    rgbwAll()
                elif gecos_in_data == "SAO":
                    ReadOutAll()
                elif gecos_in_data == "RRTC":
                    read_rtc()
                    #RTC lesen
                elif len(gecos_in_data) >= 7:
                    arr = gecos_in_data.split(";")
                    if arr[0] == "SOM":
                        set_output(arr)
                    elif arr[0] == "PWM":
                        set_pwm(arr)
                    elif arr[0] == "RGBW":
                        set_rgbw(arr)
                    elif arr[0] == "SAM":
                        read_analog(arr)
                    elif arr[0] == "OWV":
                        thread_OW_read(arr)
                    elif arr[0] == "OWC":
                        thread_OW_config(arr)
                    elif arr[0] == "SRTC":
                        #RTC Setzen
                        set_rtc(arr)
                    else:
                        gecos_in_data = gecos_in_data.replace("{", "").replace("}", "")
                        send_tcp(f"{{ERR;{gecos_in_data};Befehl nicht erkannt}}")
                        log(f"Befehl nicht erkannt: {gecos_in_data}", "ERROR")
            else:
                gecos_in_data = gecos_in_data.replace("{", "").replace("}", "")
                send_tcp(f"{{ERR;{gecos_in_data};Befehl nicht erkannt}}")
                log(f"Befehl nicht erkannt: {gecos_in_data}", "ERROR")
        else:
            arr = ""
            #Verbindung unterbrochen, Neue Verbindung akzeptieren:
            if con_closed == True:
                log("Verbindung getrennt!", "ERROR")
                try:
                    tcp_socket.close()
                except Exception as e:
                    log(f"Fehler beim Socket schliessen: {e}", "ERROR")
                thread_tcp_server()
                break
            else:
                send_tcp(f"{{{gecos_in_data};Befehl nicht erkannt}}")
                log(f"Befehl nicht erkannt: {gecos_in_data}", "ERROR")

def OWReadDevice(arr):
    """Liest Daten von OneWire-Geräten"""
    global status_ow
    x = arr[1].split("-")
    befehl = "{OWV;"
    
    if x[0] == "28":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                temp = dsOW.DS18B20OWReadTemp()               
                status = f"{temp};OK"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "-85;Fehler bei Adresse einstellen"
            status_ow = 1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "10":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                temp = dsOW.DS18S20OWReadTemp()
                status = f"{temp};OK"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "-85;Fehler bei Adresse einstellen"
            status_ow = 1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "3a":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                result = dsOW.DS2413GetState()
                status = f"{result};OK"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "-85;Fehler bei Adresse einstellen"
            status_ow = 1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "3b":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                result = dsOW.MAX31850OWReadTemp()
                status = f"{result};OK"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "-85;Fehler bei Adresse einstellen"
            status_ow = 1
        else:
            status = "-85;Fehler OW Bus belegt"            
    else:
        log("OneWire Typ nicht unterstützt", "INFO")
        status = "-85;Typ nicht untersützt"
    
    befehl += f"{arr[1]};{status}}}"
    send_tcp(befehl)
  
def OWConfigDevice(arr):
    """Konfiguriert OneWire-Geräte"""
    global status_ow
    x = arr[1].split("-")
    befehl = "{OWC;"
    status = ""
    
    if x[0] == "28":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                if dsOW.DS18B20OWSetConfig(int(arr[2])) == True:
                    status = f"{arr[2]};OK"
                else:
                    status = "FEHLER"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "Fehlerhafte OW Adresse"
            status_ow = 1
        else:
            status = "Fehler OW Bus belegt"
    elif x[0] == "3a":
        if _check_OW():
            status_ow = 0
            if dsOW.OWSelectAdress(arr[1]) == True:
                if dsOW.DS2413OWSetConfig(int(arr[2])) == True:
                    status = f"{arr[2]};OK"
                else:
                    status = "FEHLER"
            else:      
                log("Fehler beim Adresse einstellen", "INFO")
                status = "Fehlerhafte OW Adresse"
            status_ow = 1
        else:
            status = "Fehler OW Bus belegt"            
    else:
        log("OneWire Typ nicht unterstützt", "INFO")
        status = "Typ nicht untersützt"
    
    befehl += f"{arr[1]};{status}}}"
    send_tcp(befehl)

def OWSearchDevice():
    """Sucht nach OneWire-Geräten auf dem Bus"""
    global status_ow
    if _check_OW():
        status_ow = 0
        dsOW.OWSearchBus()
        status_ow = 1
    else:
        log("OneWire Bus Belegt", "INFO")

def thread_tcp_server():
    threading.Thread(target=handle_tcp_connection, daemon=True).start()

def thread_interrupt(pin):
    threading.Thread(target=interrutpKanal, args=(pin,), daemon=True).start()

def thread_OW_read(arr):
    threading.Thread(target=OWReadDevice, args=(arr,), daemon=True).start()

def thread_OW_config(arr):
    threading.Thread(target=OWConfigDevice, args=(arr,), daemon=True).start()

def thread_OW_Search():
    threading.Thread(target=OWSearchDevice, daemon=True).start()

def read_output(kanal,adresse):
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        sArr="{"
        sArr+="SAO;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        sArr="{"
        sArr+="SAO;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return

    sArr="{"
    sArr+="SAO;{0};{1};".format(kanal,hex(adresse))
    try:
        #Bytes fuer Bank A + B auslesen
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        iOut = [iOutB, iOutA]
        i=int.from_bytes(iOut,"big")
        sArr+="{0};".format(i)
        sStatus="OK"   
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler Output lesen"
        log("Fehler Output lesen: {0}".format(sArr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sStatus=sStatus.replace(";","")
        sArr+="{0}}}".format(sStatus)
        send_tcp(sArr)

def set_output(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
    try:
        #Bytes fuer Bank A + B auslesen
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        tmpArrOut=int(arr[3]).to_bytes(2,"big")
        iOutA=tmpArrOut[1]
        iOutB=tmpArrOut[0]
        plexer.writeByteData(kanal,adresse,bankA,iOutA)
        plexer.writeByteData(kanal,adresse,bankB,iOutB)
        #Prüfen und antworten.
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        sStatus="OK"      
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler Output lesen"
        log("Fehler Output: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sArr="{"
        sArr+=";".join(arr)
        sStatus=sStatus.replace(";","")
        sArr+=";{0}}}".format(sStatus)
        sArr = sArr.replace(";;",";")
        send_tcp(sArr)   
        
def log(message: str, level: str = "INFO") -> None:
    """
    Logging-Funktion für GeCoS-Server
    
    Args:
        message: Nachricht zum Loggen
        level: Log-Level (INFO, WARNING, ERROR, DEBUG)
    """
    if level == "ERROR":
        logger.error(message)
    elif level == "WARNING":
        logger.warning(message)
    elif level == "DEBUG":
        logger.debug(message)
    else:
        logger.info(message)

       
def set_bit(v: int, index: int, x: bool) -> int:
    """Setzt ein einzelnes Bit auf 1/0 (True oder False)"""
    #Bit auf 1/0 setzen (True oder False)
    mask = 1<< index
    v&=~mask
    if x:
        v |= mask
    return v
    
def ReadOutAll():
    """Liest alle Output-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['out'][kanal]:
            try:
                read_output(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von Output Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")
        

def pwmAll():
    """Liest alle PWM-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['pwm'][kanal]:
            try:
                read_pwm(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von PWM Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")


def rgbwAll():
    """Liest alle RGBW-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['rgbw'][kanal]:
            try:
                read_rgbw(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von RGBW Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")    

def interrutpKanal(pin):
    """Verarbeitet Interrupt für einen bestimmten Pin/Kanal"""
    # Kanal nach INT Pin Wählen:
    if pin==intKanal0:
        kanal=0
    elif pin==intKanal1:
        kanal=1
    elif pin==intKanal2:
        kanal=2
    else:
        log("Kanal ungültig","ERROR")
        return
    
    for device in modules['in'][kanal]:
        try:
            read_input(kanal, device, 1)
        except Exception as e:
            log(f"Fehler beim Lesen von Input Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")

def read_rtc():
    try:
        rtctime = ds.read_datetime()
        sArr="{RRTC;"
        sArr+= rtctime.strftime("%d;%m;%Y;%H;%M;%S;")
        sArr+= "{0};".format(ds.read_temp())
        sArr+="OK}"
        send_tcp(sArr)
    except Exception as e:
        sArr="{RRTC;"
        sArr+="Fehler RTC lesen}"
        send_tcp(sArr) 
        log("Error RTC lesen:" + str(e),"ERROR") 


def set_rtc(arr):
    try:
        str_dto= "{0}/{1}/{2} {3}:{4}:{5}".format(arr[2],arr[1],arr[3],arr[4],arr[5],arr[6])
        dto = datetime.strptime(str(str_dto), '%m/%d/%Y %H:%M:%S')
        ds.write_datetime(dto)
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";OK}"
        send_tcp(sArr) 
    except (ValueError, IndexError) as e: 
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Fehler RTC setzen}"
        send_tcp(sArr) 
        log(f"Error RTC setzen: {e}","ERROR") 
        

def read_analog(arr):
    # "SAM";I2C Kanal;Adresse;Channel-Analog;Resolution;Amplifier
    # {SAM;0;0x69;AnalogChannel;Resolution;Amplifier}
    # {SAM;0;0x69;0;3;0}
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    channel=int(arr[3])
    res=int(arr[4])
    amp=int(arr[5])
    if adresse <0x68 or adresse > 0x6B:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
        
    if channel <0 or channel > 3:
        log("Analog Channel ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Channel ungueltig}"
        send_tcp(sArr) 
        return
    if res <0 or res > 3:
        log("Analog Resolution ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Resolution ungueltig}"
        send_tcp(sArr) 
        return
    if amp <0 or amp > 3:
        log("Analog Amplifier ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Amplifier ungueltig}"
        send_tcp(sArr) 
        return
            
    #Config Bits bit5+6 = Channel
    # Bit 4  4Converison Mode = 1
    # Bits 3+2 Resolution
    # Bist 0+1 = Amplifier
    #arr[3] = Resolution  
    #arr[4] = Amplifier
    bconfig=b"0"
    bconfig = channel <<5 | 1 <<4 | res <<2 | amp
    plexer.writeByte(kanal,adresse,bconfig)
    #Warten bis ergebnis:
    #I2C Port Freigeben:
    if res==0:
        time.sleep(0.010)
    elif res==1:
        time.sleep(0.022)
    elif res==2:
        time.sleep(0.080)
    else:
        time.sleep(0.300)
    #Je Nach Auflösung 3 oder 4Byte lesen:
    #res=3 dann 4 sonst 3
    readyBit=0
    if res==3:
        erg=plexer.readBlockData(kanal,adresse,bconfig,4)
        readyBit=bit_from_string(erg[3],8)
    else:
        erg=plexer.readBlockData(kanal,adresse,bconfig,3)
        readyBit=bit_from_string(erg[2],8)

    signBit=0
    if readyBit==0:
        if res==0:
            #12bit
            wert = ((erg[0] & 0b00001111) <<8 | erg[1])
            signBit=bit_from_string(wert,11)
            if signBit:
                wert = set_bit(wert,11,0)
            wert=wert*0.004923
            if signBit:
                wert=wert-2048               

        elif res==1:
            #14bit
            wert = ((erg[0] & 0b00111111) <<8 | erg[1])
            signBit=bit_from_string(wert,13)
            if signBit:
                wert = set_bit(wert,13,0)
            wert=wert*0.00123075
            if signBit:
                wert=wert-2048

        elif res==2:
            #16bit
            wert = (erg[0] <<8 | erg[1])
            signBit=bit_from_string(wert,15)
            if signBit:
                wert = set_bit(wert,15,0)
            wert=wert*0.0003076875
            if signBit:
                wert=wert-2048
        else:
            #18bit
            wert = ((erg[0] & 0b00000011) <<16 | erg[1]<<8 | erg[2])
            signBit=bit_from_string(wert,17)
            if signBit:
                wert = set_bit(wert,17,0)
            wert=wert*0.000076921875
            if signBit:
                wert=wert-2048
        sStatus="OK"
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";{0};{1}{2}".format(round(wert,3),sStatus,"}")
        send_tcp(sArr) 
    else:
        log("Analog: Daten nicht bereit...","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Daten nicht bereit}"
        send_tcp(sArr) 
        return

def read_pwm(kanal, adresse):
    if adresse <0x50 or adresse > 0x57:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SPWM;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SPWM;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
    
    #{PWM;I2C-Kanal;Adresse;Kanal;Wert}
    #befehl="{0};{1};".format(kanal,hex(adresse))
    for i in range(16): #16
        sArr="{"
        sArr+="SPWM;{0};{1};{2};".format(kanal,hex(adresse),i)
        startAdr=int(i*4+6)
        #LowByte
        lByte=plexer.readByteData(kanal,adresse,startAdr+2)
        #HighByte
        hByte=plexer.readByteData(kanal,adresse,startAdr+3)
        tmpByte=0
        tmpByte=(hByte >> 4) & 0b0000001
        wert=0
        wert = wert*256+int(hByte& 0b0001111)
        wert = wert*256+int(lByte)
        if wert==0:
            wert=0
        else:
            wert=wert
        #PWM Status
        if tmpByte==0:
            sArr+= "1;"
        else:
            sArr+= "0;"
        #PWM Wert:
        sArr+= str(wert)+";"
        sArr+="OK}"
        send_tcp(sArr)

def read_rgbw(kanal, adresse):
    if adresse <0x57 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SAP;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SAP;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
        
    #{RGBW;I2C-Kanal;Adresse;RGBWKanal;StatusRGB;StatusW;R;G;B;W}
    #befehl="{0};{1};".format(kanal,hex(adresse))
    i2 = 0
    sArr="{"
    sArr+="SRGBW;{0};{1};{2};".format(kanal,hex(adresse),i2)
    hByteW=0
    hByteR=0
    r=0
    g=0
    b=0
    w=0
    i3=0
    for i in range(16): #16
        startAdr=int(i*4+6)
        #LowByte
        lByte=plexer.readByteData(kanal,adresse,startAdr+2)
        #HighByte
        hByte=plexer.readByteData(kanal,adresse,startAdr+3)
        wert=0
        wert = wert*256+int(hByte& 0b0001111)
        wert = wert*256+int(lByte)
        if i3==0:
            r=wert
            hByteR=hByte
        elif i3==1:
            g=wert
        elif i3==2:
            b=wert
        elif i3==3:
            w=wert
            hByteW=hByte

        # sArr+= str(wert)+";"
        if i2==0:
              if i == i2+3:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==0:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==0:
                    iSR=1
                else:
                    iSR=0
                sArr+="{0};{1};{2};{3};{4};{5};OK}".format(iSR,iSW,r,g,b,w)
                send_tcp(sArr)
                sArr="{"
                sArr+="SRGBW;{0};{1};{2};".format(kanal,hex(adresse),i2)
                r=0
                g=0
                b=0
                w=0
        if i2==1:
            if i == i2+6:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==1:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==1:
                    iSR=1
                else:
                    iSR=0
                sArr+="{0};{1};{2};{3};{4};{5};OK}".format(iSR,iSW,r,g,b,w)
                send_tcp(sArr)
                sArr="{"
                sArr+="SRGBW;{0};{1};{2};".format(kanal,hex(adresse),i2)
                r=0
                g=0
                b=0
                w=0
        if i2==2:
            if i== i2+9:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==1:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==1:
                    iSR=1
                else:
                    iSR=0
                sArr+="{0};{1};{2};{3};{4};{5};OK}".format(iSR,iSW,r,g,b,w)
                send_tcp(sArr)
                sArr="{"
                sArr+="SRGBW;{0};{1};{2};".format(kanal,hex(adresse),i2)
                r=0
                g=0
                b=0
                w=0
        if i==15:
            i2+=1
            #PWM Status W
            iSW=0
            tmpByte=(hByteW >> 4) & 0b0000001
            if tmpByte==1:
                iSW=1
            else:
                iSW=0
            #PWM Status R
            iSR=0
            tmpByte=(hByteR >> 4) & 0b0000001
            if tmpByte==1:
                iSR=1
            else:
                iSR=0
            sArr+="{0};{1};{2};{3};{4};{5};OK}".format(iSR,iSW,r,g,b,w)
            send_tcp(sArr)
            sArr="{"
            sArr+="SRGBW;{0};{1};{2};".format(kanal,hex(adresse),i2)
            r=0
            g=0
            b=0
            w=0

def set_pwm(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x50 or adresse > 0x57:
        log("Modul Adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul Adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[3]) < 0 or int(arr[3]) >15:
        log("PWM-Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Kanal ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("PWM-Status ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Status ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[5]) <0 or int(arr[5]) >4095:
        log("PWM-Wert ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Wert ungueltig}"
        send_tcp(sArr) 
        return
    sStatus=""
    try:
        #LED_ON Immer 0
        #LED_OFF 4096*X%-1
        #Array durchlaufen 0-15 (+1) = ausgang; ausgang*4+6 = Start Adresse LED_ON_L 
        #Array 3= Kanal 4 = wert
        i=int(arr[3])
        wert = int(arr[5]) #int(round(4095*(int(arr[5])/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        sStatus="OK"
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler PWM Setzen lesen"
        log("Fehler PWM Setzen: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sArr="{"
        sArr+=";".join(arr)
        sStatus=sStatus.replace(";","")
        sArr+=";{0}}}".format(sStatus)
        sArr = sArr.replace(";;",";")
        send_tcp(sArr)

def set_rgbw(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x58 or adresse > 0x5f:
        log("Modul Adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul Adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[3]) <0 or int(arr[3]) >3:
        log("PWMKanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Kanal ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("StatusRGB ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";StatusRGB ungueltig}"
        send_tcp(sArr) 
        return
    if int(arr[5]) <0 or int(arr[5]) >1:
        log("StatusW ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";StatusW ungueltig}"
        send_tcp(sArr) 
        return
    sStatus=""
    try:
        #LED_ON Immer 0
        #LED_OFF 4096*X%-1
        #Array durchlaufen 0-15 (+1) = ausgang; ausgang*4+6 = Start Adresse LED_ON_L 
        #Array 3= Kanal 4 = wert
        i=int(arr[3])
        if i==1:
            i+=3
        elif i==2:
            i+=6
        elif i==3:
            i+=9
        r=int(arr[6])
        g=int(arr[7])
        b=int(arr[8])
        w=int(arr[9])
        #Rot:
        wert = r #int(round(4095*(r/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Grün:
        wert = g #int(round(4095*(g/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Blau:
        wert = b #int(round(4095*(b/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Weiß:
        wert = w #int(round(4095*(w/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[5])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        sStatus="OK"
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(str(err)),"ERROR")
    except:
        sStatus="Fehler PWM Setzen lesen"
        log("Fehler PWM Setzen: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sArr="{"
        sArr+=";".join(arr)
        sStatus=sStatus.replace(";","")
        sArr+=";{0}}}".format(sStatus)
        sArr = sArr.replace(";;",";")
        send_tcp(sArr)   
        
def read_input(kanal,adresse, manual=0):
    """Liest Eingänge von einem Modul und sendet bei Änderung"""
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SAI;{0};{1};".format(kanal,hex(adresse))
        sArr+="Modul adresse ungueltig}"
        send_tcp(sArr) 
        return
    
    if kanal <0 or kanal > 2:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SAI;{0};{1};".format(kanal,hex(adresse))
        sArr+="Kanal ungueltig}"
        send_tcp(sArr) 
        return

    # Programm in Schleife -> Auf Änderung prüfen -> bei Änderung senden + Neuen Status in Variable schreiben
    # Adresse 0x20-0x23 -> Index 0&1 / 2&3 / 4&5 / 6&7
    # Erster Value - Bank A, Zweiter = Bank B
    wertAltA=0
    wertAltB=0
    idx = (adresse - 0x20) * 2  # Berechne Index basierend auf Adresse
    wertAltA = stat_in[kanal][idx]
    wertAltB = stat_in[kanal][idx + 1]
    
    try:
        # GPIO A+B Lesen und String bauen:
        wertA=plexer.readByteData(kanal,adresse,gpioA)
        wertB=plexer.readByteData(kanal,adresse,gpioB)
        if wertAltA!=wertA or wertAltB!=wertB or manual==1:
            # Unterschied, Senden!
            befehl="{SAI;"
            befehl+="{0};{1};".format(kanal,hex(adresse))
            iIn = [wertB, wertA]
            i=int.from_bytes(iIn,"big")
            befehl+="{0};".format(i)
            befehl+="OK}"
            send_tcp(befehl)

        # Erneut lesen, auf Änderung prüfen:
        wertA2=plexer.readByteData(kanal,adresse,gpioA)
        wertB2=plexer.readByteData(kanal,adresse,gpioB)
        if wertA2!=wertA or wertB2!=wertB:
            befehl="{SAI;"
            befehl+="{0};{1};".format(kanal,hex(adresse))
            iIn = [wertB2, wertA2]
            i=int.from_bytes(iIn,"big")
            befehl+="{0};".format(i)
            befehl+="OK}"
            send_tcp(befehl)
            wertA=wertA2
            wertB=wertB2        
    except OSError as err:
        befehl="{SAI;"
        befehl+="{0};{1};".format(kanal,hex(adresse))
        befehl+="0;IO Error SAI}"
        send_tcp(befehl)
        log("I/O error: {0}".format(str(err)),"ERROR")
    except Exception as e:
        befehl="{SAI;"
        befehl+="{0};{1};".format(kanal,hex(adresse))
        befehl+="0;Fehler SAI}"
        send_tcp(befehl)
        log("Fehler Input lesen: {0}".format(befehl),"ERROR")
    finally:
        # Status speichern
        stat_in[kanal][idx] = wertA
        stat_in[kanal][idx + 1] = wertB
        
        
        

def modulSuche(delete=0):
    """Sucht nach I2C-Modulen auf allen Kanälen"""
    # Daten löschen:
    if delete==1:
        for kanal in range(3):
            modules['in'][kanal].clear()
            modules['out'][kanal].clear()
            modules['pwm'][kanal].clear()
            modules['rgbw'][kanal].clear()
            modules['ana'][kanal].clear()

    for kanalSearch in range(3):        
        log("Suche Bus: {0} Kanal: {1}".format(bus,kanalSearch))
        tmpIN=""
        tmpOut=""
        tmpRGBW=""
        tmpPWM=""
        tmpUnb=""
        tmpANA=""
        for device in range(128):
            try:
                if (plexer.readByte(kanalSearch,device)!= None):
                    if device!=mux and device!=DS2482.I2C_ADDR:
                        if device>=0x20 and device <=0x23:
                            log("GeCoS 16 In : Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpIN=tmpIN+hex(device)+";"
                            if device not in modules['in'][kanalSearch]:
                                modules['in'][kanalSearch].append(device)
                                set_input_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("IN")
                            befehl+="}"
                            send_tcp(befehl)
                        elif device>=0x24 and device <=0x27:
                            log("GeCoS 16 OUT: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpOut=tmpOut+hex(device)+";"
                            if device not in modules['out'][kanalSearch]:
                                modules['out'][kanalSearch].append(device)
                                set_output_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("OUT")
                            befehl+="}"
                            send_tcp(befehl)
                        elif device>=0x50 and device <=0x57:
                            log("GeCoS 16 PWM: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpPWM=tmpPWM+hex(device)+";"
                            if device not in modules['pwm'][kanalSearch]:
                                modules['pwm'][kanalSearch].append(device)
                                set_pwm_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("PWM")
                            befehl+="}"
                            send_tcp(befehl)
                        elif device>=0x58 and device <=0x5f:
                            log("GeCoS 16 RGBW: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpRGBW=tmpRGBW+hex(device)+";"
                            if device not in modules['rgbw'][kanalSearch]:
                                modules['rgbw'][kanalSearch].append(device)
                                set_pwm_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("RGBW")
                            befehl+="}"
                            send_tcp(befehl)
                        elif device>=0x68 and device <=0x6b:
                            log("GeCoS Analog4: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpANA=tmpANA+hex(device)+";"
                            if device not in modules['ana'][kanalSearch]:
                                modules['ana'][kanalSearch].append(device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("ANA")
                            befehl+="}"
                            send_tcp(befehl)
                        else:
                            tmpUnb=tmpUnb+hex(device)+";"
                            log("GeCoS Unbekanntes Gerät: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("UNB")
                            befehl+="}"
                            send_tcp(befehl)
            except Exception as e:
                log(f"Fehler beim Scannen von Device {device} auf Kanal {kanalSearch}: {e}", "DEBUG")
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16IN',tmpIN)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16OUT',tmpOut)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'UNBEKANNT',tmpUnb)                
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16PWM',tmpPWM)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOSANA4',tmpANA)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16RGBW',tmpRGBW)
    befehl="{MOD;0;0;END}"
    send_tcp(befehl)
        
def bit_from_string(string, index):
    i=int(string)
    return i >> index & 1

if __name__ == '__main__':
    # Konfig Werte MCP:
    log("Script gestartet", "ERROR")
    
    # ArgParser:
    parser = argparse.ArgumentParser(description='GeCoS-Server - Gebäudecontrol System')
    parser.add_argument('--debug', '-d', help='Aktiviert Debug-Ausgaben', action='store_true')
    args = parser.parse_args()
    
    if args.debug:
        print_debug = True
        logger.setLevel(logging.DEBUG)

    bus = 1  # 0 für rev1 boards etc.
    mux=0x71
    kanal=0
    bankAKonfig=0x00
    bankBKonfig=0x01
    outputKonfig=0x00
    inputKonfig=0xFF
    IOCONA=0x0A
    IOCONB=0x0B
    DEFVALA=0x06
    DEFVALB=0x07
    INTCONA=0x08
    INTCONB=0x09
    GPPUA=0x0C
    GPPUB=0x0D
    IPOLA=0x02
    IPOLB=0x03
    GPINTENA=0x04
    GPINTENB=0x05    
    intBankA=0x0E
    intBankB=0x0F
    intcapA=0x10
    intcapB=0x11
    gpioA=0x12
    gpioB=0x13
    bankA=0x14
    bankB=0x15
    aOutHex = [0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80]
    #Konfig:    
    miniServerIP="192.168.178.28"
    miniServerPort=8000
    #paketLaenge=1024
    freqStd=100
    
    #Interrupt Ports:
    intKanal0=17
    intKanal1=18
    intKanal2=27
    
    #Config lesen:
    configSchreiben('Allgemein','x','x')
   
    #MUX initialisieren:
    log("Bus:" + str(bus) + " Kanal:" + str(kanal))
    plexer = multiplex(bus)

    log(datetime.now())
    
    #Modulsuche:
    modulSuche(1)
    
    #OneWire:
    dsOW = DS2482()

    thread_tcp_server()
    #RTC Lesen:
    ds = DS1307(plexer, 0x68)
    rtctime = ds.read_datetime()
    temp = ds.read_temp()
    log ("DS3231 Date: {0} Temp: {1} ".format(rtctime.strftime("%d.%m.%Y %H:%M:%S"),str(temp)))
    while True:
        try:
            # Alle Eingänge lesen
            time.sleep(0.01)
            # Schleife für Eingang Lesen:
            for kanal in range(3):
                for device in modules['in'][kanal]:
                    try:
                        read_input(kanal, device)
                    except Exception as e:
                        log(f"Fehler beim Lesen von Input Kanal {kanal}, Device {hex(device)}: {e}", "DEBUG")
        except KeyboardInterrupt:
            log("Server wird beendet...", "INFO")
            break


