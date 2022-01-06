#!/usr/bin/python
# encoding=utf-8
import smbus
import time
import sys
import datetime
from datetime import datetime
import socket
import _thread
import configparser
import os
import argparse
import numpy as np

#import RPi.GPIO as GPIO

#Status Variable 16IN 1x pro Bus mit 8 Werten
statIN0 = [0,0,0,0,0,0,0,0]
statIN1 = [0,0,0,0,0,0,0,0]
statIN2 = [0,0,0,0,0,0,0,0]

#Globale Variablen
printDebug=False
statusOW=1
clSocket = ""
clIP = ""
aIN0 = []
aIN1 = []
aIN2 = []
aOut0= []
aOut1= []
aOut2= []
aPWM0= []
aPWM1= []
aPWM2= []
aRGBW0= []
aRGBW1= []
aRGBW2= []
aANA0= []
aANA1= []
aANA2= []
dmxStop=True
I2CAdrDS2482= 0x18 #Adresse DS2482

#MUX:
class multiplex:
    statusI2C = 1
    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)

    def _check_i2c(self):
        iCnt=0
        while True:
            if self.statusI2C==1:
                return True
            else:
                iCnt+=1
                if iCnt >= 150:
                    log("I2C Status: {0}".format(str(self.statusI2C)),"ERROR")
                if iCnt>= 500:
                    log("I2C Status Abbruch: {0}".format(str(self.statusI2C)),"ERROR")
                    self.statusI2C=1
                    return False
                time.sleep(0.001)
        return False

    def channel(self, address=0x71,channel=0):  # values 0-3 indictae the channel, anything else (eg -1) turns off all channels
        
        if   (channel==0): action = 0x04
        elif (channel==1): action = 0x05
        elif (channel==2): action = 0x06
        elif (channel==3): action = 0x07
        else : action = 0x00
        self.bus.write_byte_data(address,0x04,action)  #0x04 is the register for switching channels

    def writeByteData(self, kanal, address, register, wert):
        if (self._check_i2c()==True):
            self.statusI2C=0
            self.channel(mux,kanal)
            time.sleep(0.001)
            try:
                self.bus.write_byte_data(address,register,wert)
            except:
                self.statusI2C=1
                return False
            self.statusI2C=1
        else:
            return False
        return True

    #read_byte_data(addr)
    def readByte(self, kanal, address):
        if (self._check_i2c()==True):
            self.statusI2C=0
            self.channel(mux,kanal)
            try:
                wert=self.bus.read_byte(address)
            except:
                self.statusI2C=1
                return None
            self.statusI2C=1
        else:
            return None
        return wert

    #read_byte_data(addr, register)
    def readByteData(self, kanal, address, register):
        if (self._check_i2c()==True):
            self.statusI2C=0
            try:
                self.channel(mux,kanal)
                wert=self.bus.read_byte_data(address,register)
            except:
                #zweiter versuch um selten auftretende "Remote I/O Error abzufangen"
                self.channel(mux,kanal)
                wert=self.bus.read_byte_data(address,register)
            self.statusI2C=1
        else:
            return None
        return wert

    #erg=plexer.bus.read_i2c_block_data(adresse,bconfig,4)
    def readBlockData(self, kanal, address,register,cnt):
        if (self._check_i2c()==True):
            self.statusI2C=0
            self.channel(mux,kanal)
            wert=self.bus.read_i2c_block_data(address,register,cnt)
            self.statusI2C=1
        else:
            wert=[0x00,0x00,0x00,0x00]
        return wert

    #plexer.bus.write_byte(adresse,bconfig)
    def writeByte(self,kanal,address,register):
        if (self._check_i2c()==True):
            self.statusI2C=0
            self.channel(mux,kanal)
            self.bus.write_byte(address,register)
            self.statusI2C=1
        else:
            return False
        return True


#DS2482:
class DS2482:
    #global owDeviceAddress = [[0,0], [0,0]]#; //These are each 32 bits long.

    def __init__(self):
        self._bus = smbus.SMBus(bus)
        self._owDeviceAddress = [0,0]
        self._owTripletDirection = 1
        self._owTripletFirstBit = 0
        self._owTripletSecondBit = 0
        self._owLastDevice = 0
        self._owLastDiscrepancy = 0

    def OWSearchBus(self):
        global I2CAdrDS2482
        try:
            while self.OWSearch()==1:
            #if (self.OWSearch()):
                device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
                befehl="{OWS;"
                befehl +="{0}".format(device)
                befehl+="}"
                sendUDP(befehl)
                log("Gerät gefunden: " + str(device), "INFO")
            befehl="{OWS;END}"
            sendUDP(befehl)
        except:
            befehl="{OWS;Fehler}"
            sendUDP(befehl)
            log("Fehler bei OWS Suche", "ERROR")


    def DS2482Reset(self):
        global I2CAdrDS2482
        self._bus.write_byte(I2CAdrDS2482, 0xF0) #reset DS2482


    def OWStatusRegister(self):
        global I2CAdrDS2482
        self._bus.write_byte_data(I2CAdrDS2482,0xE1,0xF0)
        e=self._bus.read_byte(I2CAdrDS2482)
        return e


    def OWReset(self):
        global I2CAdrDS2482
        self._bus.write_byte(I2CAdrDS2482,0xB4) #1Wire Reset
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
        global I2CAdrDS2482    
        self._bus.write_byte_data(I2CAdrDS2482,0xE1,0xF0)
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
        self._bus.write_byte_data(I2CAdrDS2482,0xA5, byte)
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
        global I2CAdrDS2482
        self._bus.write_byte_data(I2CAdrDS2482,0xE1,0xF0)
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
        self._bus.write_byte(I2CAdrDS2482,0x96)
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
        self._bus.write_byte_data(I2CAdrDS2482,0xE1,0xE1)
        data=self._bus.read_byte(I2CAdrDS2482)
        if(data is None):
                #Fehler
                return -1
        return data
                



    
    def OWTriplet(self):
        global I2CAdrDS2482
        if (self._owTripletDirection > 0):
            self._owTripletDirection = 0xFF
        self._bus.write_byte_data(I2CAdrDS2482, 0x78,self._owTripletDirection)
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
        global I2CAdrDS2482
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
        global I2CAdrDS2482
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
        except:
            log("Fehler beim OneWire Adresse einstellen")
            return False

    def DS2413OWSetConfig(self,data):
        try:
            self.OWSelect()
            self.OWWriteByte(0x5a)
            self.OWWriteByte(data)
            data = ~data&0xFF
            self.OWWriteByte(data)
            return True
        except:
            return False
        return True

    def DS18B20OWSetConfig(self,res):
        try:
            # 31, 63, 95, 127 9/10/11/12Bit
            self.OWSelect()
            self.OWWriteByte(78)
            self.OWWriteByte(0)
            self.OWWriteByte(0)
            self.OWWriteByte(res)
            return True
        except:
            return False
        return True
        

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
        except:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log("Fehler 1Wire: {0}".format(str(device)),"ERROR")
        finally:
            return celsius

    def DS2413GetState(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x3a): #Ist ein DS18B20
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0xF5) #Starte Messung
                    result = self.OWReadByte()
        except:
            result=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log("Fehler 1Wire: {0}".format(str(device)),"ERROR")
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
        except:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log("Fehler 1Wire: {0}".format(str(device)),"ERROR")
        finally:
            return celsius



    def DS18S20OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x10): #Ist ein DS18B20
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
        except:
            celsius=-85
            device=hex(self._owDeviceAddress[1]& 0xFF)[2:4] + "-" + hex(self._owDeviceAddress[0]<<32 | (self._owDeviceAddress[1]))[2:16]
            log("Fehler 1Wire: {0}".format(str(device)),"ERROR")
        finally:
            return celsius


class PyDMX:
    def __init__(self,COM='/dev/ttyAMA0',Brate=250000,Bsize=8,StopB=2):
        import serial
        #start serial
        self.ser = serial.Serial(COM,baudrate=Brate,bytesize=Bsize,stopbits=StopB)
        self.data = np.zeros([513],dtype='uint8')
#        self.data =[chr(0)]*513
        self.data[0] = 0 # StartCode
        self.sleepms = 250.0
        self.breakus = 176.0
        self.MABus = 16.0
        

    def set_data(self,id,data):
        self.data[id]=data
    
    def get_data(self,id):
        return self.data[id]

    def send(self):
        # Send Break : 88us - 1s
        self.ser.break_condition = True
        time.sleep(self.breakus/1000000.0)
        
        # Send MAB : 8us - 1s
        self.ser.break_condition = False
        time.sleep(self.MABus/1000000.0)
        
        # Send Data
       # print(bytearray(self.data))
        self.ser.write(bytearray(self.data))
#        sdata="".join(str(v) for v in self.data)
#        self.ser.write(sdata.encode('ASCII'))
#        print(sdata)
        # Sleep
        time.sleep(self.sleepms/1000.0) # between 0 - 1 sec

    def sendzero(self):
        self.data = [chr(0)]*513
        self.send()

    def __del__(self):
        self.sendzero()
        self.ser.close()


dmx=PyDMX('/dev/ttyAMA0')


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

def _check_OW():
    global statusOW
    iCnt=0
    while True:
        if statusOW==1:
            return True
        else:
            iCnt+=1
            if iCnt>= 10000:
                log("OW Status: {0}".format(str(statusOW)),"ERROR")
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
    except:
        log("Fehler beim Output konfigurieren","ERROR")

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
    except:
        log("Fehler beim PWM konfigurieren","ERROR")
    
def set_input_konfig(kanal,adresse):
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig","ERROR")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    #Konfiguration als Ausgangsmodul:
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
    except:
        log("Fehler beim Input konfigurieren","ERROR")

def dmxThread(dmx):
    #DMX Thread, senden min alle 1s, sonst fehler auf DMX Bus.
    global dmxStop
    while True:
        dmx.send()
        time.sleep(0.5)
        if (dmxStop==True):
            break

def thread_DMXStart():
    global dmx
    try:
        dmx
    except NameError:
        dmx=PyDMX('/dev/ttyS0')
    _thread.start_new_thread(dmxThread,(dmx,))


def dmxBefehl(arr):
    global dmxStop
    dmxBefehl=arr[1]
    befehl="{{DMX;{0}".format(dmxBefehl)
    if (dmxBefehl=="Stop"):
        dmxStop=True
        status="OK"
    elif (dmxBefehl=="Start"):
        dmxStop=False
        thread_DMXStart()
        status="OK"
    elif (dmxBefehl=="Status"):
        #Status senden: dmxStop
        if (dmxStop == False):
            status="Aktiv" 
        else:
            status ="Inaktiv"
    else:
        status ="ERROR"    

    befehl +=";{0}}}".format(status)
    sendUDP(befehl)

def dmxSetKanal(arr):
    global dmx
    #Array= {Befehl,Kanal,Laenge, Werte()}
    global dmxStop
    dmxKanal=int(arr[1])
    dmxLaenge=int(arr[2])
    befehl="{{DMXSR;{0};{1};{2}".format(arr[0],arr[1],arr[2])
    if dmxKanal <1 or dmxKanal+dmxLaenge > 513:
        log("dmx Kanal ungueltig: {0}".format(dmxKanal))
        befehl="{"
        befehl+="DMXSR;{0};{1};{2};".format(arr[0],dmxKanal,dmxLaenge)
        for kanal in range(dmxLaenge):
            befehl+=arr[3+kanal]+";"
        befehl+="DMX Kanal ungueltig}"
        sendUDP(befehl) 
        return
    if (dmxStop == True):
        log("dmx inaktiv: {0}".format(dmxKanal))
        befehl="{"
        befehl+="DMXSR;{0};{1};{2};".format(arr[0],dmxKanal,dmxLaenge)
        for kanal in range(dmxLaenge):
            befehl+=arr[3+kanal]+";"
        befehl+="DMX inaktiv}"
        sendUDP(befehl) 
        return

    for kanal in range(dmxLaenge):
        dmx.set_data(dmxKanal+kanal,int(arr[3+kanal]))
        status ="OK"    

    befehl="{"
    befehl+="DMXSR;{0};{1};".format(dmxKanal,dmxLaenge)
    for kanal in range(dmxLaenge):
        befehl+=arr[3+kanal]+";"
    befehl+=status
    befehl+="}"
    sendUDP(befehl)

def dmxGetKanal(arr):
    #Array= {Befehl,Kanal,Laenge, Werte()}
    dmxKanal=int(arr[1])
    dmxLaenge=int(arr[2])
    befehl="{{{0};{1};{2}".format(arr[0],arr[1],arr[2])
    if dmxKanal <1 or dmxKanal+dmxLaenge > 513:
        log("dmx Kanal ungueltig: {0}".format(dmxKanal))
        befehl="{"
        befehl+="DMXIR;{0};{1};".format(dmxKanal,dmxLaenge)
        for kanal in range(dmxLaenge):
            befehl+=arr[3+kanal]+";"
        befehl+="DMX Kanal ungueltig}"
        sendUDP(befehl) 
        return
    log(befehl)
    for kanal in range(dmxLaenge):
        befehl+=";"+str(dmx.get_data(dmxKanal+kanal))
        status="OK"
    befehl+=";"+status
    befehl+="}"
    sendUDP(befehl) 


def sendUDP(data):
    #Daten Senden
    global clSocket
    global clIP
    try:
        if hasattr(clSocket, 'send'):
            res=clSocket.send(data.encode())
        else:
            log("UDP-Nicht verbunden: {0}".format(data),"ERROR")
            return None            
    except IOError as err:
        log("UDP-Fehler:{0} ; {1}".format(str(err),data),"ERROR")
    #Daten pruefen:
    if (int(res)==int(data.encode().__len__())):
        log("UDP-Gesendet: {0} : {1}".format(clIP,data))
    else:
        log("UDP-Fehler. Gesendet: {0} Empfangen: {1}".format(data.__len__(),res),"ERROR")
        log("UDP-Fehlerdaten: {0}".format(data),"ERROR")

def getUDP():
    global clSocket
    global clIP
    conClosed=False
    tcpSocket=socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Internet, UDP
    tcpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    tcpSocket.bind(("0.0.0.0",miniServerPort))
    tcpSocket.listen(5)
    (clSocket, clIP) = tcpSocket.accept()
    log("Verbunden: {0}".format(clIP))
    while True:
        #log("Get UDP")
        GeCoSInData=""
        data=""
        arr=""
        while data[-1:]!="}":
            try:
                #Testen ob noch verbunden? 
                blk=clSocket.recv(1).decode("utf-8")
                if len(blk)==0:
                    conClosed=True
                    break 
            except:
                log("Fehler beim Empfangen","ERROR")
            data+=blk
        log("Empfangen: {0} : {1}".format(clIP,data))
        GeCoSInData=data[:-1]
        data = ""
        if len(GeCoSInData)>0:
            if GeCoSInData[0]=="{":
                GeCoSInData=GeCoSInData.replace("{","")
                if GeCoSInData=="MOD":
                    modulSuche()
                if GeCoSInData=="OWS":
                    thread_OW_Search()
                elif GeCoSInData=="SAI":
                    interrutpKanal(intKanal0)
                    interrutpKanal(intKanal1)
                    interrutpKanal(intKanal2)
                elif GeCoSInData=="SPWM":
                    pwmAll()
                elif GeCoSInData=="SRGBW":
                    rgbwAll()
                elif GeCoSInData=="SAO":
                    ReadOutAll()
                elif GeCoSInData=="RRTC":
                    read_rtc()
                    #RTC lesen
                elif len(GeCoSInData)>=7: #13
                    arr=GeCoSInData.split(";")
                    if arr[0]=="SOM":
                        set_output(arr)
                    elif arr[0]=="PWM":
                        set_pwm(arr)
                    elif arr[0]=="RGBW":
                        set_rgbw(arr)
                    elif arr[0]=="SAM":
                        read_analog(arr)
                    elif arr[0]=="OWV":
                        thread_OW_read(arr)
                    elif arr[0]=="OWC":
                        thread_OW_config(arr)
                    elif arr[0]=="SRTC":
                        #RTC Setzen
                        set_rtc(arr)
                    elif arr[0]=="DMX":
                        #DMX Befehl:
                        dmxBefehl(arr)
                    elif arr[0]=="DMXSR":
                        #DMX Kanal Range setzen:
                        dmxSetKanal(arr)
                    elif arr[0]=="DMXIR":
                        #DMX Kanal Range:
                        dmxGetKanal(arr)
                    else:
                        GeCoSInData.replace("{","")
                        GeCoSInData.replace("}","")
                        sendUDP("{0}ERR;{1}Befehl nicht erkannt{2}".format("{",GeCoSInData,"}"))
                        log("Befehl nicht erkannt: {0}".format(GeCoSInData),"ERROR")
            else:
                GeCoSInData.replace("{","")
                GeCoSInData.replace("}","")
                sendUDP("{0}ERR;{1}Befehl nicht erkannt{2}".format("{",GeCoSInData,"}"))
                log("Befehl nicht erkannt: {0}".format(GeCoSInData),"ERROR")
        else:
            arr=""
            #Verbindung unterbrochen, Neue Verbindung akzeptieren:
            if conClosed==True:
                log("Verbindung getrennt!","ERROR")
                try:
                    tcpSocket.close()
                except:
                    log("Fehler beim Socket schliessen","ERROR")
                thread_gecosOut()
                break
            else:
                sendUDP("{0}{1}Befehl nicht erkannt{2}".format("{",GeCoSInData,"}"))
                log("Befehl nicht erkannt: {0}".format(GeCoSInData),"ERROR")

def OWReadDevice(arr):
    global statusOW
    x = arr[1].split("-")
    befehl="{OWV;"
    if x[0] == "28":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                temp=dsOW.DS18B20OWReadTemp()
                status ="{0};{1}".format(str(temp),"OK")
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status = "-85;Fehler bei Adresse einstellen"
            statusOW=1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "10":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                temp=dsOW.DS18S20OWReadTemp()
                status ="{0};{1}".format(str(temp),"OK")
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status = "-85;Fehler bei Adresse einstellen"
            statusOW=1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "3a":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                result=dsOW.DS2413GetState()
                status ="{0};{1}".format(str(result),"OK")
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status = "-85;Fehler bei Adresse einstellen"
            statusOW=1
        else:
            status = "-85;Fehler OW Bus belegt"
    elif x[0] == "3b":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                result=dsOW.MAX31850OWReadTemp()
                status ="{0};{1}".format(str(result),"OK")
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status = "-85;Fehler bei Adresse einstellen"
            statusOW=1
        else:
            status = "-85;Fehler OW Bus belegt"            
    else:
        log("OneWire Typ nicht unterstützt","INFO")
        status="-85;Typ nicht untersützt}"
    
    befehl +="{0};{1}}}".format(arr[1],status)
    sendUDP(befehl)
  
def OWConfigDevice(arr):
    global statusOW
    x = arr[1].split("-")
    befehl="{OWC;"
    status=""
    if x[0] == "28":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                if dsOW.DS18B20OWSetConfig(int(arr[2])) == True:
                    status="{0};{1}".format(str(arr[2]),"OK")
                else:
                    status="FEHLER}"
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status="Fehlerhafte OW Adresse}"
            statusOW=1
        else:
            status="Fehler OW Bus belegt"
    elif x[0] == "3a":
        if _check_OW():
            statusOW=0
            if dsOW.OWSelectAdress(arr[1])==True:
                if dsOW.DS2413OWSetConfig(int(arr[2])) == True:
                    status="{0};{1}".format(str(arr[2]),"OK")
                else:
                    status="FEHLER}"
            else:      
                log("Fehler beim Adresse einstellen","INFO")
                status="Fehlerhafte OW Adresse}"
            statusOW=1
        else:
            status="Fehler OW Bus belegt"            
    else:
        log("OneWire Typ nicht unterstützt","INFO")
        status="Typ nicht untersützt}"
    befehl +="{0};{1}}}".format(arr[1],status)
    sendUDP(befehl)

def OWSearchDevice():
    global statusOW
    if _check_OW():
        statusOW=0
        dsOW.OWSearchBus()
        statusOW=1
    else:
        log("OneWire Bus Belegt","INFO")

def thread_gecosOut():
    _thread.start_new_thread(getUDP,())

def thread_interrupt(pin):
    _thread.start_new_thread(interrutpKanal,(pin,))

def thread_OW_read(arr):
    _thread.start_new_thread(OWReadDevice,(arr,))

def thread_OW_config(arr):
    _thread.start_new_thread(OWConfigDevice,(arr,))

def thread_OW_Search():
    _thread.start_new_thread(OWSearchDevice,())

def read_output(kanal,adresse):
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        sArr="{"
        sArr+="SAO;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        sendUDP(sArr) 
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        sArr="{"
        sArr+="SAO;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr)

def set_output(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul adresse ungueltig}"
        sendUDP(sArr) 
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr)   
        
def log(message, level="INFO"):
    timestamp= datetime.now().strftime("%H:%M:%S.%f") #time.strftime("%d.%m.%Y %H:%M:%S.%f", time.localtime(time.time()))
    if printDebug==True:
        print("{0} {1}: {2}".format(timestamp, level, message))
    if level=="ERROR":
        file = open("/home/pi/logfile.log","a")
        file.write("%s: %s\n" % (time.strftime("%d.%m.%Y %H:%M:%S"), message))
        file.close

       
def set_bit(v, index, x): #v=original wert, x= true oder false
    #Bit auf 1/0 setzen (True oder False)
    mask = 1<< index
    v&=~mask
    if x:
        v |= mask
    return v
    
def ReadOutAll():
    global aOut0
    for kanal in range(3):
        if kanal==0:
            for device in aOut0:
                try:
                    read_output(kanal,device)
                except:
                    pass
        if kanal==1:
            for device in aOut1:
                try:
                    read_output(kanal,device)
                except:
                    pass
        if kanal==2:
            for device in aOut2:
                try:
                    read_output(kanal,device)
                except:
                    pass
        

def pwmAll():
    global aPWM0,aPWM1,aPWM2
    for kanal in range(3):
        if kanal==0:
            for device in aPWM0:
                try:
                    read_pwm(kanal,device)
                except:
                    pass
        if kanal==1:
            for device in aPWM1:
                try:
                    read_pwm(kanal,device)
                except:
                    pass
        if kanal==2:
            for device in aPWM2:
                try:
                    read_pwm(kanal,device)
                except:
                    pass


def rgbwAll():
    global aRGBW0,aRGBW1,aRGBW2
    for kanal in range(3):
        if kanal==0:
            for device in aRGBW0:
                try:
                    read_rgbw(kanal,device)
                except:
                    pass
        if kanal==1:
            for device in aRGBW1:
                try:
                    read_rgbw(kanal,device)
                except:
                    pass
        if kanal==2:
            for device in aRGBW2:
                try:
                    read_rgbw(kanal,device)
                except:
                    pass    

def interrutpKanal(pin):
    #Kanal nach INT Pin Wählen:
    if pin==intKanal0:
        kanal=0
        for device in aIN0:
            try:
                read_input(kanal,device,1)
            except:
                pass        
    elif pin==intKanal1:
        kanal=1
        for device in aIN1:
            try:
                read_input(kanal,device,1)
            except:
                pass
    elif pin==intKanal2:
        kanal=2
        for device in aIN2:
            try:
                read_input(kanal,device,1)
            except:
                pass
    else:
        log("Kanal ungültig","ERROR")
        kanal=0

def read_rtc():
    try:
        rtctime = ds.read_datetime()
        sArr="{RRTC;"
        sArr+= rtctime.strftime("%d;%m;%Y;%H;%M;%S;")
        sArr+= "{0};".format(ds.read_temp())
        sArr+="OK}"
        sendUDP(sArr)
    except Exception as e:
        sArr="{RRTC;"
        sArr+="Fehler RTC lesen}"
        sendUDP(sArr) 
        log("Error RTC lesen:" + str(e),"ERROR") 


def set_rtc(arr):
    try:
        str_dto= "{0}/{1}/{2} {3}:{4}:{5}".format(arr[2],arr[1],arr[3],arr[4],arr[5],arr[6])
        dto = datetime.strptime(str(str_dto), '%m/%d/%Y %H:%M:%S')
        ds.write_datetime(dto)
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";OK}"
        sendUDP(sArr) 
    except: 
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Fehler RTC setzen}"
        sendUDP(sArr) 
        log("Error RTC setzen","ERROR") 
        

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
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
        return
        
    if channel <0 or channel > 3:
        log("Analog Channel ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Channel ungueltig}"
        sendUDP(sArr) 
        return
    if res <0 or res > 3:
        log("Analog Resolution ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Resolution ungueltig}"
        sendUDP(sArr) 
        return
    if amp <0 or amp > 3:
        log("Analog Amplifier ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Amplifier ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr) 
    else:
        log("Analog: Daten nicht bereit...","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Analog Daten nicht bereit}"
        sendUDP(sArr) 
        return

def read_pwm(kanal, adresse):
    if adresse <0x50 or adresse > 0x57:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SPWM;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SPWM;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr)

def read_rgbw(kanal, adresse):
    if adresse <0x57 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SAP;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Modul adresse ungueltig}"
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SAP;{0};{1};".format(kanal,hex(adresse))
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
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
                sendUDP(sArr)
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
                sendUDP(sArr)
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
                sendUDP(sArr)
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
            sendUDP(sArr)
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
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[3]) < 0 or int(arr[3]) >15:
        log("PWM-Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Kanal ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("PWM-Status ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Status ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[5]) <0 or int(arr[5]) >4095:
        log("PWM-Wert ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Wert ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr)

def set_rgbw(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x58 or adresse > 0x5f:
        log("Modul Adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Modul Adresse ungueltig}"
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[3]) <0 or int(arr[3]) >3:
        log("PWMKanal ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";PWM-Kanal ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("StatusRGB ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";StatusRGB ungueltig}"
        sendUDP(sArr) 
        return
    if int(arr[5]) <0 or int(arr[5]) >1:
        log("StatusW ungueltig","ERROR")
        sArr="{"
        sArr+=";".join(arr)
        sArr+=";StatusW ungueltig}"
        sendUDP(sArr) 
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
        sendUDP(sArr)   
        
def read_input(kanal,adresse, manual=0):
    global statIN0,statIN1,statIN2
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        sArr="{"
        sArr+="SAI;{0};{1};".format(kanal,hex(adresse))
        sArr+="Modul adresse ungueltig}"
        sendUDP(sArr) 
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        sArr="{"
        sArr+="SAI;{0};{1};".format(kanal,hex(adresse))
        sArr+="Kanal ungueltig}"
        sendUDP(sArr) 
        return

    # Programm in Schleife -> Auf änderung prüfen -> bei Änderung senden + Neuen Status in Variable schreiben
    # Kanal-> Auswahl statIN0-2
    # Adresse 0x20-0x23 -> Value 0&1 / 2&3 / 4&5 / 6&7
    # Erster Value - Bank A, Zweiter = Bank B
    # wertAltA=statIN0[0]
    # wertAltB=statIN0[1]
    wertAltA=0
    wertAltB=0
    if kanal==0:
        if adresse==0x20:
            wertAltA=statIN0[0]
            wertAltB=statIN0[1]
        if adresse==0x21:
            wertAltA=statIN0[2]
            wertAltB=statIN0[3]
        if adresse==0x22:
            wertAltA=statIN0[4]
            wertAltB=statIN0[5]
        if adresse==0x23:
            wertAltA=statIN0[6]
            wertAltB=statIN0[7]

    if kanal==1:
        if adresse==0x20:
            wertAltA=statIN1[0]
            wertAltB=statIN1[1]
        if adresse==0x21:
            wertAltA=statIN1[2]
            wertAltB=statIN1[3]
        if adresse==0x22:
            wertAltA=statIN1[4]
            wertAltB=statIN1[5]
        if adresse==0x23:
            wertAltA=statIN1[6]
            wertAltB=statIN1[7]

    if kanal==2:
        if adresse==0x20:
            wertAltA=statIN2[0]
            wertAltB=statIN2[1]
        if adresse==0x21:
            wertAltA=statIN2[2]
            wertAltB=statIN2[3]
        if adresse==0x22:
            wertAltA=statIN2[4]
            wertAltB=statIN2[5]
        if adresse==0x23:
            wertAltA=statIN2[6]
            wertAltB=statIN2[7]
    try:
        #GPIO A+B Lesen und String bauen:
        wertA=plexer.readByteData(kanal,adresse,gpioA)
        wertB=plexer.readByteData(kanal,adresse,gpioB)
        if wertAltA!=wertA or wertAltB!=wertB or manual==1:
            #Unterschied, Senden!
            befehl="{SAI;"
            befehl+="{0};{1};".format(kanal,hex(adresse))
            iIn = [wertB, wertA]
            i=int.from_bytes(iIn,"big")
            befehl+="{0};".format(i)
            befehl+="OK}"
            sendUDP(befehl)

            #erneut lesen, auf änderung prüfen:
        wertA2=plexer.readByteData(kanal,adresse,gpioA)
        wertB2=plexer.readByteData(kanal,adresse,gpioB)
        if wertA2!=wertA or wertB2!=wertB:
            befehl="{SAI;"
            befehl+="{0};{1};".format(kanal,hex(adresse))
            iIn = [wertB2, wertA2]
            i=int.from_bytes(iIn,"big")
            befehl+="{0};".format(i)
            befehl+="OK}"
            sendUDP(befehl)
            wertA=wertA2
            wertB=wertB2        
    except OSError as err:
        befehl="{SAI;"
        befehl+="{0};{1};".format(kanal,hex(adresse))
        befehl+="IO Error SAI}"
        sendUDP(befehl)
        log("I/O error: {0}".format(str(err)),"ERROR")
    except:
        befehl="{SAI;"
        befehl+="{0};{1};".format(kanal,hex(adresse))
        befehl+="Fehler SAI}"
        sendUDP(befehl)
        log("Fehler Input lesen: {0}".format(befehl),"ERROR")
    finally:
        if kanal==0:
            if adresse==0x20:
                statIN0[0]=wertA
                statIN0[1]=wertB
            if adresse==0x21:
                statIN0[2]=wertA
                statIN0[3]=wertB
            if adresse==0x22:
                statIN0[4]=wertA
                statIN0[5]=wertB
            if adresse==0x23:
                statIN0[6]=wertA
                statIN0[7]=wertB

        if kanal==1:
            if adresse==0x20:
                statIN1[0]=wertA
                statIN1[1]=wertB
            if adresse==0x21:
                statIN1[2]=wertA
                statIN1[3]=wertB
            if adresse==0x22:
                statIN1[4]=wertA
                statIN1[5]=wertB
            if adresse==0x23:
                statIN1[6]=wertA
                statIN1[7]=wertB

        if kanal==2:
            if adresse==0x20:
                statIN2[0]=wertA
                statIN2[1]=wertB
            if adresse==0x21:
                statIN2[2]=wertA
                statIN2[3]=wertB
            if adresse==0x22:
                statIN2[4]=wertA
                statIN2[5]=wertB
            if adresse==0x23:
                statIN2[6]=wertA
                statIN2[7]=wertB
        
        
        

def modulSuche(delete=0):
    global aOut0, aOut1, aOut2,aPWM0,aPWM1,aPWM2,aIN0,aIN1,aIN2,aANA0,aANA1,aANA2,aRGBW0,aRGBW1,aRGBW2
    #Daten löschen:
    if delete==1:
        aOut0 =[]
        aOut1 =[]
        aOut2 =[]
        aPWM0 =[]
        aPWM1 =[]
        aPWM2 =[]
        aIN0 =[]
        aIN1 =[]
        aIN2 =[]
        aANA0 =[]
        aANA1 =[]
        aANA2 =[] 
        aRGBW0 =[]
        aRGBW1 =[]
        aRGBW2 =[]

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
                    if device!=mux and device!=I2CAdrDS2482:
                        if device>=0x20 and device <=0x23:
                            log("GeCoS 16 In : Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpIN=tmpIN+hex(device)+";"
                            if kanalSearch==0:
                                if device not in aIN0:
                                    aIN0.append(device)
                                    set_input_konfig(kanalSearch,device)
                            elif kanalSearch==1:
                                if device not in aIN1:
                                    aIN1.append(device)
                                    set_input_konfig(kanalSearch,device)
                            elif kanalSearch==2:
                                if device not in aIN2:
                                    aIN2.append(device)
                                    set_input_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("IN")
                            befehl+="}"
                            sendUDP(befehl)
                        elif device>=0x24 and device <=0x27:
                            log("GeCoS 16 OUT: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpOut=tmpOut+hex(device)+";"
                            if kanalSearch==0:
                                if device not in aOut0:
                                    aOut0.append(device)
                                    set_output_konfig(kanalSearch,device)
                            elif kanalSearch==1:
                                if device not in aOut1:
                                    aOut1.append(device)
                                    set_output_konfig(kanalSearch,device)
                            elif kanalSearch==2:
                                if device not in aOut2:
                                    aOut2.append(device)
                                    set_output_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("OUT")
                            befehl+="}"
                            sendUDP(befehl)
                        elif device>=0x50 and device <=0x57:
                            log("GeCoS 16 PWM: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpPWM=tmpPWM+hex(device)+";"
                            if kanalSearch==0:
                                if device not in aPWM0:
                                    aPWM0.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            elif kanalSearch==1:
                                if device not in aPWM1:
                                    aPWM1.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            elif kanalSearch==2:
                                if device not in aPWM2:
                                    aPWM2.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("PWM")
                            befehl+="}"
                            sendUDP(befehl)
                        elif device>=0x58 and device <=0x5f:
                            log("GeCoS 16 RGBW: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpRGBW=tmpRGBW+hex(device)+";"
                            if kanalSearch==0:
                                if device not in aRGBW0:
                                    aRGBW0.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            elif kanalSearch==1:
                                if device not in aRGBW1:
                                    aRGBW1.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            elif kanalSearch==2:
                                if device not in aRGBW2:
                                    aRGBW2.append(device)
                                    set_pwm_konfig(kanalSearch,device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("RGBW")
                            befehl+="}"
                            sendUDP(befehl)
                        elif device>=0x68 and device <=0x6b:
                            log("GeCoS Analog4: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpANA=tmpANA+hex(device)+";"
                            if kanalSearch==0:
                                if device not in aANA0:
                                    aANA0.append(device)
                            elif kanalSearch==1:
                                if device not in aANA1:
                                    aANA1.append(device)
                            elif kanalSearch==2:
                                if device not in aANA2:
                                    aANA2.append(device)
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("ANA")
                            befehl+="}"
                            sendUDP(befehl)
                        else:
                            tmpUnb=tmpUnb+hex(device)+";"
                            log("GeCoS Unbekanntes Gerät: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            befehl="{MOD;"
                            befehl+="{0};{1};".format(kanalSearch,hex(device))
                            befehl+="{0}".format("UNB")
                            befehl+="}"
                            sendUDP(befehl)
            except:
                pass
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16IN',tmpIN)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16OUT',tmpOut)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'UNBEKANNT',tmpUnb)                
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16PWM',tmpPWM)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOSANA4',tmpANA)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16RGBW',tmpRGBW)
    befehl="{MOD;0;0;END}"
    sendUDP(befehl)
        
def bit_from_string(string, index):
    i=int(string)
    return i >> index & 1

if __name__ == '__main__':
    #Konfig Werte MCP:
    log("Script gestartet","ERROR")
    #ArgParser:
    parser = argparse.ArgumentParser(description='True/False print output aktiv')
    parser.add_argument('--d', help='Prints debug output',action='store_true')
    args = parser.parse_args()
    if args.d:
        printDebug=True

    bus=1       # 0 for rev1 boards etc.
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

    thread_gecosOut()
    #RTC Lesen:
    ds = DS1307(plexer, 0x68)
    rtctime = ds.read_datetime()
    temp = ds.read_temp()
    log ("DS3231 Date: {0} Temp: {1} ".format(rtctime.strftime("%d.%m.%Y %H:%M:%S"),str(temp)))
    while True:
        try:
            #Alle eingänge lesen
            time.sleep(0.01)
            #Schleife für Eingang Lesen:
            for device in aIN0:
                try:
                    kanal=0
                    read_input(kanal,device)
                except:
                    pass
            for device in aIN1:
                try:
                    kanal=1
                    read_input(kanal,device)
                except:
                    pass
            for device in aIN2:
                try:
                    kanal=2
                    read_input(kanal,device)
                except:
                    pass
        except KeyboardInterrupt:
            break
