#!/usr/bin/python
# encoding=utf-8
import smbus
import time
import datetime
import socket
import _thread
import configparser
import RPi.GPIO as GPIO

#Globale Variablen
statusI2C=1
clSocket=""
clIP=""
class multiplex:
    
    def __init__(self, bus):
        self.bus = smbus.SMBus(bus)

    def channel(self, address=0x71,channel=0):  # values 0-3 indictae the channel, anything else (eg -1) turns off all channels
        
        if   (channel==0): action = 0x04
        elif (channel==1): action = 0x05
        elif (channel==2): action = 0x06
        elif (channel==3): action = 0x07
        else : action = 0x00
        self.bus.write_byte_data(address,0x04,action)  #0x04 is the register for switching channels 

#Konfiguration schreiben, wenn nicht vorhanden, anlegen, sonst gewünschte Daten hinzufügen/Anpassen
def configSchreiben(bereich,wert1, wert2):
    config = configparser.ConfigParser()
    config.read('Config.cfg')
    if config.has_section('Allgemein') != True:
        config['Allgemein'] = {'IP':'127.0.0.1','Port':'8000',
                            'StartZeit':str(datetime.datetime.now())}
        
    if bereich=='Allgemein':       
        if config.has_option('Allgemein','StartZeit'):
            config.set('Allgemein','StartZeit',str(datetime.datetime.now()))
        else:                    
            config['Allgemein'] = {'StartZeit':str(datetime.datetime.now())}
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

def set_output_konfig(kanal,adresse):
    global statusI2C
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig","ERROR")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
    statusI2C=0
    #Konfiguration als Ausgangsmodul:
    try:        
        plexer.channel(mux,kanal) 
        plexer.bus.write_byte_data(adresse,bankAKonfig,outputKonfig)
        plexer.bus.write_byte_data(adresse,bankBKonfig,outputKonfig)   
        log("Adresse: " +str(hex(adresse)) + " - Port A + B als Output gesetzt")
    except:
        statusI2C=1
        log("Fehler beim Output konfigurieren","ERROR")
    statusI2C=1

def set_pwm_konfig(kanal, adresse):
    global statusI2C
    if adresse <0x50 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
    statusI2C=0
    try:
        plexer.channel(mux,kanal)
        log("Adresse: {0} - PWM Konfig gesetzt".format(hex(adresse)))
        
        #Mode1 = sleep  Register 0  Wert = 16
        plexer.bus.write_byte_data(adresse,0x00,0x10)
        #prescale: round((25.000.000/(4096*Freuqnz))-1) Frequenz aus Konfig lesen!
        prescale=round((25000000/(4096*freqStd))-1)
        plexer.bus.write_byte_data(adresse,0xFE,prescale)
        #mode1 = sleep Register 0  Wert=32
        plexer.bus.write_byte_data(adresse,0x00,0x20)
        #mode2 = Ausgang Register 1  Wert = 4
        plexer.bus.write_byte_data(adresse,0x01,0x04)        
        statusI2C=1
    except:
        statusI2C=1
        log("Fehler beim PWM konfigurieren","ERROR")
    
def set_input_konfig(kanal,adresse):
    global statusI2C
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig","ERROR")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR")
        time.sleep(0.001)
    statusI2C=0
    #Konfiguration als Ausgangsmodul:
    try:
        plexer.channel(mux,kanal)
        plexer.bus.write_byte_data(adresse,bankAKonfig,inputKonfig)
        plexer.bus.write_byte_data(adresse,bankBKonfig,inputKonfig)
        plexer.bus.write_byte_data(adresse,IOCONA,0x44)
        plexer.bus.write_byte_data(adresse,IOCONB,0x44)
        plexer.bus.write_byte_data(adresse,DEFVALA,0x00)
        plexer.bus.write_byte_data(adresse,DEFVALB,0x00)
        plexer.bus.write_byte_data(adresse,INTCONA,0x00)
        plexer.bus.write_byte_data(adresse,INTCONB,0x00)
        plexer.bus.write_byte_data(adresse,GPPUA,0x00)
        plexer.bus.write_byte_data(adresse,GPPUB,0x00)
        plexer.bus.write_byte_data(adresse,IPOLA,0x00)
        plexer.bus.write_byte_data(adresse,IPOLB,0x00)
        plexer.bus.write_byte_data(adresse,GPINTENA,0xFF)
        plexer.bus.write_byte_data(adresse,GPINTENB,0xFF)
        log("Adresse:{0} - Port A + B als Input gesetzt".format(hex(adresse)),"INFO")
        statusI2C=1
    except:
        log("Fehler beim Input konfigurieren","ERROR")
        statusI2C=1
    finally:
        statusI2C=1

def sendUDP(data):
    #Daten Senden
    global clSocket
    global clIP
    try:
        res=clSocket.send(data.encode())
    except IOError as err:
        log("Fehler:{0} ; {1}".format(err,data),"ERROR")
    #Daten pruefen:
    if (int(res)==int(data.encode().__len__())):
        log("Gesendet: {0} : {1}".format(clIP,data))
    else:
        log("Fehler. Gesendet: {0} Empfangen: {1}".format(data.__len__(),res),"ERROR")
        log("Fehlerdaten: {0}".format(data),"ERROR")

def getUDP():
    global statusI2C
    global clSocket
    global clIP
    conClosed=False
    (clSocket, clIP) = tcpSocket.accept()
    log("Verbunden: {0}".format(clIP))
    while True:
        #log("Get UDP")
        GeCoSInData=""
        data=""
        arr=""
        while data[-2:]!="\r\n":
            try:
                #Testen ob noch verbunden? 
                blk=clSocket.recv(1).decode("utf-8")
                if len(blk)==0:
                    conClosed=True
                    break 
            except:
                log("Fehler beim Empfangen")
            data+=blk
        GeCoSInData=data[:-2]
        data = ""
        if GeCoSInData[0]=="{":
            GeCoSInData.replace("{","")
            GeCoSInData.replace("}","")
            if GeCoSInData=="MOD":
                modulSuche()
            elif GeCoSInData=="SAI":
                interrutpKanal(intKanal0)
                interrutpKanal(intKanal1)
                interrutpKanal(intKanal2)
            elif GeCoSInData=="SAP":
                pwmAll()
            elif GeCoSInData=="SAO":
                ReadOutAll()
            elif len(GeCoSInData)>=13: #13
                try:
                    arr=GeCoSInData.split(";")
                    if arr[0]=="SOM":
                        set_output(arr)
                    elif arr[0]=="SPM":
                        set_pwm(arr)
                    elif arr[0]=="SAM":
                        read_analog(arr)
                    # if len(arr)==20:
                    #     adresse=int(arr[1],16)
                    #     if adresse>=0x24 and adresse <=0x27:
                    #         set_output(arr)
                    #     elif adresse>=0x50 and adresse <=0x5f:
                    #         set_pwm(arr)             
                    # elif len(arr)==7:
                    #     adresse=int(arr[1],16)
                    #     if adresse>=0x68 and adresse <=0x6B:
                    #         read_analog(arr)
                except:
                    arr=""
                    log("Fehler: {0}".format(GeCoSInData),"ERROR")
                    statusI2C=1
        else:
            arr=""
            log("Fehler: {0}".format(GeCoSInData),"ERROR")
            statusI2C==1
            #Verbindung unterbrochen, Neue Verbindung akzeptieren:
            if conClosed==True:
                thread_gecosOut()
                break

def thread_gecosOut():
    _thread.start_new_thread(getUDP,())

def thread_interrupt(pin):
    _thread.start_new_thread(interrutpKanal,(pin,))

def read_output(kanal,adresse):
    global statusI2C
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        return

    sArr="{0};{1};".format(kanal,adresse)
    try:
        while True:
            if statusI2C==1:
                break
            log("I2C Status: {0}".format(str(statusI2C))) 
            time.sleep(0.001)            
        
        statusI2C=0
        #Bytes fuer Bank A + B auslesen
        plexer.channel(mux,kanal) 
        iOutA=plexer.bus.read_byte_data(adresse,bankA)
        iOutB=plexer.bus.read_byte_data(adresse,bankB)
        i=0
        for i in range(8):
            if bit_from_string(iOutA,i)==1:
                sArr+="1;"
            else:
                sArr+="0;"
        i=0
        for i in range(8):
            if bit_from_string(iOutB,i)==1:
                sArr+="1;"
            else:
                sArr+="0;"
        sendUDP(sArr)   
    except OSError as err:
        statusI2C=1
        log("I/O error: {0}".format(err),"ERROR")
    except:
        statusI2C=1
        log("Fehler Output lesen: {0}".format(sArr),"ERROR")
    finally:
        statusI2C=1

def set_output(arr):
    global statusI2C
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        sArr="{{"
        sArr+=";".join(arr)
        sArr+=";Modul adresse ungueltig}}"
        sendUDP(sArr) 
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        sArr="{{"
        sArr+=";".join(arr)
        sArr+=";Kanal ungueltig}}"
        sendUDP(sArr) 
        return
        
    try:
        while True:
            if statusI2C==1:
                break
            log("I2C Status: {0}".format(str(statusI2C))) 
            time.sleep(0.001)            
        
        statusI2C=0
        #Bytes fuer Bank A + B auslesen
        plexer.channel(mux,kanal) 
        iOutA=plexer.bus.read_byte_data(adresse,bankA)
        iOutB=plexer.bus.read_byte_data(adresse,bankB)
        i=0
        for i in range(8):
            if (int(arr[i+2])==1):
                iOutA=set_bit(iOutA,i,True)
            else:
                iOutA=set_bit(iOutA,i,False)
        i=0
        for i in range(8):
            if (int(arr[i+10])==1):
                iOutB=set_bit(iOutB,i,True)
            else:
                iOutB=set_bit(iOutB,i,False)                
        plexer.channel(mux,kanal)
        plexer.bus.write_byte_data(adresse,bankA,iOutA)
        plexer.bus.write_byte_data(adresse,bankB,iOutB)
        #Prüfen und antworten.
        iOutA=plexer.bus.read_byte_data(adresse,bankA)
        iOutB=plexer.bus.read_byte_data(adresse,bankB)
        sStatus="OK"        
    except OSError as err:
        statusI2C=1
        sStatus=err
        log("I/O error: {0}".format(err),"ERROR")
    except:
        statusI2C=1
        sStatus=err
        log("Fehler Output: {0}".format(arr),"ERROR")
    finally:
        statusI2C=1
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sArr="{{"
        sArr+=";".join(arr)
        # sArr="{{{0};{1};{2};".format(arr[0],kanal,adresse)
        # i=0
        # for i in range(8):
        #     if bit_from_string(iOutA,i)==1:
        #         sArr+="1;"
        #     else:
        #         sArr+="0;"
        # i=0
        # for i in range(8):
        #     if bit_from_string(iOutB,i)==1:
        #         sArr+="1;"
        #     else:
        #         sArr+="0;"
        sArr+=";{0}}}".format(sStatus.Replace(";",""))
        sendUDP(sArr)   
        
def log(message, level="INFO"):
    timestamp= time.strftime("%d.%m.%Y %H:%M:%S", time.localtime(time.time()))
    print("{0} {1}: {2}".format(timestamp, level, message))

       
def set_bit(v, index, x): #v=original wert, x= true oder false
    #Bit auf 1/0 setzen (True oder False)
    mask = 1<< index
    v&=~mask
    if x:
        v |= mask
    return v
    
def ReadOutAll():
    global statusI2C
    for kanal in range(3):
        for i in range(4):
            device=0x24+i
            try:
                read_output(kanal,device)
            except:
                statusI2C=1
            pass

def pwmAll():
    global statusI2C
    for kanal in range(3):
        for i in range(15):
            device=0x50+i
            try:
                read_pwm(kanal,device)
            except:
                statusI2C=1
            pass

def interrutpKanal(pin):
    global statusI2C
    #Kanal nach INT Pin Wählen:
    if pin==intKanal0:
        kanal=0
    elif pin==intKanal1:
        kanal=1
    elif pin==intKanal2:
        kanal=2
    else:
        kanal=0                
    #Versuche alle 16In module zu lesen und Int zu finden.
    for i in range(4):
            device=0x20+i
            try:
                read_input(kanal,device)
            except:
                statusI2C=1
            pass
    if GPIO.input(pin)==GPIO.LOW:
        for i in range(4):
            device=0x20+i
            try:
                read_input(kanal,device)
            except:
                statusI2C=1
            pass
            
def read_analog(arr):
    global statusI2C
    adresse=int(arr[1],16)
    kanal=int(arr[0])
    channel=int(arr[2])
    res=int(arr[3])
    amp=int(arr[4])
    if adresse <0x68 or adresse > 0x6B:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
        
    if channel <0 or channel > 3:
        log("Channel ungueltig","ERROR")
        return
    if res <0 or res > 3:
        log("Resolution ungueltig","ERROR")
        return
    if amp <0 or amp > 3:
        log("Amplifier ungueltig","ERROR")
        return
            
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
        
    statusI2C=0
    plexer.channel(mux,kanal)
    #Config Bits bit5+6 = Channel
    # Bit 4  4Converison Mode = 1
    # Bits 3+2 Resolution
    # Bist 0+1 = Amplifier
    #arr[3] = Resolution  
    #arr[4] = Amplifier
    bconfig=b"0"
    bconfig = channel <<5 | 1 <<4 | res <<2 | amp
    plexer.bus.write_byte(adresse,bconfig)
    #Warten bis ergebnis:
    #I2C Port Freigeben:
    statusI2C=1
    if res==0:
        time.sleep(0.010)
    elif res==1:
        time.sleep(0.022)
    elif res==2:
        time.sleep(0.080)
    else:
        time.sleep(0.300)
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
    statusI2C=0
    #Je Nach Auflösung 3 oder 4Byte lesen:
    #res=3 dann 4 sonst 3
    readyBit=0
    if res==3:
        erg=plexer.bus.read_i2c_block_data(adresse,bconfig,4)
        readyBit=bit_from_string(erg[3],8)
    else:
        erg=plexer.bus.read_i2c_block_data(adresse,bconfig,3)
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
        #print("Wert:",wert)
        befehl="{0};{1};{2};{3}".format(kanal,hex(adresse),channel,round(wert,3))
        sendUDP(befehl)    
    else:
        log("Analog: Daten nicht bereit...","ERROR") 
    statusI2C=1

def read_pwm(kanal, adresse):
    global statusI2C
    if adresse <0x50 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
        
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
        
    statusI2C=0
    plexer.channel(mux,kanal)
    befehl="{0};{1};".format(kanal,hex(adresse))
    for i in range(16):
        startAdr=int(i*4+6)
        #LowByte
        lByte=plexer.bus.read_byte_data(adresse,startAdr+2)
        #HighByte
        hByte=plexer.bus.read_byte_data(adresse,startAdr+3)
        wert=0
        wert = wert*256+int(hByte)
        wert = wert*256+int(lByte)
        #wert=int.from_bytes(lByte+hByte,byteorder="big")
        if wert==0:
            wert=0
        else:
            wert=round((wert/4095)*100)
        befehl+= str(wert)+";"
    print(befehl)

    sendUDP(befehl)
    #log(befehl)
    statusI2C=1


def set_pwm(arr):
    global statusI2C
    adresse=int(arr[1],16)
    kanal=int(arr[0])
    if adresse <0x50 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
    statusI2C=0
    plexer.channel(mux,kanal)
    #LED_ON Immer 0
    #LED_OFF 4096*X%-1
    #Array durchlaufen 0-15 (+1) = ausgang; ausgang*4+6 = Start Adresse LED_ON_L 
    for i in range(16):
        wert = int(round(4095*(int(arr[i+2])/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        plexer.bus.write_byte_data(adresse,startAdr,0x00)
        plexer.bus.write_byte_data(adresse,startAdr+1,0x00)
        plexer.bus.write_byte_data(adresse,startAdr+2,lByte)
        plexer.bus.write_byte_data(adresse,startAdr+3,hByte)
    statusI2C=1
        
def read_input(kanal,adresse):
    global statusI2C
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return
        
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"ERROR") 
        time.sleep(0.001)
        
    statusI2C=0
    plexer.channel(mux,kanal)
    #GPIO A+B Lesen und String bauen:
    wertA=plexer.bus.read_byte_data(adresse,gpioA)
    wertB=plexer.bus.read_byte_data(adresse,gpioB)
    befehl="{0};{1};".format(kanal,hex(adresse))
    for i in range(8):
        intSA = bit_from_string(wertA,i)
        befehl+= str(intSA)+";"
    for i in range(8):
        intSB = bit_from_string(wertB,i)
        befehl+= str(intSB)+";"
    sendUDP(befehl)
    #erneut lesen, auf änderung prüfen:
    wertA2=plexer.bus.read_byte_data(adresse,gpioA)
    wertB2=plexer.bus.read_byte_data(adresse,gpioB)
    befehl="{0};{1};".format(kanal,hex(adresse))
    #print("Vergleich A:",wertA2,"-",wertA)
    #print("Vergleich B:",wertB2,"-",wertB)
    if wertA2!=wertA or wertB2!=wertB:
        for i in range(8):
            intSA = bit_from_string(wertA,i-1)
            befehl+= str(intSA)+";"
        for i in range(8):
            intSB = bit_from_string(wertB,i-1)
            befehl+= str(intSB)+";"
        sendUDP(befehl)
    statusI2C=1

def modulSuche():
    global statusI2C
    while True:
        if statusI2C==1:
            break
        log("I2C Status: {0}".format(str(statusI2C)),"I2C Busy") 
        time.sleep(0.001)
        
    for kanalSearch in range(3):
        log("Suche Bus: {0} Kanal: {1}".format(bus,kanalSearch))
        plexer.channel(mux,kanalSearch)
        tmpIN=""
        tmpOut=""
        tmpRGBW=""
        tmpPWM=""
        tmpUnb=""
        tmpANA=""
        for device in range(128):
            try:
                plexer.bus.read_byte(device)
                if device!=mux and device!=oneWire:
                    if device>=0x20 and device <=0x23:
                        log("GeCoS 16 In : Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                        tmpIN=tmpIN+hex(device)+";"
                        set_input_konfig(kanalSearch,device)
                    elif device>=0x24 and device <=0x27:
                        log("GeCoS 16 OUT: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                        tmpOut=tmpOut+hex(device)+";"
                        set_output_konfig(kanalSearch,device)
                    elif device>=0x50 and device <=0x57:
                        log("GeCoS 16 PWM: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                        tmpPWM=tmpPWM+hex(device)+";"
                        set_pwm_konfig(kanalSearch,device)
                    elif device>=0x58 and device <=0x5f:
                        log("GeCoS 16 RGBW: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                        tmpRGBW=tmpRGBW+hex(device)+";"
                        set_pwm_konfig(kanalSearch,device)
                    elif device>=0x68 and device <=0x6b:
                        log("GeCoS Analog4: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                        tmpANA=tmpANA+hex(device)+";"
                    else:
                        tmpUnb=tmpUnb+hex(device)+";"
                        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'UNBEKANNT',hex(device))
                        log("GeCoS Unbekanntes Gerät: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
            except:
                pass
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16IN',tmpIN)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16OUT',tmpOut)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'UNBEKANNT',tmpUnb)                
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16PWM',tmpPWM)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOSANA4',tmpANA)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16RGBW',tmpRGBW)  
        
def bit_from_string(string, index):
    i=int(string)
    return i >> index & 1
    #i,j=divmod(index,8)
    #if string & (1<<j):
    #    return 1
    #else:
    #    return 0

if __name__ == '__main__':
    #Konfig Werte MCP:
    bus=1       # 0 for rev1 boards etc.
    mux=0x71
    oneWire=0x18
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
   
    #TCP Socket:
    tcpSocket=socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Internet, UDP
    tcpSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    tcpSocket.bind(("0.0.0.0",miniServerPort))
    tcpSocket.listen(5)
    thread_gecosOut()
    
   
    #Interrupt routine GeCoS 16 IN
    GPIO.setmode(GPIO.BCM)
    #Kanal0
    GPIO.setup(intKanal0,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.add_event_detect(intKanal0, GPIO.FALLING, callback=thread_interrupt, bouncetime = 10)
    #Kanal1
    GPIO.setup(intKanal1,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.add_event_detect(intKanal1, GPIO.FALLING, callback=thread_interrupt, bouncetime = 10)
    #Kanal2
    GPIO.setup(intKanal2,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
    GPIO.add_event_detect(intKanal2, GPIO.FALLING, callback=thread_interrupt, bouncetime = 10)
    
    #MUX initialisieren:
    log("Bus: " + str(bus) + " Kanal: " + str(kanal))
    plexer = multiplex(bus)
    plexer.channel(mux,kanal)
    time.sleep(0.01)
    modulSuche()
    log(datetime.datetime.now())
    log("UDP Port: {0}".format(miniServerPort))
    thread_interrupt(intKanal0)
    thread_interrupt(intKanal1)
    thread_interrupt(intKanal2)
    while True:
        #UDP Daten Empfangen: Wartet auf Daten. Zwei Scripte? Eins IN eins OUT?
        time.sleep(0.1)

