#!/usr/bin/env python3
# encoding=utf-8
"""
Mux - I2C Multiplexer Modul für GeCoS-Server
Verwaltet die Kommunikation über I2C-Multiplexer
"""

import smbus
import time
import logging
from typing import Optional, List
from enum import IntEnum

# Logger konfigurieren
logger = logging.getLogger(__name__)


class MuxChannel(IntEnum):
    """I2C Multiplexer Kanal-Konfiguration"""
    CHANNEL_0 = 0x04
    CHANNEL_1 = 0x05
    CHANNEL_2 = 0x06
    CHANNEL_3 = 0x07
    CHANNEL_OFF = 0x00


class Multiplex:
    """
    I2C Multiplexer-Klasse für GeCoS-Server
    
    Verwaltet die Kommunikation mit I2C-Geräten über einen Multiplexer
    und stellt Thread-sichere Operationen bereit.
    """
    
    MUX_ADDRESS = 0x71
    I2C_RETRY_COUNT = 500
    I2C_RETRY_WARN_THRESHOLD = 150
    I2C_RETRY_DELAY = 0.001
    
    def __init__(self, bus: int):
        """
        Initialisiert den Multiplexer
        
        Args:
            bus: I2C-Busnummer (üblicherweise 1)
        """
        self.bus = smbus.SMBus(bus)
        self._status_i2c = 1
        logger.info(f"Multiplexer initialisiert auf Bus {bus}")
    
    @property
    def status_i2c(self) -> int:
        """Gibt den aktuellen I2C-Status zurück"""
        return self._status_i2c
    
    @status_i2c.setter
    def status_i2c(self, value: int) -> None:
        """Setzt den I2C-Status"""
        self._status_i2c = value

    def _check_i2c(self) -> bool:
        """
        Überprüft, ob der I2C-Bus verfügbar ist
        
        Wartet bis der Bus frei ist oder ein Timeout erreicht wird.
        
        Returns:
            bool: True wenn Bus verfügbar, False bei Timeout
        """
        retry_count = 0
        
        while True:
            if self._status_i2c == 1:
                return True
            
            retry_count += 1
            if retry_count >= self.I2C_RETRY_WARN_THRESHOLD:
                logger.warning(f"I2C Status: {self._status_i2c}")
            
            if retry_count >= self.I2C_RETRY_COUNT:
                logger.error(f"I2C Status Abbruch: {self._status_i2c}")
                self._status_i2c = 1
                return False
            
            time.sleep(self.I2C_RETRY_DELAY)
        
        return False

    def select_channel(self, channel: int = 0) -> None:
        """
        Wählt einen Multiplexer-Kanal aus
        
        Args:
            channel: Kanalnummer (0-3), 0 deaktiviert alle Kanäle
        """
        if 0 <= channel <= 3:
            action = MuxChannel(MuxChannel.CHANNEL_0 + channel)
        else:
            action = MuxChannel.CHANNEL_OFF
        
        try:
            self.bus.write_byte_data(self.MUX_ADDRESS, 0x04, action)
        except Exception as e:
            logger.error(f"Fehler beim Kanalwechsel auf {channel}: {e}")
            raise

    def write_byte_data(self, kanal: int, address: int, register: int, wert: int) -> bool:
        """
        Schreibt ein Byte zu einer I2C-Adresse
        
        Args:
            kanal: Multiplexer-Kanal (0-3)
            address: I2C-Geräteadresse
            register: Register-Adresse
            wert: Zu schreibender Wert (0-255)
            
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        if not self._check_i2c():
            return False
        
        self._status_i2c = 0
        try:
            self.select_channel(kanal)
            time.sleep(self.I2C_RETRY_DELAY)
            self.bus.write_byte_data(address, register, wert)
            return True
        except Exception as e:
            logger.error(f"Fehler beim Schreiben (Kanal={kanal}, Addr={hex(address)}, Reg={hex(register)}): {e}")
            return False
        finally:
            self._status_i2c = 1

    def read_byte(self, kanal: int, address: int) -> Optional[int]:
        """
        Liest ein Byte von einer I2C-Adresse
        
        Args:
            kanal: Multiplexer-Kanal (0-3)
            address: I2C-Geräteadresse
            
        Returns:
            Optional[int]: Gelesener Wert oder None bei Fehler
        """
        if not self._check_i2c():
            return None
        
        self._status_i2c = 0
        try:
            self.select_channel(kanal)
            wert = self.bus.read_byte(address)
            return wert
        except Exception as e:
            logger.error(f"Fehler beim Lesen (Kanal={kanal}, Addr={hex(address)}): {e}")
            return None
        finally:
            self._status_i2c = 1

    def read_byte_data(self, kanal: int, address: int, register: int) -> Optional[int]:
        """
        Liest ein Byte von einer I2C-Register-Adresse
        
        Args:
            kanal: Multiplexer-Kanal (0-3)
            address: I2C-Geräteadresse
            register: Register-Adresse
            
        Returns:
            Optional[int]: Gelesener Wert oder None bei Fehler
        """
        if not self._check_i2c():
            return None
        
        self._status_i2c = 0
        try:
            self.select_channel(kanal)
            wert = self.bus.read_byte_data(address, register)
            return wert
        except Exception as e:
            logger.error(f"Fehler beim Lesen (Kanal={kanal}, Addr={hex(address)}, Reg={hex(register)}): {e}")
            return None
        finally:
            self._status_i2c = 1

    def read_block_data(self, kanal: int, address: int, register: int, cnt: int) -> List[int]:
        """
        Liest einen Datenblock von einer I2C-Adresse
        
        Args:
            kanal: Multiplexer-Kanal (0-3)
            address: I2C-Geräteadresse
            register: Register-Adresse
            cnt: Anzahl zu lesender Bytes
            
        Returns:
            List[int]: Liste der gelesenen Werte oder [0x00, ...] bei Fehler
        """
        if not self._check_i2c():
            return [0x00] * cnt
        
        self._status_i2c = 0
        try:
            self.select_channel(kanal)
            wert = self.bus.read_i2c_block_data(address, register, cnt)
            return wert
        except Exception as e:
            logger.error(f"Fehler beim Block-Lesen (Kanal={kanal}, Addr={hex(address)}, Cnt={cnt}): {e}")
            return [0x00] * cnt
        finally:
            self._status_i2c = 1

    def write_byte(self, kanal: int, address: int, register: int) -> bool:
        """
        Schreibt ein einzelnes Byte an eine I2C-Adresse
        
        Args:
            kanal: Multiplexer-Kanal (0-3)
            address: I2C-Geräteadresse
            register: Zu schreibendes Byte
            
        Returns:
            bool: True bei Erfolg, False bei Fehler
        """
        if not self._check_i2c():
            return False
        
        self._status_i2c = 0
        try:
            self.select_channel(kanal)
            self.bus.write_byte(address, register)
            return True
        except Exception as e:
            logger.error(f"Fehler beim Byte-Schreiben (Kanal={kanal}, Addr={hex(address)}): {e}")
            return False
        finally:
            self._status_i2c = 1
    
    # Rückwärtskompatibilität - Alte Methodennamen
    def channel(self, channel: int = 0) -> None:
        """Veraltete Methode - Verwenden Sie select_channel()"""
        self.select_channel(channel)
    
    def writeByteData(self, kanal: int, address: int, register: int, wert: int) -> bool:
        """Veraltete Methode - Verwenden Sie write_byte_data()"""
        return self.write_byte_data(kanal, address, register, wert)
    
    def readByte(self, kanal: int, address: int) -> Optional[int]:
        """Veraltete Methode - Verwenden Sie read_byte()"""
        return self.read_byte(kanal, address)
    
    def readByteData(self, kanal: int, address: int, register: int) -> Optional[int]:
        """Veraltete Methode - Verwenden Sie read_byte_data()"""
        return self.read_byte_data(kanal, address, register)
    
    def readBlockData(self, kanal: int, address: int, register: int, cnt: int) -> List[int]:
        """Veraltete Methode - Verwenden Sie read_block_data()"""
        return self.read_block_data(kanal, address, register, cnt)
    
    def writeByte(self, kanal: int, address: int, register: int) -> bool:
        """Veraltete Methode - Verwenden Sie write_byte()"""
        return self.write_byte(kanal, address, register)


# Alias für Rückwärtskompatibilität
multiplex = Multiplex
