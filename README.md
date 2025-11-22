# Laboratorio-5-comunicaciones-industriales



 Descripción General del Laboratorio

Este laboratorio integral cubre tres sistemas fundamentales en automatización industrial, implementando desde la infraestructura de red básica hasta sistemas de control avanzados con PLCs y comunicación industrial.


---

 Punto 1: Configuración y Prueba Avanzada de Red Sencilla

 Objetivo Completo

Establecer, verificar y documentar completamente la configuración de red industrial, incluyendo protocolos básicos y diagnóstico de conectividad.

 Procedimiento Detallado

1.1 Configuración de Interfaces de Red

# Ver estado completo de todas las interfaces
ip link show
ip addr show

# Configurar IP estática (ejemplo)
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Ver routing table
ip route show
route -n

1.2 Análisis Avanzado ARP

# Ver tabla ARP completa con timers
arp -v

# Monitoreo continuo ARP
sudo arping -I eth0 192.168.1.1

# Limpiar cache ARP específica
sudo arp -d 192.168.1.1

1.3 Pruebas de Conectividad Extendidas

# Ping extendido con estadísticas
ping -c 20 -i 0.5 192.168.1.1

# Prueba de throughput con paquetes grandes
ping -s 1472 -M do 192.168.1.1

# Traceroute con diferentes protocolos
traceroute -T google.com  # TCP
traceroute -I google.com  # ICMP
traceroute -U google.com  # UDP

1.4 Análisis de Puertos y Servicios

# Escaneo de puertos locales
netstat -tulpn

# Ver conexiones establecidas
ss -tunap

# Escaneo de red con nmap
sudo nmap -sS 192.168.1.0/24

1.5 Diagnóstico de Rendimiento

# Prueba de ancho de banda
iperf3 -c 192.168.1.1 -t 30

# Monitoreo de interfaz en tiempo real
iftop -i eth0

# Estadísticas de red
sar -n DEV 1 10

 Documentación de Arquitectura

Diagrama de red creado con draw.io

Tabla de direccionamiento IP

Documentación de VLANs si aplica

Políticas de firewall implementadas



---

 Punto 2: Comunicación RS485 Modbus RTU – Implementación Completa

 Hardware Requerido

Raspberry Pi 4 Model B 4GB

Arduino Uno R3 + Shield MAX485

Cable par trenzado AWG22

USB–RS485 (opcional)

Resistencias 120Ω



---

 Configuración en Raspberry Pi

2.1 Instalación de Dependencias

sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv git build-essential
pip install minimalmodbus pyserial modbus-tk
sudo apt install minicom screen

2.2 Configuración del Puerto Serial

sudo nano /boot/config.txt

# Agregar:
enable_uart=1
dtoverlay=disable-bt
core_freq=250

sudo systemctl disable hciuart
sudo systemctl disable bluetooth

ls -l /dev/ttyS0
sudo usermod -a -G dialout $USER

2.3 Configuración GPIO para MAX485

import RPi.GPIO as GPIO
import time

DE_PIN = 17
RE_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_PIN, GPIO.OUT)
GPIO.setup(RE_PIN, GPIO.OUT)

def set_transmit_mode():
    GPIO.output(DE_PIN, GPIO.HIGH)
    GPIO.output(RE_PIN, GPIO.HIGH)
    time.sleep(0.001)

def set_receive_mode():
    GPIO.output(DE_PIN, GPIO.LOW)
    GPIO.output(RE_PIN, GPIO.LOW)

2.4 Código Maestro Modbus (Python)

#!/usr/bin/env python3


2.5 Script Automático de Configuración

Script maestro Modbus RTU para Raspberry Pi
Comunicación RS485 con Arduino esclavo
"""

import minimalmodbus
import serial
import time
import logging
from datetime import datetime

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('modbus_communication.log'),
        logging.StreamHandler()
    ]
)

class ModbusMaster:
    def __init__(self, port, slave_address, baudrate=9600):
        self.port = port
        self.slave_address = slave_address
        self.baudrate = baudrate
        self.instrument = None
        self.setup_modbus()
    
    def setup_modbus(self):
        """Configurar conexión Modbus RTU"""
        try:
            self.instrument = minimalmodbus.Instrument(self.port, self.slave_address)
            self.instrument.serial.baudrate = self.baudrate
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 1.0
            self.instrument.mode = minimalmodbus.MODE_RTU
            self.instrument.clear_buffers_before_each_transaction = True
            
            logging.info(f"Modbus Master configurado en {self.port}")
            
        except Exception as e:
            logging.error(f"Error configurando Modbus: {e}")
            raise
    
    def read_temperature(self):
        """Leer registro de temperatura del esclavo"""
        try:
            # Leer registro 0 (temperatura) con 1 decimal
            temperature = self.instrument.read_register(0, 1)
            logging.info(f"Temperatura leída: {temperature}°C")
            return temperature
            
        except Exception as e:
            logging.error(f"Error leyendo temperatura: {e}")
            return None
    
    def read_humidity(self):
        """Leer registro de humedad del esclavo"""
        try:
            # Leer registro 1 (humedad) con 1 decimal
            humidity = self.instrument.read_register(1, 1)
            logging.info(f"Humedad leída: {humidity}%")
            return humidity
            
        except Exception as e:
            logging.error(f"Error leyendo humedad: {e}")
            return None
    
    def write_register(self, register_address, value):
        """Escribir en registro del esclavo"""
        try:
            self.instrument.write_register(register_address, value)
            logging.info(f"Escrito valor {value} en registro {register_address}")
            return True
            
        except Exception as e:
            logging.error(f"Error escribiendo registro: {e}")
            return False
    
    def read_multiple_registers(self, start_address, quantity):
        """Leer múltiples registros"""
        try:
            registers = self.instrument.read_registers(start_address, quantity)
            logging.info(f"Registros leídos: {registers}")
            return registers
            
        except Exception as e:
            logging.error(f"Error leyendo múltiples registros: {e}")
            return None

def main():
    """Función principal"""
    # Configuración
    SERIAL_PORT = '/dev/ttyS0'
    SLAVE_ADDRESS = 1
    BAUD_RATE = 9600
    
    # Crear instancia del maestro Modbus
    try:
        master = ModbusMaster(SERIAL_PORT, SLAVE_ADDRESS, BAUD_RATE)
    except Exception as e:
        logging.error(f"No se pudo inicializar Modbus Master: {e}")
        return
    
    # Bucle principal de monitoreo
    read_interval = 5  # segundos
    
    while True:
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"\n--- Lectura {timestamp} ---")
            
            # Leer datos del esclavo
            temperature = master.read_temperature()
            humidity = master.read_humidity()
            
            if temperature is not None and humidity is not None:
                print(f"Temperatura: {temperature}°C")
                print(f"Humedad: {humidity}%")
                
                # Guardar datos en archivo
                with open('sensor_data.csv', 'a') as f:
                    f.write(f"{timestamp},{temperature},{humidity}\n")
            
            # Ejemplo de escritura (cada 30 segundos)
            current_second = datetime.now().second
            if current_second % 30 == 0:
                master.write_register(2, current_second)
            
            time.sleep(read_interval)
            
        except KeyboardInterrupt:
            logging.info("Interrupción por usuario. Cerrando aplicación...")
            break
        except Exception as e:
            logging.error(f"Error en bucle principal: {e}")
            time.sleep(read_interval)

if __name__ == "__main__":
    main()




sudo apt update && sudo apt upgrade -y
sudo apt install python3-pip python3-venv git -y
pip3 install minimalmodbus pyserial
sudo raspi-config nonint do_serial 2
sudo usermod -a -G dialout $USER
mkdir -p ~/modbus_logs


---

 Código Arduino Completo (Esclavo Modbus)

/**
 * Arduino Modbus RTU Slave

bash
#!/bin/bash
# setup_modbus.sh - Configuración automática Raspberry Pi para Modbus

echo "Configurando Raspberry Pi para comunicación Modbus RTU..."

# Actualizar sistema
echo "Actualizando sistema..."
sudo apt update && sudo apt upgrade -y

# Instalar dependencias
echo "Instalando dependencias..."
sudo apt install python3-pip python3-venv git -y

# Instalar librerías Python
echo "Instalando librerías Python..."
pip3 install minimalmodbus pyserial

# Configurar puerto serial
echo "Configurando puerto serial..."
sudo raspi-config nonint do_serial 2  # Habilitar serial, deshabilitar console

# Agregar usuario al grupo dialout
sudo usermod -a -G dialout $USER

# Crear directorio para logs
mkdir -p ~/modbus_logs

echo "Configuración completada. Por favor, reinicie el sistema."


Código Arduino Completo (Esclavo Modbus)

cpp
/**
 * Arduino Modbus RTU Slave
 * Controlado por Raspberry Pi via RS485
 */

#include <ModbusRtu.h>
#include <DHT.h>

// Definiciones pines
#define DHT_PIN 2
#define DHT_TYPE DHT22

// Dirección Modbus del esclavo
#define MODBUS_SLAVE_ID 1

// Objetos
Modbus slave(MODBUS_SLAVE_ID, 0, 0);  // id, trasmit control, USB
DHT dht(DHT_PIN, DHT_TYPE);

// Registros Modbus
uint16_t modbusRegisters[10];

// Variables de sensor
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 2000;  // 2 segundos

void setup() {
  // Inicializar comunicación Serial para Modbus (RS485)
  Serial.begin(9600);
  slave.begin(9600);
  
  // Inicializar sensor DHT
  dht.begin();
  
  // Inicializar registros
  initializeRegisters();
  
  // Configurar pin de LED integrado
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Mensaje de inicio
  Serial.println("Arduino Modbus RTU Slave iniciado");
  Serial.println("Dirección: " + String(MODBUS_SLAVE_ID));
}

void loop() {
  // Procesar peticiones Modbus
  slave.poll(modbusRegisters, 10);
  
  // Leer sensores periódicamente
  readSensors();
  
  // Actualizar registros Modbus
  updateModbusRegisters();
  
  // Controlar LED según registro de control
  controlLED();
  
  delay(10);  // Pequeño delay para estabilidad
}

void initializeRegisters() {
  /**
   * Inicializar valores de registros Modbus
   * 
   * Registro 0: Temperatura (entero * 10)
   * Registro 1: Humedad (entero * 10) 
   * Registro 2: Contador de escrituras
   * Registro 3: Estado LED (0=apagado, 1=encendido)
   * Registro 4: Valor PWM (0-255)
   * Registros 5-9: Reservados
   */
  for(int i = 0; i < 10; i++) {
    modbusRegisters[i] = 0;
  }
}

void readSensors() {
  /**
   * Leer sensores cada SENSOR_READ_INTERVAL
   */
  unsigned long currentTime = millis();
  
  if(currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    // Leer temperatura y humedad
    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();
    
    // Validar lecturas
    if(!isnan(newTemp)) {
      temperature = newTemp;
    }
    if(!isnan(newHum)) {
      humidity = newHum;
    }
    
    lastSensorRead = currentTime;
    
    // Debug por serial
    Serial.print("T: ");
    Serial.print(temperature);
    Serial.print("°C, H: ");
    Serial.print(humidity);
    Serial.println("%");
  }
}

void updateModbusRegisters() {
  /**
   * Actualizar registros Modbus con datos actuales
   */
  
  // Temperatura (registro 0) - multiplicar por 10 para mantener 1 decimal
  modbusRegisters[0] = (uint16_t)(temperature * 10);
  
  // Humedad (registro 1) - multiplicar por 10 para mantener 1 decimal
  modbusRegisters[1] = (uint16_t)(humidity * 10);
  
  // Incrementar contador de ciclos (registro 2)
  static unsigned long cycleCount = 0;
  modbusRegisters[2] = cycleCount++;
  if(cycleCount > 65535) cycleCount = 0;
}

void controlLED() {
  /**
   * Controlar LED según registro de control
   */
  if(modbusRegisters[3] == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  // Control PWM si está configurado
  if(modbusRegisters[4] <= 255) {
    analogWrite(3, modbusRegisters[4]);  // Pin 3 PWM
  }
}

// Función para debug (opcional)
void printRegisterStatus() {
  Serial.println("=== Estado Registros Modbus ===");
  for(int i = 0; i < 5; i++) {
    Serial.print("Registro ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(modbusRegisters[i]);
  }
  Serial.println("===============================");
}


---



 Punto 3: Programación PLC - Implementación Profesional

Instalación TIA Portal V17 + PLCSIM

 Requisitos del Sistema

Windows 10/11 Pro

16 GB RAM mínimo

SSD 256 GB+

DirectX 11

Resolución 1080p


Fase 1: Preparación

1. Desactivar antivirus


2. Ejecutar como administrador


3. Verificar .NET 4.8


4. Descargar instaladores oficiales



Fase 2: Instalación

1. Ejecutar Start.exe
2. Seleccionar TIA Portal V17
3. Idiomas: EN, DE, ES
4. Instalar:
   - Portal Professional
   - PLCSIM
   - Startdrive
   - Safety Advanced
   - Energy Suite
5. Ruta: C:\Program Files\Siemens\Automation\

Fase 3: Post-Instalación

Licencias

Configuración de updates

Instalación HSPs



---

📁 Estructura del Proyecto

Control_Lampara_Industrial/
├── Device/
│   ├── PLC_1 [CPU 1212C]
│   ├── HMI_1 [KTP700]
│   └── Network/
├── PLC_1/
│   ├── OB1
│   ├── FC1
│   ├── FC2
│   ├── DB1
│   └── DB2
└── HMI_1/
    ├── Screens/
    ├── Templates/
    └── Alarmes/


---

 Configuración del Hardware

CPU: 1212C AC/DC/RLY

Firmware: V4.5

IP: 192.168.0.10/24

Módulos DI/DQ especificados



---
 Programación en Ladder

DB1 – Datos Lámpara

STRUCT
    EstadoLámpara : Bool;
    TiempoEncendido : Time;
    ContadorCiclos : Int;
    AlarmaSobreTemperatura : Bool;
    ModoOperacion : Byte;
END_STRUCT

FC1 – Control de Lámpara

// Network 1: Control Manual/Automático
|---| |---| |---|/|----( )---|

// Network 2: Modo Manual
|---| |---|/|----( )---|

// Network 3: Control Automático

