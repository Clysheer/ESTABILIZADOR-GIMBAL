# ESTABILIZADOR-GIMBAL

Proyecto de gimbal de tres ejes con Arduino Nano, IMU MPU6050 y servos SG90. Incluye PCB diseñada, firmware en C y estructura 3D en PLA. Estabiliza una cámara ligera compensando ±45° con un error menor a 3°, validando el control PID implementado.

## 📋 Introducción

El objetivo principal de este trabajo fue diseñar y construir un gimbal electrónico de tres ejes capaz de mantener una cámara estable, compensando los movimientos involuntarios del usuario. Los gimbals son ampliamente utilizados en drones, cámaras portátiles y equipos de filmación profesional, debido a su capacidad para estabilizar la imagen incluso en condiciones dinámicas.

Este proyecto permitió integrar múltiples conocimientos adquiridos a lo largo de la tecnicatura: electrónica analógica y digital, diseño de circuitos impresos (PCB), programación de microcontroladores, control automático y procesamiento de señales de sensores.

### 🎯 Objetivos específicos

- Diseñar y construir el circuito de control de los motores
- Adquirir y procesar datos del sensor inercial MPU6050 mediante I2C
- Implementar algoritmo de control PID para mantener posición estable en tres ejes
- Diseñar, modelar e imprimir en 3D la estructura mecánica
- Integrar hardware, software y mecánica en un sistema funcional

## 🔧 Descripción del Proyecto

### ¿Qué es un Gimbal?

Un gimbal es un sistema de suspensión que permite mantener un objeto (como una cámara) estable y nivelado mientras se mueve. Utiliza motores o servomotores para contrarrestar movimientos no deseados y estabilizar el objeto en los tres ejes principales: **Yaw** (giro horizontal), **Pitch** (inclinación vertical) y **Roll** (rotación lateral).

### Componentes Principales

| Componente | Descripción |
|------------|-------------|
| **Servos SG90** | 3 servomotores para control de movimiento en tres ejes |
| **MPU6050** | Sensor IMU (acelerómetro + giroscopio 3 ejes) |
| **Arduino Nano** | Microcontrolador para procesamiento y control |
| **Step-up LM2596** | Convertidor DC-DC para alimentación (3.7V → 5V) |
| **Batería LiPo** | Fuente de energía 3.7V |
| **Estructura 3D** | Diseño en PLA para soporte mecánico |

### Control PID

El control PID (Proporcional, Integral, Derivativo) se implementa para mantener la cámara en posición estable:

- **P (Proporcional)**: Corrige el error instantáneo
- **I (Integral)**: Elimina error acumulado en el tiempo
- **D (Derivativo)**: Reduce oscilaciones y mejora estabilidad

## 💻 Software

### Arquitectura del Software

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   MPU6050   │    │   Arduino   │    │   Servos    │
│   Sensor    │───▶│   Nano      │───▶│   SG90      │
│             │    │             │    │             │
└─────────────┘    └─────────────┘    └─────────────┘
```

### Flujo de Funcionamiento

1. **Inicialización del sistema**
2. **Lectura del sensor MPU6050**
3. **Filtrado y fusión de datos** (filtro complementario)
4. **Cálculo de ángulos** (Roll, Pitch, Yaw)
5. **Control PID** para cada eje
6. **Generación de señales PWM** para servomotores
7. **Bucle principal** en tiempo real (~100 Hz)

### Código Principal

```cpp
#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;
Servo servoRoll, servoPitch, servoYaw;

// Variables PID
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float errorRoll, errorPitch, errorYaw;
float integralRoll = 0, integralPitch = 0, integralYaw = 0;

void setup() {
    // Inicialización de componentes
    Wire.begin();
    mpu.initialize();
    
    // Calibración del giroscopio
    calibrateGyro();
    
    // Configuración de servos
    servoRoll.attach(9);
    servoPitch.attach(10);
    servoYaw.attach(11);
}

void loop() {
    // Lectura de sensores
    readMPU6050();
    
    // Filtrado complementario
    complementaryFilter();
    
    // Cálculo PID
    calculatePID();
    
    // Control de servos
    updateServos();
    
    delay(10); // ~100 Hz
}
```

## 🔌 Hardware

### Diseño Electrónico

**Esquema de conexiones:**

- **MPU6050**: SDA → A4, SCL → A5, VCC → 3.3V/5V, GND → GND
- **Servos**: Señal → pines PWM (D3, D5, D6), Vcc → 5V, GND → GND
- **Alimentación**: Batería → Step-up → 5V rail

![Esquemático PCB](https://github.com/user-attachments/assets/433663b6-8bd0-4409-8fea-0d0e43cc9d39)

### Especificaciones Técnicas

| Parámetro | Especificación |
|-----------|----------------|
| **Ejes** | 3 (Yaw, Pitch, Roll) |
| **Rango de corrección** | ±45° por eje |
| **Error de estabilización** | < 3° |
| **Latencia de control** | < 20 ms |
| **Tasa de muestreo IMU** | 200-500 Hz |
| **Alimentación** | LiPo 3.7V → Step-up 5V |
| **Consumo** | 500 mA (reposo) - 2A (máximo) |

## 🏗️ Diseño Mecánico

### Estructura 3D en Fusion 360

**Parámetros de diseño:**
- Espesor mínimo de paredes: 2.5-3 mm
- Holgura para encajes: 0.2-0.4 mm
- Bosses para tuercas M3

**Configuración de impresión (PLA):**
- Layer height: 0.20 mm
- Infill: 20% (gyroid para rigidez)
- Perímetro: 3 shells
- Temperatura: 200-210°C

### Proceso de Construcción

1. **Montaje de alimentación** - Step-up y batería
2. **Comunicación IMU** - MPU6050 y I2C
3. **Control y firmware** - Arduino y programación básica
4. **Actuadores y PID** - Servos y algoritmo de control
5. **Integración mecánico-eléctrica** - Ensamblaje final
6. **Calibración y pruebas** - Ajuste y validación

![Diseño Final](https://github.com/user-attachments/assets/c950accb-c8f7-47d4-8324-6bb3b229b0f3)

## 📊 Planificación

### Diagrama de Gantt

![Diagrama de Gantt](https://github.com/user-attachments/assets/ce0098a0-5fe5-4a19-bb03-d41b003178d2)

### Lista de Componentes y Costos

| Componente | Cantidad | Precio Unitario | Total |
|------------|----------|-----------------|-------|
| Arduino Nano | 1 | $8.000 | $8.000 |
| MPU6050 | 1 | $4.500 | $4.500 |
| Servomotores SG90 | 3 | $2.300 | $6.900 |
| Step-up LM2596 | 1 | $2.900 | $2.900 |
| Batería LiPo 3.7V | 1 | $3.500 | $3.500 |
| **Total estimado** | | | **$27.000 - $30.000** |

## ✅ Conclusiones

El desarrollo del gimbal electrónico de tres ejes permitió integrar conocimientos de electrónica, programación, control, diseño 3D y fabricación de PCBs, culminando en un sistema funcional capaz de estabilizar una cámara ligera.

**Logros principales:**
- Sistema de control PID implementado exitosamente
- Estructura mecánica diseñada y fabricada en PLA
- PCB personalizada que optimiza el cableado
- Error de estabilización menor a 3° alcanzado

## 🚀 Mejoras Futuras

1. **ESP32** - Mayor poder de procesamiento y conectividad WiFi/Bluetooth
2. **Motores brushless** - Para mayor torque y suavidad
3. **Filtros avanzados** - Kalman o Madgwick para mejor estimación
4. **Rediseño estructural** - Optimización de peso y rigidez
5. **Batería de mayor capacidad** - Con sistema de carga integrado
6. **Modos de funcionamiento** - Lock, follow, panorámico
7. **Cámaras más pesadas** - GoPro o cámaras compactas

## 📖 Manual de Usuario

### Puesta en Marcha

1. Conectar la batería al sistema
2. Verificar LED de alimentación del Arduino
3. Esperar 2-3 segundos para inicialización del MPU6050
4. Mantener el gimbal quieto durante calibración automática

### Operación

- Movimientos suaves para mejor estabilización
- Mantener cámara centrada en el soporte
- No exceder peso recomendado

### Solución de Problemas

| Problema | Causa Probable | Solución |
|----------|----------------|----------|
| Vibración excesiva | Ganancias PID incorrectas | Ajustar Kp, Ki, Kd |
| Inclinación lateral | Calibración incorrecta | Reiniciar en superficie estable |
| Sin alimentación | Batería o step-up | Verificar conexiones y voltajes |

## 📚 Bibliografía

- Arduino Nano Documentation - Arduino.cc
- MPU6050 Datasheet - Bosch Sensortec
- KiCad EDA Documentation - kicad.org
- SG90 Technical Specifications - TowerPro
- PID Theory Explained - National Instruments
- Fusion 360 User Manual - Autodesk

---

**Repositorio creado para el proyecto de Gimbal Electrónico de 3 Ejes**  
*Integrando electrónica, programación y diseño mecánico para estabilización de cámaras*
