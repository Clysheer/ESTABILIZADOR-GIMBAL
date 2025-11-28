# ESTABILIZADOR-GIMBAL
Proyecto de gimbal de tres ejes con Arduino Nano, IMU MPU6050 y servos SG90. Incluye PCB diseñada, firmware en C y estructura 3D en PLA. Estabiliza una cámara ligera compensando ±45° con un error menor a 3°, validando el control PID implementado.
**Introducción**
El objetivo principal de este trabajo fue diseñar y construir un gimbal electrónico de tres ejes capaz de mantener una cámara estable, compensando los movimientos involuntarios del usuario. Los gimbals son ampliamente utilizados en drones, cámaras portátiles y equipos de filmación profesional, debido a su capacidad para estabilizar la imagen incluso en condiciones dinámicas. La motivación para desarrollar este proyecto surgió de la necesidad creciente de capturar tomas fluidas y sin vibraciones, especialmente en aplicaciones audiovisuales y sistemas aéreos no tripulados.
El desarrollo del gimbal permitió integrar y aplicar múltiples conocimientos adquiridos a lo largo de la tecnicatura, tales como electrónica analógica y digital, diseño de circuitos impresos (PCB), programación de microcontroladores, control automático y procesamiento de señales provenientes de sensores. Este enfoque interdisciplinario hizo posible comprender de manera global cómo distintos sistemas electrónicos pueden trabajar en conjunto para resolver un problema real.
Desde un punto de vista teórico, la construcción del gimbal implicó estudiar conceptos fundamentales de análisis y control de sistemas dinámicos. Para lograr una estabilización eficiente fue necesario comprender el funcionamiento de sensores inerciales (IMU), específicamente el MPU6050, que combina un acelerómetro y un giroscopio de 3 ejes. Estos sensores permiten obtener información sobre la aceleración y la velocidad angular del dispositivo, datos que deben ser filtrados y procesados para estimar correctamente la orientación de la cámara. Se analizaron filtros complementarios y técnicas básicas de fusión sensorial para mejorar la precisión y reducir el ruido de las mediciones.
Otro aspecto central del estudio fue el control PID (Proporcional, Integral y Derivativo), un algoritmo clásico en sistemas de control realimentados. Fue necesario comprender su funcionamiento, las funciones de cada uno de sus términos y cómo su ajuste influye directamente en la estabilidad y velocidad de respuesta del sistema. Se realizaron análisis teóricos sobre estabilidad, amortiguamiento, respuesta al escalón y efectos de la sintonización de parámetros Kp, Ki y Kd. Este controlador permitió corregir en tiempo real la diferencia entre la posición deseada y la posición medida, generando las señales necesarias para ajustar los ángulos de los servomotores.
Asimismo, se estudiaron los principios de funcionamiento de los servomotores, particularmente su control mediante modulación por ancho de pulso (PWM). Se analizaron sus limitaciones mecánicas y eléctricas, torque disponible, velocidad de respuesta y resolución angular. También se evaluaron distintos configuraciones mecánicas y de soporte para garantizar la correcta transmisión del movimiento.
La estructura mecánica del gimbal se diseñó mediante software CAD y luego fue fabricada mediante impresión 3D en PLA. Esto permitió estudiar conceptos de diseño mecánico, peso, distribución de masa, centro de gravedad y rigidez estructural, factores claves para garantizar que el sistema pueda estabilizarse correctamente sin introducir vibraciones.En conjunto, este proyecto representó una integración completa de teoría y práctica, abarcando áreas como control automático, electrónica digital, diseño de hardware, programación embebida y fabricación mecánica. El resultado final fue un sistema funcional capaz de estabilizar una cámara ligera, demostrando la importancia de la combinación entre conocimientos teóricos y aplicaciones prácticas.
Objetivos específicos
●   Diseñar y construir el circuito de control de los motores, integrando los servomotores y la electrónica necesaria para su funcionamiento.
●   Adquirir y procesar los datos del sensor inercial MPU6050 mediante comunicación I2C.
●   Implementar un algoritmo de control PID capaz de mantener la posición estable en los tres ejes.
●   Diseñar, modelar e imprimir en 3D la estructura mecánica que sostiene la cámara y los motores.
●   Integrar hardware, software y mecánica en un único sistema funcional.

Se realizaron ajustes para poder tener los 3 servos bien colocados
**Descripción del proyecto**
**¿Qué es un Gimbal?**
Un gimbal es un sistema de suspensión que permite mantener un objeto (como una cámara) estable y nivelado mientras se mueve. Utiliza motores o servomotores para contrarrestar los movimientos no deseados y estabilizar el objeto en cuestión. En este caso, el gimbal de 3 ejes se utiliza para estabilizar una cámara pequeña o dispositivo similar durante los movimientos en los tres ejes principales: Yaw (giro horizontal), Pitch (inclinación vertical) y Roll (rotación lateral).
En lugar de usar un sistema mecánico pesado, un gimbal moderno utiliza sensores electrónicos y motores controlados electrónicamente para realizar ajustes en tiempo real. Esto permite una estabilización más precisa y dinámica, ideal para aplicaciones como la grabación de videos o fotografía aérea, donde la estabilidad es crucial.
**Componentes principales del Gimbal**
Servos
En este proyecto, se usan tres servomotores para controlar el movimiento de la cámara en los tres ejes. Los servos son motores de corriente continua con un mecanismo de control que permite ajustar su ángulo de forma precisa. Cada servo se controla mediante una señal de modulación por ancho de pulso (PWM), donde el ancho del pulso determina el ángulo del servo.
Los servos tienen la ventaja de ser fáciles de controlar mediante una señal PWM, lo que los hace ideales para proyectos como este, donde se requiere un control preciso de los movimientos. Sin embargo, los servos también tienen limitaciones en cuanto a la velocidad y torque, lo que debe ser considerado al diseñar el gimbal.
MPU6050 (Sensor IMU)
El MPU6050 es un sensor de movimiento que combina un acelerómetro de 3 ejes y un giroscopio de 3 ejes. Este sensor es capaz de medir la aceleración y la velocidad angular en los tres ejes del espacio (X, Y y Z). Se utiliza para detectar el movimiento de la cámara y proporcionar información precisa sobre su orientación en el espacio.
●   Acelerómetro: mide la aceleración en los tres ejes, lo que ayuda a detectar la inclinación de la cámara con respecto a la gravedad.
●   Giroscopio: mide la velocidad angular, es decir, la rotación de la cámara en cada uno de los ejes.La información de estos dos sensores se procesa para calcular la orientación actual de la cámara. La combinación de ambos datos (acelerómetro y giroscopio) permite tener una estimación más precisa del movimiento, ya que cada sensor compensa las limitaciones del otro.
En el proyecto, el MPU6050 se conecta al Arduino a través del protocolo I2C, lo que permite transmitir los datos del sensor con solo dos cables: uno para la señal de reloj (SCL) y otro para la señal de datos (SDA). Estos datos se usan para calcular el ángulo de inclinación de la cámara y se pasan al algoritmo de control PID que ajusta los servos para mantener la estabilización.
Arduino
El Arduino es el cerebro del sistema, responsable de recibir los datos del sensor MPU6050 y procesarlos para calcular los ajustes que deben realizarse en los servos. El Arduino, en este caso, actúa como un controlador en tiempo real que implementa un algoritmo de control PID para mantener los ángulos de la cámara estables.
El microcontrolador Arduino UNO se usa por su simplicidad, facilidad de programación y la amplia cantidad de librerías disponibles, que simplifican el trabajo con el sensor MPU6050 y el control de los servos. A través de las señales PWM generadas por el Arduino, se controla el ángulo de cada servo, asegurando que la cámara mantenga la orientación deseada en cada momento.
Step-up Converter (Ajuste de voltaje)
Para alimentar la placa del Arduino y los servos, se utiliza un step-up converter. Dado que los servos y el Arduino requieren una tensión de 5V, pero el sistema está alimentado por una batería Li-Po de 3.7V, es necesario usar un step-up para elevar la tensión de la batería hasta los 5V necesarios para el funcionamiento del sistema.
Un step-up converter (convertidor elevador) es un tipo de fuente de alimentación DC-DC que convierte una entrada de baja tensión (en este caso, 3.7V) a una salida de mayor tensión (5V). Esto es crucial, ya que los servos y el Arduino no funcionarían correctamente con 3.7V, ya que ambos requieren una tensión constante de 5V. El step-up también asegura que la energía se entregue de forma eficiente, sin perder demasiada potencia.
**Control PID**
El control PID (Proporcional, Integral, Derivativo) es un algoritmo que se utiliza para mantener la cámara en una posición estable. Este tipo de control en lazo cerrado ajusta continuamente la salida (en este caso, el movimiento de los servos) en función de la diferencia entre el valor actual y el valor deseado.
●   Proporcional (P): Compara la diferencia (error) entre el ángulo actual y el deseado. Cuanto mayor es el error, mayor será la corrección aplicada.●   Integral (I): Suma el error a lo largo del tiempo, lo que ayuda a corregir pequeños desajustes acumulados.
●   Derivativo (D): Analiza la tasa de cambio del error, ayudando a reducir oscilaciones y hacer que el sistema sea más estable.
El objetivo del PID es ajustar los servos en tiempo real para que la cámara mantenga la orientación correcta, minimizando el error de estabilización. Los parámetros Kp, Ki y Kd se ajustan mediante pruebas para lograr una respuesta estable y rápida.
**Funcionamiento del Gimbal**
El proceso de estabilización del gimbal funciona de la siguiente manera:
1.  Lectura del sensor: El MPU6050 mide constantemente la aceleración y la velocidad angular de la cámara.
2.  Cálculo de los ángulos: Los datos del sensor se procesan en el Arduino para calcular el ángulo de inclinación en los tres ejes.
3.  Control PID: El algoritmo PID calcula el error entre el ángulo actual y el deseado (normalmente 0° en todos los ejes).
4.  Ajuste de los servos: El Arduino ajusta los servos mediante señales PWM para contrarrestar cualquier movimiento no deseado y estabilizar la cámara.
**Limitaciones y Mejoras**
El uso de servos para este tipo de proyecto tiene algunas limitaciones, como el torque limitado y la velocidad de respuesta. Si los movimientos son muy rápidos o fuertes, los servos podrían no ser lo suficientemente rápidos para corregir la orientación a tiempo. Sin embargo, para cámaras pequeñas y movimientos suaves, los servos proporcionan una solución económica y fácil de implementar.
Como posible mejora, se podría usar motores brushless o stepper motors en lugar de servos, ya que ofrecen mayor torque y velocidad. Sin embargo, esto complicaría el diseño y el control del sistema.
**Lista de componentes y precios estimados**
●  Arduino Nano - 8000
●  MPU6050 - 4500
●  3 Servomotores -2300 c/u
●  Step-up LM2596 - 2900
●  4 JST - 200 c/u
●  Borneras - 100 c/u
●  Pineras hembra - 1800
● Filamento PLA para diseño 3D - 19.000 x kg (50 grs usados)
●  Tornillos - 500
●  Cables de interconexión - 2000
●  Batería de litio 3,7v - 3500
●  Precio estimado: 27.000 a 30.000
**SOFTWARE**
El software desarrollado para el gimbal electrónico de 3 ejes se encarga de leer los datos del sensor inercial MPU6050, procesarlos mediante algoritmos de filtrado, calcular los ángulos de orientación y ejecutar un control PID para ajustar la posición de los servomotores SG90 en tiempo real. El objetivo del software es garantizar que la cámara permanezca estable aun cuando el usuario realice movimientos bruscos.
El firmware se implementó en lenguaje C/C++ utilizando el entorno de Arduino IDE y emplea librerías específicas para el manejo del sensor y los servos. Toda la lógica del control está optimizada para funcionar en un microcontrolador de recursos limitados como el Arduino Nano.
Arquitectura general del software
El software se divide en los siguientes módulos principales:
1.  Inicialización del sistema
2.  Lectura del sensor MPU 6050 (acelerómetro + giroscopio)
3.  Procesamiento y filtrado de datos (filtro complementario)
4.  Cálculo de los ángulos de orientación (Roll, Pitch, Yaw)
5.  Control PID para cada eje
6.  Generación de señales PWM para los servomotores
7.  Bucle principal (loop) en tiempo real
<img width="255" height="645" alt="image" src="https://github.com/user-attachments/assets/1b4291a9-5131-499f-83e9-1309a1407efe" />
**3. Descripción detallada de cada bloque**
**3.1. Inicialización del sistema**
En esta etapa se configuran:●  La comunicación I2C (Wire.h)
●  La velocidad de muestreo del MPU6050
●  Los pines y librerías de los servos
●  Variables internas del PID
●  Temporizadores para cálculos en tiempo real
El sistema queda listo para funcionar dentro del loop principal.
**3.2. Lectura del sensor MPU6050**
El sensor proporciona:
●  Aceleración (Ax, Ay, Az)
●  Velocidad angular (Gx, Gy, Gz)
La lectura se realiza a través de I2C y se normaliza a unidades físicas:
●  Acelerómetro → g (9.81 m/s²)
●  Giroscopio → °/s (grados por segundo)
Estas señales suelen contener ruido, por eso necesitan filtrado.
**3.3. Filtrado y Fusión de Datos**
Para obtener una estimación precisa de los ángulos se utiliza un filtro complementario, que combina:
●  Los datos estables del acelerómetro●  La rapidez y suavidad del giroscopio
El filtro realiza:
Ángulo = 0.98*(Ángulo + gyro*dt) + 0.02*(Accel)
Este método reduce vibraciones y evita errores acumulados.
**3.4. Cálculo de Ángulos (Roll, Pitch y Yaw)**
Del acelerómetro se obtienen roll y pitch mediante trigonometría:
●  Roll = atan2(Ay, Az)
●  Pitch = atan2(-Ax, sqrt(Ay² + Az²))
Yaw se obtiene integrando el giroscopio Gz.
Cada ángulo se estabiliza mediante el filtro complementario.
**3.5. Control PID**
El PID compara la posición actual (sensor) con la deseada (0° para estabilización).
Para cada eje se calcula:
●  Error: diferencia entre ángulo deseado y ángulo real
●  P: corrige el error instantáneo
●  I: acumula errores pequeños
●  D: suaviza y evita oscilacionesEl PID entrega un valor corregido que indica cuánto debe moverse cada servo.
**3.6. Señales PWM a los Servomotores**
Arduino genera pulsos PWM con:
●  Frecuencia: 50 Hz
●  Ancho del pulso: 500–2500 µs
El ancho del pulso define la posición del servo.
Ejemplo:
●  1000 µs → -45º
●  1500 µs → 0º
●  2000 µs → +45º
**3.7. Loop principal**
Se ejecuta unas 200 a 400 veces por segundo, realizando: 1. Lectura MPU6050
2. Filtrado de señales
3. PID
4. PWM
5. Estabilización
Esto permite una respuesta rápida del gimbal.
**4. Flujo de funcionamiento del software (texto para informe)**
1. El sistema inicia y configura todos los módulos.
2. El microcontrolador lee las mediciones del MPU6050.
3. Los datos se filtran para obtener ángulos limpios. 4. Se calcula la orientación de la cámara.
5. El PID determina cuánto corregir la posición.
6. Los servos se ajustan mediante PWM.
7. El proceso se repite en tiempo real.
**Codigo PID**
#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;
Servo servoRoll, servoPitch, servoYaw;
// Ángulos filtrados
float roll = 0, pitch = 0, yaw = 0;
// Offsets del giroscopio (se calibran en setup)
float gyroXoffset = 0, gyroYoffset = 0, gyroZoffset = 0;
// Setpoints (posición deseada del gimbal) float setpointRoll = 0;
float setpointPitch = 0; float setpointYaw = 0; // Variables PIDfloat errorRoll, errorPitch, errorYaw;
float prevErrorRoll = 0, prevErrorPitch = 0, prevErrorYaw = 0;
float integralRoll = 0, integralPitch = 0, integralYaw = 0;
// Ganancias PID (ajustar según pruebas)
float Kp = 2.0;
float Ki = 0.5;
float Kd = 1.0;
unsigned long lastTime = 0;
float dt = 0.01;
// Factor del filtro complementario (0.98 → giroscopio domina, 0.02 → acelerómetro corrige) const float alpha = 0.98;
void setup() {
Serial.begin(9600);
Wire.begin();
// Inicializar MPU6050
Serial.println("Inicializando MPU6050...");
mpu.initialize();
if (!mpu.testConnection()) {
Serial.println("Error: no se detecta MPU6050!"); while (1);
}
// Calibrar giroscopio (promedio en reposo) Serial.println("Calibrando giroscopio...");
long gxSum = 0, gySum = 0, gzSum = 0; for (int i = 0; i < 2000; i++) {
int16_t ax, ay, az, gx, gy, gz;
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); gxSum += gx;
gySum += gy;
gzSum += gz;
delay(2);
}
gyroXoffset = gxSum / 2000.0; gyroYoffset = gySum / 2000.0; gyroZoffset = gzSum / 2000.0;
Serial.println("Calibración completa.");
// Servos
servoRoll.attach(9); servoPitch.attach(10); servoYaw.attach(11);servoRoll.write(90);
servoPitch.write(90);
servoYaw.write(90);
lastTime = millis();
}
void loop() {
unsigned long now = millis();
dt = (now - lastTime) / 1000.0;
if (dt <= 0) dt = 0.01;
lastTime = now;
// Leer datos del MPU6050
int16_t ax, ay, az, gx, gy, gz;
mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
// Convertir giroscopio a º/s (sensibilidad: 131 LSB/(º/s))
float gyroXrate = (gx - gyroXoffset) / 131.0;
float gyroYrate = (gy - gyroYoffset) / 131.0;
float gyroZrate = (gz - gyroZoffset) / 131.0;
// Calcular ángulos con acelerómetro
float accPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
float accRoll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
// Filtro complementario
pitch = alpha * (pitch + gyroXrate * dt) + (1 - alpha) * accPitch;
roll  = alpha * (roll - gyroYrate * dt) + (1 - alpha) * accRoll;
yaw += gyroZrate * dt; // yaw solo con giroscopio (deriva un poco con el tiempo)
// --- PID Roll ---
errorRoll = setpointRoll - roll;
integralRoll += errorRoll * dt;
float derivativeRoll = (errorRoll - prevErrorRoll) / dt;
float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll; prevErrorRoll = errorRoll;
// --- PID Pitch ---
errorPitch = setpointPitch - pitch;
integralPitch += errorPitch * dt;
float derivativePitch = (errorPitch - prevErrorPitch) / dt;
float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch; prevErrorPitch = errorPitch;
// --- PID Yaw ---
errorYaw = setpointYaw - yaw; integralYaw += errorYaw * dt;float derivativeYaw = (errorYaw - prevErrorYaw) / dt;
float outputYaw = Kp * errorYaw + Ki * integralYaw + Kd * derivativeYaw; prevErrorYaw = errorYaw;
// Convertir a servo (90 = centro)
int servoRollVal  = constrain(90 + outputRoll, 0, 180); int servoPitchVal = constrain(90 + outputPitch, 0, 180); int servoYawVal   = constrain(90 + outputYaw, 0, 180);
servoRoll.write(servoRollVal); servoPitch.write(servoPitchVal); servoYaw.write(servoYawVal);
// Debug
Serial.print("Roll: "); Serial.print(roll);
Serial.print(" | Pitch: "); Serial.print(pitch); Serial.print(" | Yaw: "); Serial.print(yaw); Serial.print(" | S_R: "); Serial.print(servoRollVal); Serial.print(" | S_P: "); Serial.print(servoPitchVal); Serial.print(" | S_Y: "); Serial.println(servoYawVal); delay(10); // ~100 Hz
}
**Hardware**
**DISEÑO ELECTRÓNICO Y ESQUEMÁTICO (MÓDULOS Y CIRCUITO PRINCIPAL)**
Objetivo de la etapa: definir la arquitectura eléctrica del gimbal y generar el esquemático que interconecta MCU, IMU, drivers de potencia, alimentación y servos.
Módulos utilizados
●   Microcontrolador: Arduino Nano (ATmega328P)
●   IMU: MPU6050 (Acelerómetro + Giroscopio 3-ejes)
●   Actuadores: 3 × Servomotores SG90 (PWM)
●   Regulación: Step-up DC-DC (LM2596 o equivalente)
●   Alimentación: Batería LiPo 1S (3.7 V)
●   Conectores: JST 3/4 pines, pines hembra 2.54 mm
●   Indicadores: LED de alimentación y pulsadores de calibración (opcional)
Esquema eléctrico (descripción para replicar)
●   V_BAT → STEPUP_IN
●   STEPUP_OUT 5V → RAIL_5V (alimentación para Arduino y servos)
●   GND común a batería, step-up, servos y Arduino
●   Arduino (5V) alimenta: VIN o 5V según modelo (en Nano usar pin 5V)
● MPU6050: SDA → A4 (I2C SDA), SCL → A5 (I2C SCL), VCC → 3.3V o 5V (según breakout) – usar 3.3V si es breakout sin regulador; comprobar tolerancias. GND → GND
●   Servos: señal → pines PWM del Arduino (ej: D3, D5, D6), Vcc → RAIL_5V, GND → GND
●   Condensadores de desacople: 100µF en entrada y salida del step-up, 0.1µF cerámico cerca del MCU y del IMU
●   Fusible polímero o PTC en línea de batería (recomendado)Aplicación: el esquema se usa para fabricar la PCB que interconecta todos los módulos con vías limpias, anclajes mecánicos para el ensamblado y header para conexión de la estructura 3D.
<img width="879" height="411" alt="image" src="https://github.com/user-attachments/assets/433663b6-8bd0-4409-8fea-0d0e43cc9d39" />
**DESCRIPCIÓN DE CADA ETAPA DE CONSTRUCCIÓN**
Etapa A — Montaje y prueba de la alimentación
●   Módulo: Step-up LM2596, fusible PTC, conector batería.
●   Schematic focal: entrada batería → PTC → condensadores → LM2596 → 5V rail → protecciones.
●   Pruebas: medir 5V sin carga y con servos conectados; verificar rizado < 100 mV.
●   Aplicación: suministrar energía estable a servos y MCU.
Etapa B — Comunicación y sensor IMU
●   Módulo: MPU6050 (I2C).
●   Schematic focal: pull-ups I2C, decoupling, y test pads.
●   Pruebas: leer registros WHO_AM_I, comprobar aceleraciones en reposo.●   Aplicación: proveer mediciones de aceleración y velocidad angular.
Etapa C — Control y firmware básico
●   Módulo: Arduino Nano + firmware base (I2C, lectura IMU, serial).
●   Schematic focal: pines de programación, led de status.
●   Pruebas: mostrar datos por Serial, verificar lecturas estables.
●   Aplicación: plataforma de prueba para algoritmos.
Etapa D — Actuadores y control PID
●   Módulo: 3× SG90, headers de servo, filtros de alimentación.
●   Schematic focal: ruta de alimentación de servos y pines PWM.
●   Pruebas: enviar ángulos de prueba, medir respuesta y tiempos.
●   Aplicación: ajuste de Kp, Ki, Kd y verificación de comportamiento.
Etapa E — Integración mecánico-eléctrica
●   Módulo: estructura 3D en PLA (soportes, anclajes de servos) y montaje final de PCB.
●   Schematic focal: puntos de anclaje, orificios M3, distancias.
●   Pruebas: montaje mecánico, balanceo de cargas, centrado de masas.
●   Aplicación: ver comportamiento dinámico real.
Etapa F — Calibración, pruebas finales y documentación
●   Módulo: software de calibración y parámetros finales, manual de usuario.
●   Schematic focal: puntos de ajuste (trim) y header de calibración.
●   Pruebas: medición de error final (<3°), endurance tests.
●   Aplicación: entrega del proyecto y repositorio.
**ESPECIFICACIONES TÉCNICAS (DETALLADAS)**
Especificaciones generales
●   Ejes: 3 (Yaw, Pitch, Roll)
●   Rango de corrección: ±45° (por eje)
●   Error de estabilización: < 3° (según pruebas)
●   Latencia de control objetivo: < 20 ms
●   Tasa de muestreo IMU: 200–500 Hz
●   Alimentación: 1S LiPo 3.7 V → Step-up a 5 V
●   Consumo estimado: 500 mA (reposo) – hasta 2 A con 3 servos en carga
Componentes críticos y parámetros
●   MPU6050: resolución ±2g/±250°/s (configurable)
●   Servos SG90: torque 1.6–2.2 kg·cm, velocidad ~0.1 s/60°
●   Step-up LM2596: efficiency ~80–90%, corriente máxima 3 A (con disipación adecuada)
●   MCU: ATmega328P @16 MHz, memoria suficiente para PID y filtros
<img width="899" height="1532" alt="image" src="https://github.com/user-attachments/assets/f7fc629e-27c9-45c5-8f98-e32e1fc26220" />
*DISEÑO 3D EN FUSION 360 (PARA LA ESTRUCTURA EN PLA)**
Objetivo: diseñar la estructura que aloje la cámara y los servos, permita montaje seguro y tenga puntos de anclaje para la PCB y la batería.
Flujo en Fusion 360
1.  Esbozo (Sketch): definir dimensiones base del soporte de cámara y ubicación de servos.
2.  Modelado: extruir y crear cortes, añadir nervaduras para rigidez.
3.  Soportes y anclajes: crear orificios M3 y bosses para inserción de tuercas empotradas.
4.  Verificación de interferencias: usar la herramienta de assemble para montar servos y cámara y verificar colisiones.
5.  Exportar: generar archivos STL por cada pieza y PDFs con dimensiones.
Parámetros de diseño recomendados
●   Espesor mínimo de paredes: 2.5–3 mm (para PLA estructural).
●   Bosses para tuercas: altura 3–4 mm, diámetro 2 mm mayor que el tornillo.
●   Holgura para encajes: 0.2–0.4 mm según precisión de impresora.
●   Zona de montaje de servos: ajuste de 0.5–1 mm si el servo se inserta a presión.
●   Anclajes para PCB: 4 puntos con separación
Configuraciones de impresión (PLA) sugeridas
●   Layer height: 0.20 mm
●   Infill: 20% (rectilinear o gyroid para rigidez)
●   Perímetro: 3 shells
●   Temp extrusor: 200–210 °C (según filamento)●   Temp cama: 50–60 °C
●   Velocidad: 40–60 mm/s
●   Soportes: donde haya voladizos > 45°
●   Post-procesado: lijado y, si es necesario, encolado con epoxi para uniones críticas
Pruebas mecánicas
●   Balanceo: colocar la cámara y medir centro de gravedad; ajustar contrapesos si hace falta
●   Rigidez: ensayo de vibración manual y comprobar resonancias con la IMU (picos en frecuencia)
**Diseño Montado y finalizado**
<img width="736" height="339" alt="image" src="https://github.com/user-attachments/assets/a3b45236-305d-488f-9a98-a2b382f607ed" />
**Proceso de diseño **
<img width="1599" height="899" alt="image" src="https://github.com/user-attachments/assets/e8c8b42c-d206-4bc8-97fd-f431d634465c" />
**Primer Protoripo**
<img width="705" height="1097" alt="image" src="https://github.com/user-attachments/assets/0819e72e-3f33-4f94-bd6e-0297b10f5c93" />]
**Diseño Final**
<img width="899" height="1599" alt="image" src="https://github.com/user-attachments/assets/c950accb-c8f7-47d4-8324-6bb3b229b0f3" />
**Diagrama de gantt, reparto de tareas y tiempos**
<img width="2048" height="805" alt="image" src="https://github.com/user-attachments/assets/ce0098a0-5fe5-4a19-bb03-d41b003178d2" />
**Conclusiones**
El desarrollo del gimbal electrónico de tres ejes permitió integrar conocimientos de electrónica, programación, control, diseño 3D y fabricación de PCBs, culminando en un sistema funcional capaz de estabilizar una cámara ligera frente a movimientos no deseados. A lo largo del proyecto se logró implementar un sistema de control basado en un algoritmo PID, utilizando un sensor inercial MPU6050 y servomotores SG90, todo gestionado por un microcontrolador Arduino Nano.
La estructura mecánica se diseñó íntegramente en Fusion 360 y se imprimió en PLA, logrando un soporte liviano, rígido y adecuado para alojar los motores y la cámara. Por otro lado, el diseño y fabricación de la PCB en KiCad permitió reducir el cableado, mejorar la robustez del sistema y optimizar la distribución de cada componente electrónico.
Las pruebas realizadas mostraron que el sistema puede estabilizar la cámara con un error angular reducido, incluso frente a perturbaciones suaves y movimientos moderados. Si bien los servomotores tienen limitaciones de torque y velocidad, el desempeño general fue satisfactorio para las condiciones planteadas en este proyecto.
Este trabajo permitió consolidar habilidades adquiridas durante la tecnicatura, especialmente en control, sensores, programación, diseño CAD/CAE y prototipado electrónico. Asimismo, evidenció la importancia de integrar correctamente la mecánica, la electrónica y el software para obtener un sistema mecatrónico funcional y estable.
**Posibles mejoras y líneas de trabajo futuras**
Si bien el gimbal cumple con los objetivos iniciales, existen varias mejoras que podrían implementarse para obtener un sistema más robusto, preciso y profesional:
1. Reemplazo del Arduino Nano por un ESP32
El ESP32 permitiría ampliar significativamente las capacidades del sistema:
●   Mayor poder de procesamiento para ejecutar filtros más avanzados (Kalman, Madgwick).
●   Control de motores más preciso gracias a PWM de alta resolución.
●   Conectividad WiFi y Bluetooth para control remoto de la cámara.
●   Posibilidad de transmisión de datos, telemetría o configuración desde una app.Además, el mayor rendimiento del ESP32 permitiría implementar estabilización en tiempo real con menor latencia.
2. Uso de motores brushless con drivers gimbal (BGC)
Los servos SG90 son económicos pero limitados. Cambiar a motores brushless específicos para gimbal permitiría:
●   Movimientos más suaves y continuos.
●   Mayor torque disponible.
●   Mayor velocidad de respuesta.
●   Ausencia de vibraciones características del servo.
Con esto se lograría una estabilización semi-profesional comparable a la de los gimbals comerciales.
3. Implementación de un filtro de fusión de sensores avanzado
En lugar del filtrado simple basado en el MPU6050, se podría agregar:
●   Filtro de Kalman
●   Filtro Madgwick o Mahony
Esto mejoraría la precisión en la estimación de los ángulos, sobre todo en presencia de vibraciones o cambios bruscos.
4. Rediseño estructural optimizado
Usando Fusion 360 se podrían:
●   Reducir peso aplicando aligeramientos en zonas no críticas.
●   Mejorar la ergonomía del soporte.
●   Aumentar la rigidez de la estructura para reducir resonancias.●   Integrar directamente soportes para cableado interno.
Un diseño más optimizado también permitiría mejorar la autonomía de la batería.
5. Integración de una batería de mayor capacidad y sistema de carga
Se recomienda:
●   Agregar un módulo de carga TP4056.
●   Mejorar la autonomía utilizando una batería Li-Ion 18650.
●   Incorporar un indicador de nivel de batería en la PCB.
Esto haría que el gimbal sea más cómodo y seguro de utilizar.
6. Agregar modos de funcionamiento
El software podría ampliarse con funciones más completas:
●   Modo “lock”: mantiene fija la orientación.
●   Modo “follow”: sigue el movimiento del usuario suavizado.
●   Modo panorámico: giro automático para capturas 360°.
●   Configuración mediante Bluetooth o app móvil.
7. Integración de una cámara más pesada
Con un rediseño mecánico y motores más potentes se podría admitir:
●   Cámaras deportivas (GoPro).
●   Cámaras compactas.
●   Módulos de visión artificial.
Esto ampliará notablemente su campo de aplicación.
**APÉNDICES**
Apéndice A – Manual de Usuario del Gimbal de 3 Ejes
1. Introducción
Este manual tiene como objetivo indicar al usuario cómo utilizar correctamente el gimbal electrónico de 3 ejes diseñado en este proyecto. Incluye instrucciones de encendido, calibración, operación y mantenimiento básico.
2. Componentes del sistema
El gimbal está compuesto por:
●   Estructura 3D impresa (PLA) diseñada en Fusion 360.
●   Microcontrolador Arduino Nano (o placa equivalente).
●   Sensor inercial MPU6050 (acelerómetro + giroscopio).
●   3 servomotores SG90 para los ejes Pitch, Roll y Yaw.
●   Placa PCB personalizada diseñada en KiCad.
●   Convertidor Step-Up LM2596 (para elevar la tensión de la batería).
●   Batería LiPo/Li-Ion 3.7 V.
●   Cableado interno con conectores JST.
3. Puesta en marcha
3.1 Encendido
1.  Conectar la batería al sistema.
2.  Verificar que el LED del Arduino encienda.3.  Esperar 2–3 segundos para que el MPU6050 se inicialice.
4. Mantener el gimbal quieto mientras se calibra automáticamente.
3.2 Calibración inicial
Durante los primeros segundos:
●   El sistema toma valores base de aceleración y giro.
●   El PID se ajusta a la postura inicial de la cámara.
Recomendación:
Colocar el gimbal en una superficie estable y evitar movimiento.
4. Operación
4.1 Movimientos
El gimbal estabiliza automáticamente los tres ejes:
●   Pitch (inclinación vertical)
●   Roll (rotación lateral)
●   Yaw (rotación horizontal)
El usuario puede manipular la base mientras el sistema compensa movimientos involuntarios.
4.2 Funcionamiento del sistema
El algoritmo PID calcula en tiempo real la diferencia entre:
●   Ángulo deseado (0°)
●   Ángulo real medido por el MPU6050y ajusta los servos inmediatamente.
4.3 Modo de uso recomendado
●   Movimientos suaves → mejor estabilización.
●   Evitar golpes o vibraciones excesivas.
●   Mantener la cámara bien centrada en el soporte.
●   No exceder el peso recomendado (ideal: cámaras pequeñas).
5. Mantenimiento
5.1 Mantenimiento eléctrico
●   Verificar periódicamente soldaduras y conectores JST.
●   Asegurarse de que el LM2596 entregue 5 V estables.
●   No utilizar baterías hinchadas o dañadas.
5.2 Mantenimiento mecánico
●   Revisar tornillos y soportes 3D.
●   Evitar exposición prolongada al sol (PLA puede deformarse).
● Aplicar suavemente lubricación siliconada en los servos si es necesario.
6. Solución de problemas
Problema
Causa probable
Solución
El gimbal vibra demasiado
Ganancias PID incorrectas
Ajustar Kp, Ki o Kd
Se inclina hacia un lado
Calibración incorrecta
Reiniciar con el gimbal
quietoLos servos hacen ruido continuo
No enciende
Demasiado esfuerzo mecánico
Sin alimentación
Recentrar la cámara Revisar batería y LM2596
Movimientos lentos
Servo dañado o bajo torque
Reemplazar servo SG90
8. Seguridad
●   No conectar la batería al revés.
●   Evitar contacto con agua o humedad.
● No operar cerca de objetos frágiles o personas.
●   Mantener fuera del alcance de niños.
**BIBLIOGRAFÍA**
●   Arduino. Arduino Nano Documentation. Arduino.cc.
●   Bosch Sensortec. MPU6050 6-axis MotionTracking Device – Datasheet.
●   KiCad Project. KiCad EDA Documentation. kicad.org.
●   SG90 Micro Servo Motor. Technical Specifications – TowerPro.
●   Texas Instruments. LM2596 Step-Down Voltage Regulator Datasheet.
●   National Instruments. PID Theory Explained.
●   O’Reilly Media. Make: Electronics – Learning Through Discovery.
●   Z. Chen & Others. Fundamentals of Inertial Navigation and Inertial Sensors.
●   Autodesk. Fusion 360 User Manual. autodesk.com.
●   Sebastian Madgwick. An efficient orientation filter for inertial/magnetic sensor arrays.
●   R. Isermann. Digital Control Systems. Springer.
●   Proteus & KiCad community forums (consultas técnicas).
Páginas utilizadas para teoría e investigación
●   https://www.arduino.cc
●   https://kicad.org
●   https://randomnerdtutorials.com (ejemplos MPU6050 y ESP32)
●   https://makeradvisor.com
●   https://electronics-tutorials.ws●   https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050
Bibliografía sugerida para ampliación (opcional)
●   Lynch, K. & Park, F. Modern Robotics. Cambridge University Press.
●   Craig, J. Introduction to Robotics: Mechanics and Control. Pearson.
●   Dorf & Bishop. Modern Control Systems.



