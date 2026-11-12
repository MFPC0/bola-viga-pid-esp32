# Sistema bola-viga con control PID (ESP32)

Una viga movida por un servomotor mantiene una pelota en la posición que se configura desde una interfaz web. La posición se mide con un sensor infrarrojo Sharp GP2Y0A21.

Proyecto integrador de Ingeniería en Mecatrónica, Universidad Tecnológica Metropolitana (UTM), Mérida, 2026. Proyecto en equipo. **3.er lugar en feria de proyectos.**

## Qué se hizo
- Adaptación a ESP32 del código PID original de [ElectroNoobs](https://electronoobs.com) y ensamblaje de la máquina.
- Filtro de Kalman para la lectura del sensor, anti-windup en la integral y ajuste de ganancias según la zona de la viga.
- Interfaz web embebida en la ESP32 (Access Point + WebSocket + ArduinoJson): setpoint, modo automático/manual, sintonía de Kp, Ki y Kd y gráficas en vivo.
- Sintonización experimental con registros de varias corridas.

## Hardware
- ESP32
- Servomotor (pin 18)
- Sensor Sharp GP2Y0A21 (ADC, pin 34)

## Uso
1. Abre `Base_PID_sintonizado.ino` en Arduino IDE (en una carpeta con el mismo nombre).
2. Librerías: ESP32Servo, ArduinoJson, ESPAsyncWebServer y AsyncTCP.
3. Carga el sketch, conéctate a la red `PID_BallBeam` y abre la IP que aparece en el monitor serial.

## Créditos
El algoritmo PID base y el diseño mecánico de referencia son de ElectroNoobs. La adaptación a ESP32, el filtrado, la sintonía y la interfaz web se hicieron en este proyecto.
