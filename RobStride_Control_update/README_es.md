# RobStride Control

Biblioteca de control de motores RobStride con implementaciones en Python, C++, Rust y Arduino.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B17)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://www.python.org/downloads/)
[![Rust](https://img.shields.io/badge/Rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32-00979D.svg)](https://www.arduino.cc/)

## Características del Proyecto

- **Soporte multi-lenguaje**: Python (facilidad de uso), C++ (alto rendimiento), Rust (seguridad), Arduino (embebido)
- **Múltiples modos de control**: Modo MIT, control de posición, control de velocidad, control de par
- **Rendimiento en tiempo real**: Ciclos de control de alta frecuencia, respuesta de baja latencia
- **Fiabilidad industrial**: Manejo completo de errores y mecanismos de protección
- **Soporte multiplataforma**: Sistemas Linux y controladores embebidos

## Modelos de Motores Soportados

| Modelo | Par Máximo | Velocidad Máxima | Rango KP | Rango KD |
|--------|------------|------------------|----------|----------|
| RS-00 | 17 Nm | 50 rad/s | 500.0 | 5.0 |
| RS-01 | 17 Nm | 44 rad/s | 500.0 | 5.0 |
| RS-02 | 17 Nm | 44 rad/s | 500.0 | 5.0 |
| RS-03 | 60 Nm | 50 rad/s | 5000.0| 100.0|
| RS-04 | 120 Nm| 15 rad/s | 5000.0| 100.0|
| RS-05 | 17 Nm | 33 rad/s | 500.0 | 5.0 |
| RS-06 | 60 Nm | 20 rad/s | 5000.0| 100.0|

## Estructura del Proyecto

```
RobStride-Control/
├── python/              # Implementación Python
│   ├── src/            # Código fuente
│   ├── examples/       # Programas de ejemplo
│   └── robstride_dynamics/  # SDK de Python
├── cpp/                 # Implementación C++
│   ├── src/            # Código fuente
│   ├── include/        # Archivos de cabecera
│   └── examples/       # Programas de ejemplo
├── rust/                # Implementación Rust
│   ├── src/            # Código fuente
│   └── examples/       # Programas de ejemplo
├── arduino/             # Implementación Arduino
│   ├── simple_joint_control/    # Control simple de articulación
│   ├── joint_position_control/   # Control de posición
│   ├── dual_motor_control/       # Control dual de motores
│   └── advanced_motor_control/   # Control avanzado
└── scripts/             # Scripts de construcción e instalación
```

## Inicio Rápido

### Requisitos del Sistema

- **Sistema operativo**: Linux (Ubuntu 18.04+ / Debian 10+)
- **Hardware**: Interfaz CAN compatible con SocketCAN
- **Permisos**: Acceso root o sudo a dispositivos CAN

### Configuración del Entorno

```bash
# Instalar herramientas CAN
sudo apt-get install can-utils

# Configurar interfaz CAN
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Ejecutar script de configuración
./scripts/setup.sh
```

### Versión Python

```bash
cd python
pip install -r requirements.txt

# Control de posición
python3 src/position_control.py 11

# Control de velocidad
python3 src/speed_control.py 11

# Programas de ejemplo
python3 examples/basic_usage.py 11
```

### Versión C++

```bash
cd cpp
make install-deps
make

# Ejecutar programa principal
sudo ./build/robstride-mit-position 11

# Ejecutar ejemplo
g++ -std=c++17 -I../include examples/basic_control.cpp -o basic_control
sudo ./basic_control 11
```

### Versión Rust

```bash
cd rust
cargo build --release

# Ejecutar programa principal
cargo run --release --bin robstride-mit-position -- 11

# Ejecutar ejemplo
cargo run --release --bin basic_control -- 11
```

### Versión Arduino

```bash
cd arduino

# Usar Arduino IDE
1. Abrir simple_joint_control/simple_joint_control.ino
2. Seleccionar ESP32 Dev Module
3. Subir programa

# Usar PlatformIO
pio run --target upload

# Diferentes ejemplos de control
- simple_joint_control     # Control simple de articulación
- joint_position_control    # Control de posición
- dual_motor_control        # Control dual de motores
- advanced_motor_control    # Control avanzado
```

## Métricas de Rendimiento

| Implementación | Frecuencia Control | Latencia | Uso CPU | Uso Memoria | Plataforma |
|----------------|-------------------|---------|----------|-------------|-----------|
| Python         | 100 Hz            | 5ms     | 15%      | 50MB        | Linux |
| C++            | 200 Hz            | 1ms     | 5%       | 10MB        | Linux |
| Rust           | 150 Hz            | 2ms     | 8%       | 15MB        | Linux |
| Arduino        | 50-200Hz          | 2-20ms  | 5-15%    | 10-50KB     | ESP32/MCU  |

## Modos de Control

### Modo MIT (Modo 0)
Envío directo de objetivos de posición, velocidad y par para aplicaciones de alto rendimiento.
- **Frecuencia de control**: 50-100Hz
- **Aplicaciones**: Control de articulaciones robot, posicionamiento preciso

### Modo Posición (Modo 1)
Control de posición basado en lazo de posición interno.
- **Frecuencia de control**: 20-50Hz
- **Aplicaciones**: Movimiento punto a punto, seguimiento de trayectoria

### Modo Velocidad (Modo 2)
Control de lazo cerrado de velocidad.
- **Frecuencia de control**: 20-50Hz
- **Aplicaciones**: Control de cintas transportadoras, robots con ruedas

## Ejemplos de Uso

### Control MIT de Posición Python

```python
from src.position_control import PositionControllerMIT

# Inicializar controlador
controller = PositionControllerMIT(motor_id=11)
controller.connect()

# Establecer posición a 90 grados
controller.set_angle(90.0)

# Ajustar parámetros de control
controller.set_kp(30.0)  # Ganancia proporcional
controller.set_kd(0.5)   # Ganancia derivativa
```

### Control MIT de Posición C++

```cpp
#include "can_interface.h"
#include "protocol.h"

CanInterface can;
can.init("can0");

// Configurar parámetros del motor
enable_motor(can.socket(), 11);
set_mode_raw(can.socket(), 11, ControlMode::MIT_MODE);

// Enviar comando de posición
write_operation_frame(can.socket(), 11, M_PI/2, 30.0, 0.5);
```

### Control MIT de Posición Rust

```rust
let socket = Arc::new(Mutex::new(CanSocket::open("can0")?));

// Habilitar motor y configurar modo
enable_motor(&socket, 11)?;
set_mode_raw(&socket, 11, 0)?;

// Enviar comando de posición
write_operation_frame(&socket.lock()?, 11, std::f64::consts::PI/2.0, 30.0, 0.5)?;
```

### Control MIT de Posición Arduino

```cpp
#include "TWAI_CAN_MI_Motor.h"

TWAI_CAN_MI_Motor motor(11);  // ID del motor=11

void setup() {
    Serial.begin(115200);
    motor.init(CAN_SPEED_1000KBPS);
    motor.enable_motor();
}

void loop() {
    // Establecer posición a 90 grados
    motor.send_mit_command(PI/2, 30.0, 0.5);
    delay(2000);

    // Volver a origen
    motor.send_mit_command(0, 30.0, 0.5);
    delay(2000);
}
```

## Interfaz de Control Interactivo

### Comandos de Control de Posición
- `90` - Establecer a 90 grados
- `-45` - Establecer a -45 grados
- `kp 30` - Establecer ganancia proporcional a 30
- `kd 0.5` - Establecer ganancia derivativa a 0.5
- `home` - Volver a origen
- `status` - Mostrar estado
- `quit` - Salir

### Comandos de Control de Velocidad
- `5.0` - Establecer velocidad positiva 5 rad/s
- `-3.0` - Establecer velocidad negativa 3 rad/s
- `stop` - Detener motor
- `status` - Mostrar estado
- `quit` - Salir

## Solución de Problemas

### Problemas Comunes

1. **No se encuentra el motor**
   ```bash
   # Verificar conexión CAN
   sudo ip link show can0
   # Escanear motores
   python3 -c "from robstride_dynamics import RobstrideBus; print(RobstrideBus.scan_channel('can0'))"
   ```

2. **Acceso denegado**
   ```bash
   # Agregar usuario al grupo dialout
   sudo usermod -a -G dialout $USER
   # Reiniciar sistema
   ```

3. **Control inestable**
   - Ajustar parámetros Kp/Kd
   - Verificar carga del bus CAN
   - Validar estabilidad de la fuente de alimentación

### Herramientas de Depuración

```bash
# Monitorear tráfico CAN
sudo candump can0

# Filtrar ID específico
sudo candump can0,0C0:7FF

# Verificar estadísticas CAN
sudo ip -details link show can0
```

## Guía de Desarrollo

### Requisitos de Compilación

- **Python**: Python 3.8+, python-can
- **C++**: GCC 7+ o Clang 8+, CMake 3.12+
- **Rust**: Rust 1.70+, Cargo
- **Arduino**: Arduino IDE 1.8.19+, soporte ESP32

### Pruebas

```bash
# Pruebas Python
python3 examples/basic_usage.py 11

# Pruebas C++
make test

# Pruebas Rust
cargo test

# Pruebas Arduino
Verificar información de depuración a través del monitor serie del Arduino IDE
```

### Contribuir Código

1. Fork del proyecto
2. Crear rama de características
3. Enviar cambios
4. Crear Pull Request

## Licencia

MIT License - Ver archivo [LICENSE](LICENSE)

## Soporte Técnico

- Issues: https://github.com/tianrking/robstride-control/issues