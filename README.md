# Practica 1 Robo

colcon build

source install/setup.bash
source /opt/ros/jazzy/setup.bash

# Práctica 1 - Robótica: Vehículo Autónomo con Webots y ROS2

## 📋 Descripción

Este paquete implementa un vehículo autónomo en simulación que:
- ✅ Sigue las marcas viales de la carretera
- ✅ Detecta señales de tráfico (STOP, YIELD, límites de velocidad)
- ✅ Reacciona según las señales detectadas

## 🏗️ Arquitectura del Sistema

### Diagrama de Nodos y Comunicación

```
┌─────────────────────────────────────────────────────────────────┐
│                         WEBOTS SIMULATOR                         │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              CitroenCZero Robot                           │  │
│  │  - Motores: left_rear_wheel, right_rear_wheel            │  │
│  │  - Cámaras: car_camera (512x256), road_camera (512x16)  │  │
│  └──────────────────────────────────────────────────────────┘  │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                │ webots_ros2_driver
                                │
                ┌───────────────┴───────────────┐
                │                               │
                ▼                               ▼
    ┌─────────────────────┐         ┌─────────────────────┐
    │  car_camera         │         │  road_camera        │
    │  /car_camera/       │         │  /road_camera/      │
    │  image_raw          │         │  image_raw          │
    └──────────┬──────────┘         └──────────┬──────────┘
               │                               │
               │                               │
               ▼                               ▼
    ┌─────────────────────┐         ┌─────────────────────┐
    │  sign_detector      │         │  lane_controller    │
    │  - Reconocimiento   │         │  - Detección línea  │
    │    de señales       │         │  - Control PD       │
    │    (OpenCV)         │         │  - Suavizado        │
    └──────────┬──────────┘         └──────────┬──────────┘
               │                               │
               │ /car/max_speed               │ /car/left_wheel
               │ (Float64)                     │ /car/right_wheel
               │                               │ (Float64)
               ▼                               │
    ┌─────────────────────┐                   │
    │  lane_controller    │◄──────────────────┘
    │  (recibe comando    │
    │   de velocidad)     │
    └──────────┬──────────┘
               │
               │ /car/left_wheel
               │ /car/right_wheel
               │ (Float64)
               ▼
    ┌─────────────────────────────┐
    │  webots_ros2_driver         │
    │  (Plugin que conecta con    │
    │   motores del robot)        │
    └──────────────────────────────┘
```

### Topics ROS2

| Topic | Tipo | Publicador | Suscriptor | Descripción |
|-------|------|------------|-----------|-------------|
| `/car_camera/image_raw` | `sensor_msgs/Image` | webots_ros2_driver | sign_detector | Imagen frontal 512x256 |
| `/road_camera/image_raw` | `sensor_msgs/Image` | webots_ros2_driver | lane_controller | Imagen carretera 512x16 |
| `/car/max_speed` | `std_msgs/Float64` | sign_detector | lane_controller | Velocidad máxima actual |
| `/car/left_wheel` | `std_msgs/Float64` | lane_controller | webots_ros2_driver | Velocidad rueda izq. |
| `/car/right_wheel` | `std_msgs/Float64` | lane_controller | webots_ros2_driver | Velocidad rueda der. |

## 📦 Estructura del Paquete

```
car_pkg/
├── car_pkg/
│   ├── __init__.py
│   ├── lane_controller.py      # Seguimiento de carretera
│   └── sign_detector.py        # Detección de señales
├── launch/
│   └── robot_launch.py         # Launcher principal
├── resource/
│   ├── car_pkg                 # Archivo de recurso vacío
│   ├── car.urdf                # Definición URDF del robot
│   ├── stop.png                # Template señal STOP
│   ├── yield.png               # Template señal YIELD
│   ├── speed_55.jpg            # Template velocidad 55
│   └── speed_65.png            # Template velocidad 65
├── world/
│   └── city_traffic.wbt        # Mundo de simulación
├── package.xml                 # Dependencias del paquete
├── setup.py                    # Configuración de instalación
└── setup.cfg                   # Configuración de scripts
```

## 🔧 Dependencias

### Paquetes ROS2 necesarios:
- `rclpy` - Cliente Python de ROS2
- `sensor_msgs` - Mensajes de sensores
- `std_msgs` - Mensajes estándar
- `geometry_msgs` - Mensajes de geometría
- `cv_bridge` - Puente OpenCV-ROS
- `webots_ros2_driver` - Driver de Webots

### Paquetes Python necesarios:
- `opencv-python` (`cv2`)
- `numpy`

## 🚀 Instalación

### 1. Instalar Webots ROS2
```bash
sudo apt install ros-jazzy-webots-ros2
export WEBOTS_HOME=/usr/local/webots
```

### 2. Crear workspace (si no existe)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Clonar el paquete
```bash
# Copia tu paquete car_pkg en ~/ros2_ws/src/
```

### 4. Compilar
```bash
cd ~/ros2_ws
colcon build --packages-select car_pkg
source install/setup.bash
```

## ▶️ Ejecución

### Lanzar la simulación completa:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch car_pkg robot_launch.py
```

Esto iniciará:
1. Webots con el mundo `city_traffic.wbt`
2. El driver de Webots conectado al robot
3. El nodo `lane_controller` (seguimiento de carretera)
4. El nodo `sign_detector` (detección de señales)

### Comandos útiles para depuración:

**Ver lista de topics activos:**
```bash
ros2 topic list
```

**Ver mensajes de un topic:**
```bash
ros2 topic echo /car_camera/image_raw
ros2 topic echo /car/max_speed
```

**Ver información de un nodo:**
```bash
ros2 node info /lane_controller_node
```

**Ver diagrama de nodos:**
```bash
ros2 run rqt_graph rqt_graph
```

## 🧩 Componentes Principales

### 1. Lane Controller (`lane_controller.py`)

**Función:** Mantener el vehículo centrado en el carril siguiendo las marcas viales.

**Algoritmo:**
1. **Procesamiento de imagen:**
   - Recibe imagen 512x16 de `road_camera`
   - Convierte a escala de grises
   - Calcula media por columna → vector 1D de 512 valores
   - Encuentra el máximo (centro de las marcas viales)

2. **Control PD:**
   ```
   error = (posición_línea - centro_ideal) / (ancho/2)
   derivada = (error - error_anterior) / dt
   giro = Kp * error + Kd * derivada
   ```

3. **Conversión a velocidades de rueda:**
   ```
   velocidad_izq = velocidad_base + giro * MAX_TURN_DIFF
   velocidad_der = velocidad_base - giro * MAX_TURN_DIFF
   ```

4. **Suavizado:** Buffer de 5 muestras para suavizar las salidas

**Parámetros ajustables:**
- `BASE_SPEED = 10.0` - Velocidad base en rad/s
- `KP = 0.5` - Ganancia proporcional
- `KD = 0.1` - Ganancia derivativa
- `MAX_TURN_DIFF = 8.0` - Diferencia máxima entre ruedas

### 2. Sign Detector (`sign_detector.py`)

**Función:** Detectar señales de tráfico y publicar comandos de velocidad.

**Algoritmo:**
1. **Detección por template matching (OpenCV):**
   ```python
   result = cv2.matchTemplate(imagen, template, cv2.TM_CCOEFF_NORMED)
   _, max_confidence, _, _ = cv2.minMaxLoc(result)
   
   if max_confidence > THRESHOLD:
       señal_detectada = True
   ```

2. **Lógica de señales:**
   - **STOP:** Publica velocidad 0.0 → el coche para 1 segundo
   - **YIELD:** Publica velocidad máxima / 2
   - **SPEED_55:** Publica 55.0 km/h
   - **SPEED_65:** Publica 65.0 km/h

3. **Anti-rebote:** Evita detecciones repetidas de la misma señal (5 segundos de cooldown)

**Parámetros ajustables:**
- `TEMPLATE_THRESHOLD = 0.7` - Umbral de confianza para detección (0.0-1.0)

### 3. Launcher (`robot_launch.py`)

**Función:** Iniciar todos los componentes del sistema.

**Componentes lanzados:**
1. **WebotsLauncher:** Inicia Webots con `city_traffic.wbt`
2. **WebotsController:** Conecta el driver ROS2 con el robot usando `car.urdf`
3. **lane_controller:** Nodo de seguimiento de carril
4. **sign_detector:** Nodo de detección de señales
5. **Event handler:** Cierra todos los nodos cuando Webots se cierra

## 🔍 Solución de Problemas

### Problema: Webots no se inicia
```bash
# Verifica que WEBOTS_HOME esté configurado:
echo $WEBOTS_HOME

# Si no está configurado:
export WEBOTS_HOME=/usr/local/webots
```

### Problema: No se encuentran las plantillas de señales
```bash
# Verifica que las imágenes estén en:
ls ~/ros2_ws/install/car_pkg/share/car_pkg/resource/

# Si no están, recompila con:
colcon build --packages-select car_pkg --symlink-install
```

### Problema: El coche no se mueve
```bash
# Verifica que los topics estén publicando:
ros2 topic hz /car/left_wheel
ros2 topic hz /car/right_wheel

# Verifica los logs del lane_controller:
ros2 node list
ros2 node info /lane_controller_node
```

### Problema: Las señales no se detectan
```bash
# Ajusta el umbral en sign_detector.py:
self.TEMPLATE_THRESHOLD = 0.6  # Prueba valores entre 0.5-0.8

# Verifica las imágenes capturadas:
ros2 topic echo /car_camera/image_raw
```

## 📊 Ajustes de Rendimiento

### Mejorar seguimiento de carril:
1. **Curvas cerradas:** Aumentar `MAX_TURN_DIFF`
2. **Oscilaciones:** Reducir `KP`, aumentar `KD`
3. **Lentitud de reacción:** Aumentar `KP`

### Mejorar detección de señales:
1. **Falsos positivos:** Aumentar `TEMPLATE_THRESHOLD`
2. **No detecta señales:** Reducir `TEMPLATE_THRESHOLD`
3. **Añadir más templates:** Agregar imágenes con diferentes tamaños en `resource/`

## 📝 Funciones Clave

### lane_controller.py

```python
def camera_callback(self, msg):
    """
    Callback principal que:
    1. Procesa imagen de road_camera
    2. Detecta posición de línea
    3. Calcula control PD
    4. Publica velocidades a ruedas
    """
```

### sign_detector.py

```python
def camera_callback(self, msg):
    """
    Callback que:
    1. Recibe imagen de car_camera
    2. Compara con templates usando matchTemplate
    3. Detecta señal con mayor confianza
    4. Publica comando de velocidad correspondiente
    """

def process_signal(self, signal_name):
    """
    Procesa la señal detectada y publica:
    - STOP: 0.0 (parará 1 seg automáticamente)
    - YIELD: velocidad_max / 2
    - SPEED_X: X km/h (actualiza velocidad_max)
    """
```

## 🎓 Conceptos Clave de ROS2

### Nodos
- Procesos independientes con una funcionalidad específica
- En este proyecto: `lane_controller` y `sign_detector`

### Topics
- Canales de comunicación asíncrona publish/subscribe
- Ejemplo: `/car_camera/image_raw`, `/car/left_wheel`

### Messages
- Estructuras de datos enviadas por topics
- Ejemplo: `sensor_msgs/Image`, `std_msgs/Float64`

### Launch Files
- Scripts que inician múltiples nodos con configuración
- Facilita el despliegue del sistema completo

## 📚 Referencias

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Webots ROS2 Interface](https://github.com/cyberbotics/webots_ros2)
- [OpenCV Template Matching](https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html)

## 👥 Autor

Proyecto desarrollado para la asignatura de Robótica - ETSISI UPM

## 📄 Licencia

Apache License 2.0
