
### **Tutorial: Micro-ROS en ESP32 con PlatformIO y ROS 2 Jazzy**

**Objetivo:** Crear un nodo Micro-ROS en un ESP32 que publique un contador numérico en un tópico de ROS 2. Usaremos la configuración moderna de PlatformIO para comunicarnos con un agente Micro-ROS que se ejecuta en Ubuntu 24.04 con ROS 2 Jazzy.

-----

### **1. Prerrequisitos en el Host (Ubuntu 24.04)**

Debes tener un entorno funcional con:

  * **ROS 2 Jazzy Jalisco** instalado.
  * **Visual Studio Code** con la extensión **PlatformIO IDE**.
  * **Herramientas de compilación de ROS 2** (`colcon`, `git`, etc.).

### **2. Configurar el Agente de Micro-ROS (Versión Jazzy)**

El agente se debe compilar desde el fuente para que coincida con tu instalación de ROS 2 Jazzy.

1.  **Crear y compilar el workspace del agente:**
    ```bash
    # Crear workspace
    mkdir -p ~/micro_ros_ws/src
    cd ~/micro_ros_ws

    # Clonar el repositorio de configuración para Jazzy
    git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

    # Instalar dependencias y compilar
    source /opt/ros/jazzy/setup.bash
    rosdep update && rosdep install --from-paths src --ignore-src -y
    colcon build

    # Sourcear el workspace compilado
    source install/local_setup.bash
    ```

Para configurar el agente sigue las instrucciones de: https://micro.ros.org/docs/tutorials/core/first_application_linux/

### **3. Crear y Configurar el Proyecto PlatformIO para ESP32 (Método Actualizado)**

Aquí es donde aplicamos tu mejora.

1.  **Crear un Nuevo Proyecto en PlatformIO:**

      * **Name:** `micro_ros_esp32_jazzy`
      * **Board:** `ESP32 Dev Module`
      * **Framework:** `Arduino`

2.  **Configurar `platformio.ini`:**
    Abre el archivo `platformio.ini` que se encuentra en la raíz de tu proyecto y reemplaza su contenido con esta configuración limpia y moderna:

    ```ini
    [env:esp32dev]
    platform = espressif32
    board = esp32dev
    framework = arduino

    ; --- Configuración específica de Micro-ROS ---
    ; Define la distribución de ROS 2 a la que apuntamos.
    board_microros_distro = jazzy

    ; Define el método de transporte. 'serial' usa el USB-UART.
    board_microros_transport = serial

    ; Dependencia de la biblioteca de Micro-ROS para PlatformIO
    lib_deps =
        https://github.com/micro-ROS/micro_ros_platformio

    ; Velocidad para el monitor serie
    monitor_speed = 115200
    ```

#### **Ventajas de este Método**

  * **Claridad:** `board_microros_distro = jazzy` es mucho más explícito que esperar que las versiones coincidan implícitamente.
  * **Simplicidad:** `board_microros_transport = serial` reemplaza la necesidad de conocer y escribir las `build_flags` específicas como `-D MICRO_ROS_TRANSPORT_ARDUINO_SERIAL`. La biblioteca se encarga de los detalles.
  * **Mantenibilidad:** Si cambias de transporte (por ejemplo, a WiFi), solo necesitas cambiar esta línea a `board_microros_transport = wifi`, lo cual es mucho más intuitivo.

### **4. Escribir el Código del Firmware**

**El código C++ en `src/main.cpp` no necesita ningún cambio.** La API de `rclc` sigue siendo la misma. La configuración en `platformio.ini` solo afecta a cómo la biblioteca se compila y se enlaza, no a cómo la usas en tu código.

Puedes usar el código para probar la comunicación:

```cpp
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

// Define the built-in LED pin, which is usually GPIO 2 on most ESP32 dev boards.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "esp32_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
```

**Nota de optimización:** Con la nueva configuración, la llamada `set_microros_serial_transports(Serial);` en `setup()` ya no es estrictamente necesaria porque la biblioteca la configura internamente basándose en `board_microros_transport = serial`. Sin embargo, es una buena práctica dejarla por si acaso o para compatibilidad, ya que no interfiere. Para un código más limpio, puedes removerla.

### **5. Flujo de Trabajo: Compilar, Subir y Verificar**

Este proceso final es exactamente el mismo.

1.  **Iniciar el Agente de Micro-ROS:**
    En una terminal, sourcea tus entornos y ejecuta el agente.

    ```bash
    source /opt/ros/jazzy/setup.bash
    source ~/micro_ros_ws/install/local_setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
    ```

2.  **Compilar y Subir desde VS Code:**

      * Usa el botón de **Build (✓)** para compilar.
      * Usa el botón de **Upload (→)** para flashear el firmware en el ESP32.

3.  **Verificar en ROS 2:**
    En una segunda terminal, sourcea ROS 2 y verifica que todo funciona.

    ```bash
    source /opt/ros/jazzy/setup.bash
    ros2 topic list
    # Deberías ver /esp32_publisher
    ros2 topic echo /esp32_publisher
    # Deberías ver el contador incrementándose
    ```

¡Felicidades\! Ahora tienes un proyecto configurado con las mejores y más actuales prácticas para usar Micro-ROS con PlatformIO, lo que hará que tus futuros proyectos sean más fáciles de gestionar.