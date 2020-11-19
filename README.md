# drone-paranal-ros


## Activación y autorización

Al ejecutar por primera vez la aplicación basada en OSDK, el desarrollador deberá ingresar en el DJI Assistant 2 (Windows) para obtener la licencia de autorización desde el servidor de DJI para ejecutar la aplicación.


- [ ] Activar la aplicación: DJI Assistant 2
- [ ] Activar el solucionador de problemas
- [ ] Revisar la conexión del dispositivo "onboard computer", confirmar que el voltaje de linea UART es de 3.3V.
- [ ] Revisar que los dispositivos tengan acceso a internet ( Móvil o control RC ).
- [ ] Revisar que el API CONTROL esté activado en DJI Assistant 2
- [x] Revisar que los datos de aplicación (APP ID, API KEY, BAUD RATE) concuerden con los datos de la aplicación de la central.
- [x] Revisar que los permisos de acceso del dispositivo estén correctos.

## Configuración del entorno de desarrollo

### 1. Herramientas de desarrollo
- [x] Compilador C (GCC)
- [x] CMake
- [x] Ros Toolchain
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```

### 2. Instalar software dependiente

- [x] Clonar código fuente del repositorio OSDK-Ros en la carpeta ```src``` de ```catkin_ws``` : https://github.com/dji-sdk/Onboard-SDK-ROS
- [x] Clonar código fuente de OSDK en la carpeta ```src``` de ```catkin_ws``` : https://github.com/dji-sdk/Onboard-SDK
- [x] Realizar un build del código fuente de OSDK
```
cd catkin_ws/src/Onboard-SDK
mkdir build && cd build
cmake ..
sudo make -j7 install
```
- [x] Instalar nmea comms utilizando ```sudo apt install ros-{release}-nmea-comms```, para obtener la versión de ros, utilizar ```echo $ROS_DISTRO```
- [X] Instalar ffmpeg ```sudo apt-get install libavcodec-dev libswresample-dev```

### 3. Configurar permisos
- [x] Añadir usuario al grupo dialout ```sudo usermod -a -G dialout $ USER```
- [x] Instalar libUSB ```sudo apt-get install libusb-1.0-0-dev```

## Ejecutar el SAMPLE
### Preparación previa
- [x] Ejecutar ```catkin_make``` en el directorio ```catkin_ws```
- [x] Añadir los build al bash utilizando ```source devel/setup.bash``` desde el directorio ```catkin_ws```
- [x] Editar la información de App ID, App Key y otros utilizando ```rosed dji_osdk_ros dji_vehicle_node.launch```

- Los pasos a continuación requieren de tener el vehiculo conectado por UART.

### Ejecutar el nodo principal

Para inicializar el nodo principal se utiliza el siguiente comando:
```roslaunch dji_osdk_ros dji_vehicle_node.launch```

### Ejecutar el código de ejemplo

Para ejecutar el código de ejemplo, se debe inicializar el nodo principal en una terminal adicional.
Ahora, en una nueva terminal, se deben ejecutar los siguientes comandos:
```
$ source devel/setup.bash
$ rosrun dji_osdk_ros flight_control_node
```
También se puede utilizar el nodo ```telemetry_node```
