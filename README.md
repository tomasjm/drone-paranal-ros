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
Es necesario instalar los siguientes paquetes 
- [x] Compilador C (GCC)
- [x] CMake
- [x] Ros Toolchain

### 2. Instalación de ROS Melodic en Ubuntu 18.04 LTS
- Configurar los repositorios de Ubuntu para permitir paquetes restrictivos : https://help.ubuntu.com/community/Repositories/Ubuntu

#### 2.1. Agregar la lista de paquetes de ROS Melodic (sources.list)
Se agrega la lista al archivo de sources.list:
```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'```
Se configura el keyserver:
```sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654```
#### 2.2. Instalación
Para instalar ROS se ejecutan los siguientes comandos
```
sudo apt update
sudo apt install ros-melodic-ros-base
```
El paquete ```ros-melodic-ros-base``` es el más básico posible para la instalación en una raspberry o beaglebone, para otras opciones revisar la documentación de instalación de ROS en la distribución de Ubuntu. El paquete base no tiene aplicaciones GUI.

#### 2.3. Configuración del entorno macro de ROS
Es necesario agregar el ```setup.bash``` de Ros-melodic en los sources de la terminal utilizada, de esta manera, se tendrá siempre acceso a los comandos de ROS.
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### 2.4. Instalación de dependencias de comandos de ROS
```sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential```
Para confirmar la instalación, ejecutar:
```
sudo rosdep init
rosdep update
```
### 3. Inicializar Workspace de Catkin para trabajar con ROS
ROS utiliza workspaces de Catkin para poder trabajar con su entorno, para inicializar un workspace, es necesario seguir los siguientes pasos:
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
catkin_init_workspace
```
Si se ejecuta todo sin errores, el workspace se habrá creado correctamente, con esto funcionando, será necesario proceder a configurar e instalar el OnboardSDK del drone.
 

### 4. Configuración e instalación de OnboardSDK y OnboardSDK-ROS

#### 4.1. OnboardSDK
Es necesario clonar el código fuente del repositorio en la carpeta ```src``` de ```catkin_ws```.
```
cd ~/catkin_ws/src
git clone https://github.com/dji-sdk/Onboard-SDK
```
Con esto se tendrá acceso al código del SDK, ahora es necesario compilar e instalar el SDK en el sistema.
```
cd ~/catkin_ws/src/Onboard-SDK
mkdir build && cd build
cmake ..
sudo make -j7 install
```
#### 4.2. OnboardSDK-ROS
Para este paso solo es necesario clonar el código fuente de OSDK-ROS en la carpeta ```src``` de ```catkin_ws```:
```
cd ~/catkin_ws/src
git clone https://github.com/dji-sdk/Onboard-SDK-ROS
```
#### 4.3. Instalación de paquetes necesarios para la comunicación 
-  Instalar nmea comms: 
```sudo apt install ros-{release}-nmea-comms```
Para obtener la versión de ros, utilizar ```echo $ROS_DISTRO```
- Instalar ffmpeg ```sudo apt-get install libavcodec-dev libswresample-dev```

### 3. Configurar permisos
- Añadir usuario al grupo dialout ```sudo usermod -a -G dialout $ USER```
-  Instalar libUSB si es que no existe para permitir la comunicación serial en el puerto ```ttyUSB0 (por defecto)```. 
```sudo apt-get install libusb-1.0-0-dev```

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
