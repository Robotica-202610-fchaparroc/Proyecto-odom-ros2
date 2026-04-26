
# Proyecto2: Aplicación de Algoritmo A* en escenas de Navegación con Gazebo y C-Space

Este proyecto permite cargar escenas .txt y sdf para simular el entorno de un robot en un espacio de configuraciones en Gazebo, permitiendo construir un C-space con un punto de inicio y punto final del robot. Una vez discretizado el espacio de configuraciones se usa el algorimto A* para planear la ruta del punto inicial al punto final teniendo en cuenta los obstáculos del espacio. Finalmente una vez se encuentra la ruta se envian los comandos por odometria para que el robot realice la ruta correspondiente.

## Estructura principal
- `proyecto/` : Código fuente del paquete.
	- `navigation_node.py` : Nodo principal que expone funciones de utilidad para estudiantes (leer Lidar, rotar, mover relativo, cargar escenas, menú interactivo).
	- `logic/lidar.py` : Funciones de ayuda para extraer distancias y rangos del mensaje `LaserScan`.
	- `logic/movement.py` : Algoritmos para calcular velocidades angulares/lineales y perfiles de movimiento relativos.
	- `logic/astra_grid.py` : Módulo para encontrar la ruta del robot usando el algoritmo A* sobre la malla creada del C-Space.
	- `logic/c_space.py` : Módulo para representar el espacio de configuraciones.
	- `logic/cspace_from_txt.py` : Módulo que convierte el archivo .txt en una malla C-space. Interpreta obstáculos, dimensiones, genera la matriz C-Space junto con los C-Obstáculos.
	- `logic/cspace_plot.py` : Módulo usado para dibujar el C-space, mostrando la malla, obtáculos reales, C-obstáculos, ruta del A*, y punto de inicio y final del robot.
	- `logic/geometry.py` : Módulo con operaciones matematicas para interpretar las posiciones del robot y las formas de los obstaculos en el C-space.
	- `logic/point_poligone.py` : Módulo para determinar si un punto esta dentro o fuera de una obstáculo, ayuda para la clasificación de las celdas.

- `data/` : Archivos de escena (`Escena-Problema1.txt` ... `Escena-Problema6.txt`) usados por el nodo para pruebas y visualización textual.
- `worlds/` : Archivos de escena (`escena1.sdf` ... `escena2.sdf`) usados por el simulador gazebo, para visualizar el entorno real, la posición del robot y el movimiento del mismo en el entorno.
- `resource/` : Recursos de empaquetado.
- `test/` : Tests básicos y comprobaciones de estilo.

## Características importantes
- Interfaz de consola no bloqueante: `navigation_node.py` inicia un hilo con un menú interactivo para probar funciones durante la ejecución.
- Wrappers didácticos: Métodos como `leer_distancia_en_angulo`, `leer_distancias_en_rango`, `rotar_relativo`, `mover_relativo` están pensados para llamar fácilmente desde el menú o se integren en otros módulos.
- Manejo de escenas: `cargar_escena_txt(numero)` lee el archivo `data/Escena-Problema{numero}.txt` y guarda su contenido en `self.texto_escena` y la función `iniciar_escena_completa(numero_escena)` permite cargar la escena .sdf en el simulador gazebo, ejecutar el puente bridge, cargar la excena txt y verificar que una vez cargada la escena se pueda hacer lectura de odometría y lidar, esta función se habilita desde el menú con la opción **1. Cargar e iniciar simulador con escena**.
- Generar C-space con malla y ruta de planificación: Desde el menú con la opción **2. Generar C-space desde escena TXT, Planificar ruta con A* y Graficar malla C-space** una vez se haya cargado la escena a trabajar, se utiliza esta opción para crear el C-space en dicha escena, discretizar la malla en el espacio y generar la ruta de planificación con A*, obteniendo como resultado una gráfica con el resultado.
- Ejecutar la ruta A* en el robot: Desde el menú con la opción **4. Ejecutar plan A* en el robot** se ejecuta la ruta A* definida para la escena, enviando comandos al robot en gazebo para que ejecute los comandos y llegue al final.
- Nuevas escenas: Si se quieren agregar nuevas escenas, solo es necesario agregar el archivo .txt en `/data` con el nombre **Escena-Problema#.txt** y el archivo .sdf en la carpeta `/worlds` con el nombre **escena#.sdf**, automaticamente el sistema las reconocera y ya podra realizar el C-space, calcular la ruta y ejecutarla a travez del menú.

## Cómo ejecutar (entorno ROS2)
1. Asegurar que el proyecto este dentro de un entorno Ros2

2. Dirigirse a la fuente del workspace (desde la raíz del workspace):

```bash
colcon build --symlink-install
source install/setup.bash
```

3. Ejecutar el nodo (desde un terminal con ROS2 configurado):

```bash
ros2 run proyecto navigation
```

Nota: El paquete asume tópicos `odom`, `scan_raw` y `cmd_vel` disponibles en el bus ROS2 (simulador Gazebo, robot real o nodo de prueba).

## Menú interactivo
1. Cargar e iniciar simulador con escena
2. Generar C-space desde escena TXT, Planificar ruta con A* y Graficar malla C-space
3. Ejecutar plan A* en el robot
4. Salir

**Notas**:
1. Luego de agregar los archivos de las esencas en las carpetas `/data` y `/worlds` respectivamente se ejecuta el programa, y se va a ejecutando en orden, 1 cargar la escena con gazebo, 2 crear el C-space y generar la ruta y 3 ejecutar la ruta enviando comandos al robot dentro del simulador.
2. Estos pasos son consecutivos, si no cargo la escena, no podra ejecutar el simulador. Si no ejecuto el simulador no podra crear el C-space, y si se generó el C-space pero no se encontró una ruta valida con el Algoritmo no se ejecutará el envio de comandos al robot.


## Editar y extender
- Para cambiar la lógica del Lidar, revise `proyecto/logic/lidar.py`.
- Para ajustar perfiles de movimiento o tolerancias, edite `proyecto/logic/movement.py`.
- Para añadir nuevas escenas, coloque archivos `Escena-ProblemaX.txt` en la carpeta `data/`.
