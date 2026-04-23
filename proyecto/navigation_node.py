import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_movimiento_relativo, calcular_rotacion
from .logic.cspace_from_txt import generar_cspace_desde_texto_escena
from .logic.cspace_plot import mostrar_cspace


class NavigationNode(Node):
    def __init__(self):
        super().__init__('student_navigation')

        # Procesos externos
        self.gz_process = None
        self.bridge_process = None

        # Estado conexión simulador
        self.odom_recibido = False
        self.scan_recibido = False
        self.esperando_conexion = False
        self.tiempo_inicio_espera = None
        self.mensaje_pendiente = None

        # Estado Escena y Malla
        self.cspace_resultado = None
        self.lado_robot=0.5

        # Estado robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None

        # Estado movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None
        self.tiempo_maniobra = 0.0

        # Escena cargada
        self.texto_escena = ""

        # Estado menú/comandos
        self.comando_activo = None
        self.parametros_comando = []

        # ROS interfaces
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil iniciado.")

        # Menú en hilo aparte
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()

    # =======================================================
    # CALLBACKS ROS2
    # =======================================================
    def odom_callback(self, msg):
        self.odom_recibido = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def lidar_callback(self, msg):
        self.scan_recibido = True
        self.last_scan = msg

    # =======================================================
    # RUTAS Y PROCESOS
    # =======================================================
    def obtener_rutas_base(self):
        package_share = get_package_share_directory('proyecto2')
        ruta_data = os.path.join(package_share, 'data')
        ruta_worlds = os.path.join(package_share, 'worlds')
        return ruta_data, ruta_worlds

    def _detener_proceso(self, proceso, nombre):
        if not proceso or proceso.poll() is not None:
            return None

        self.get_logger().info(f"Deteniendo {nombre}...")
        try:
            if os.name == 'nt':
                proceso.terminate()
            else:
                os.killpg(os.getpgid(proceso.pid), signal.SIGTERM)
            proceso.wait(timeout=5)
        except Exception:
            try:
                if os.name == 'nt':
                    proceso.kill()
                else:
                    os.killpg(os.getpgid(proceso.pid), signal.SIGKILL)
            except Exception:
                pass
        return None

    def detener_gazebo(self):
        self.gz_process = self._detener_proceso(self.gz_process, "Gazebo")
        if os.name != 'nt':
            try:
                subprocess.run(["pkill", "-f", "gz sim"], check=False)
            except Exception:
                pass

    def detener_bridge(self):
        self.bridge_process = self._detener_proceso(self.bridge_process, "bridge")

    def lanzar_gazebo_escena(self, numero_escena):
        _, ruta_worlds = self.obtener_rutas_base()
        ruta_sdf = os.path.join(ruta_worlds, f'escena{numero_escena}.sdf')

        if not os.path.exists(ruta_sdf):
            self.get_logger().error(f"No se encontró el archivo SDF: {ruta_sdf}")
            return False

        self.detener_gazebo()
        time.sleep(2)

        print(f"\nLanzando Gazebo con ruta SDF: {ruta_sdf}")

        try:
            log_path = os.path.join(ruta_worlds, "gazebo_launch.log")
            log_file = open(log_path, "w")
            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = ruta_worlds + ":" + env.get("GZ_SIM_RESOURCE_PATH", "")

            kwargs = {
                "stdin": subprocess.DEVNULL,
                "stdout": log_file,
                "stderr": log_file,
            }

            if os.name != 'nt':
                kwargs.update({
                    "cwd": ruta_worlds,
                    "env": env,
                    "preexec_fn": os.setsid
                })

            self.gz_process = subprocess.Popen(["gz", "sim", "-r", ruta_sdf], **kwargs)

            print(f"✅ Gazebo lanzado con escena {numero_escena} en autoplay.")
            return True

        except Exception as e:
            self.get_logger().error(f"Error al lanzar Gazebo: {e}")
            return False

    def lanzar_bridge(self):
        _, ruta_worlds = self.obtener_rutas_base()
        script_path = os.path.join(ruta_worlds, "bridge_gz.sh")

        if not os.path.exists(script_path):
            self.get_logger().error(f"No se encontró bridge_gz.sh en: {script_path}")
            return False

        self.detener_bridge()
        print("\nLanzando bridge ROS2")

        try:
            log_path = os.path.join(ruta_worlds, "bridge.log")
            log_file = open(log_path, "w")

            env = os.environ.copy()
            env["GZ_SIM_RESOURCE_PATH"] = ruta_worlds + ":" + env.get("GZ_SIM_RESOURCE_PATH", "")

            if os.name == 'nt':
                self.get_logger().error("bridge_gz.sh requiere bash (usar Linux o WSL)")
                return False

            self.bridge_process = subprocess.Popen(
                ["bash", script_path],
                cwd=ruta_worlds,
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=log_file,
                preexec_fn=os.setsid
            )

            print("✅ Bridge lanzado correctamente.")
            return True

        except Exception as e:
            self.get_logger().error(f"Error al lanzar el bridge: {e}")
            return False

    ## CREAR MALLA EN ESCENA
    def buscar_celda_punto_cspace(self, x, y, matriz, dx, dy, ancho, alto):
        columnas = int(round(ancho / dx))
        filas = int(round(alto / dy))

        if not (0.0 <= x <= ancho and 0.0 <= y <= alto):
            return None, None, "FUERA"

        columna = min(int(x / dx), columnas - 1)
        fila_desde_abajo = min(int(y / dy), filas - 1)
        fila_desde_arriba = filas - 1 - fila_desde_abajo

        estado = matriz[fila_desde_arriba][columna]
        return fila_desde_arriba, columna, estado

    def nombre_estado_celda(self, estado):
        nombres = {
            "L": "Libre",
            "S": "Semilibre",
            "O": "Ocupada",
            "FUERA": "Fuera del workspace",
        }
        return nombres.get(estado, estado)

    def probar_cspace_escena_actual(self):
        if not self.texto_escena.strip():
            print("⚠️ Primero carga una escena TXT.")
            return

        try:
            entrada=input("Ingresa el lado del robot en metros (Enter para usar 0.5 por default): ").strip()

            if entrada == "":
                self.lado_robot=0.5
            else:
                self.lado_robot=float(entrada)
        except ValueError:
            print("Valor invalido, se usara por defecto 0.5")
            self.lado_robot=0.5

        resultado = generar_cspace_desde_texto_escena(
            self.texto_escena,
            lado_robot=self.lado_robot,
            delta_x=0.25,
            delta_y=0.25
        )

        filas = len(resultado["matriz"])
        columnas = len(resultado["matriz"][0]) if filas > 0 else 0

        ocupadas = sum(c == "O" for fila in resultado["matriz"] for c in fila)
        semilibres = sum(c == "S" for fila in resultado["matriz"] for c in fila)
        libres = sum(c == "L" for fila in resultado["matriz"] for c in fila)

        q0x, q0y, q0theta = resultado["q0"]
        qfx, qfy, qftheta = resultado["qf"]

        fila_i, col_i, estado_i = self.buscar_celda_punto_cspace(
            q0x, q0y,
            resultado["matriz"],
            resultado["delta_x"], resultado["delta_y"],
            resultado["ancho"], resultado["alto"]
        )

        fila_f, col_f, estado_f = self.buscar_celda_punto_cspace(
            qfx, qfy,
            resultado["matriz"],
            resultado["delta_x"], resultado["delta_y"],
            resultado["ancho"], resultado["alto"]
        )

        print("\n--- C-SPACE GENERADO ---")
        print(f"Dimensiones workspace: {resultado['ancho']} x {resultado['alto']}")
        print(f"Resolución: dx={resultado['delta_x']}, dy={resultado['delta_y']}")
        print(f"Lado robot: lado_robot={resultado['lado_robot']}")
        print(f"Filas: {filas}, Columnas: {columnas}")
        print(f"Número de obstáculos: {len(resultado['obstaculos'])}")
        print(f"Número de C-obstáculos: {len(resultado['c_obstaculos'])}")
        print(f"Celdas libres: {libres}")
        print(f"Celdas semilibres: {semilibres}")
        print(f"Celdas ocupadas: {ocupadas}")
        print(
            f"Límites C-space: "
            f"x[{resultado['c_x_min']:.3f}, {resultado['c_x_max']:.3f}] "
            f"y[{resultado['c_y_min']:.3f}, {resultado['c_y_max']:.3f}]"
        )

        print("\n--- CLASIFICACIÓN DE CONFIGURACIONES ---")
        print(
            f"q0 = ({q0x:.2f}, {q0y:.2f}, {q0theta:.1f}°) -> "
            f"fila={fila_i}, columna={col_i}, estado={self.nombre_estado_celda(estado_i)}"
        )
        print(
            f"qf = ({qfx:.2f}, {qfy:.2f}, {qftheta:.1f}°) -> "
            f"fila={fila_f}, columna={col_f}, estado={self.nombre_estado_celda(estado_f)}"
        )
        print("------------------------\n")

        self.cspace_resultado = resultado

    def graficar_cspace_escena_actual(self):
        if not hasattr(self, "cspace_resultado") or self.cspace_resultado is None:
            print("⚠️ Primero genera el C-space de la escena.")
            return

        mostrar_cspace(self.cspace_resultado)



    # =======================================================
    # WRAPPERS PARA LOS ESTUDIANTES
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancia_direccion(self, direccion):
        mapa_direcciones = {
            'frente': 0.0,
            'izquierda': 90.0,
            'derecha': 270.0,
            'atras': 180.0
        }

        direccion = direccion.lower()
        if direccion not in mapa_direcciones:
            self.get_logger().error(f"Dirección '{direccion}' no válida.")
            return float('inf')

        return self.leer_distancia_en_angulo(mapa_direcciones[direccion])

    def leer_distancias_en_rango(self, grados_min, grados_max):
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        if self.target_theta_relativo is None:
            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos)

        cmd, completado = calcular_rotacion(
            self.current_theta,
            self.target_theta_relativo,
            tolerancia=tolerancia
        )
        self.cmd_pub.publish(cmd)

        if completado:
            self.target_theta_relativo = None

        return completado

    def mover_relativo(self, distancia_x_metros, distancia_y_metros, cono_vision=30, dist_segura=0.3, vel_lineal=0.4):
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = True
            self.tiempo_maniobra = 0.0

        if abs(distancia_x_metros) >= abs(distancia_y_metros):
            if distancia_x_metros >= 0:
                cono_despejado = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                cono_despejado = (
                    self.leer_distancias_en_rango(180 - cono_vision, 180)
                    + self.leer_distancias_en_rango(-180, -180 + cono_vision)
                )
        else:
            if distancia_y_metros > 0:
                cono_despejado = self.leer_distancias_en_rango(90 - cono_vision, 90 + cono_vision)
            else:
                cono_despejado = self.leer_distancias_en_rango(270 - cono_vision, 270 + cono_vision)

        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra,
            distancia_x_metros,
            distancia_y_metros,
            cono_despejado,
            dist_segura=dist_segura,
            vel_lineal=vel_lineal
        )
        self.cmd_pub.publish(cmd)
        self.tiempo_maniobra += 0.1

        if estado in ['COMPLETADO', 'BLOQUEADO']:
            self.pose_inicial_relativa = None
            self.tiempo_maniobra = 0.0

        return estado

    # =======================================================
    # ESCENAS Y SIMULADOR
    # =======================================================
    def cargar_escena_txt(self, numero_escena):
        ruta_data, _ = self.obtener_rutas_base()
        ruta_txt = os.path.join(ruta_data, f'Escena-Problema{numero_escena}.txt')

        if not os.path.exists(ruta_txt):
            self.get_logger().error(f"No se encontró el archivo TXT: {ruta_txt}")
            return False

        try:
            with open(ruta_txt, 'r', encoding='utf-8') as archivo:
                self.texto_escena = archivo.read()

            print(f"\n✅ Escena de texto {numero_escena} cargada correctamente.")
            print(f"--- Resumen Escena {numero_escena} ---")
            lineas = self.texto_escena.splitlines()
            print("\n".join(lineas[:10]))
            print("---------------------------\n")
            return True

        except Exception as e:
            self.get_logger().error(f"Error al leer la escena TXT: {e}")
            return False

    def iniciar_escena_completa(self, numero_escena):
        self.odom_recibido = False
        self.scan_recibido = False
        self.esperando_conexion = False
        self.tiempo_inicio_espera = None

        if not self.lanzar_gazebo_escena(numero_escena):
            return

        time.sleep(3)

        if not self.lanzar_bridge():
            return

        time.sleep(3)

        if not self.cargar_escena_txt(numero_escena):
            return

        self.esperando_conexion = True
        self.tiempo_inicio_espera = time.time()

        print("⏳ Esperando conexión con el robot...")

        while self.esperando_conexion and rclpy.ok():
            time.sleep(0.2)

        if self.mensaje_pendiente:
            print(f"{self.mensaje_pendiente}\n")
            self.mensaje_pendiente = None

    # =======================================================
    # MENÚ
    # =======================================================
    def menu_interactivo(self):
        while rclpy.ok():
            if self.comando_activo is not None:
                time.sleep(0.1)
                continue

            if self.mensaje_pendiente:
                print(f"\n{self.mensaje_pendiente}\n")
                self.mensaje_pendiente = None

            print("\n" + "=" * 35)
            print("--- MENÚ DE NAVEGACIÓN ---")
            print("1. Leer distancia en un ángulo")
            print("2. Leer distancias en un rango")
            print("3. Rotar grados relativos")
            print("4. Mover relativo a la posición (X, Y)")
            print("5. Cargar e iniciar simulador con escena")
            print("6. Leer distancia por dirección (Frente, Atras, Izquierda, Derecha)")
            print("7. Lanzar Gazebo con escena SDF")
            print("8. Lanzar bridge ROS2 <-> Gazebo")
            print("9. Generar C-space desde escena TXT")
            print("10. Graficar malla C-space")
            print("=" * 35)

            try:
                opcion = input("Elige una opción (1-10): ")

                if opcion == '1':
                    angulo = float(input("Ingresa el ángulo (en grados): "))
                    self.parametros_comando = [angulo]
                    self.comando_activo = 1

                elif opcion == '2':
                    ang_min = float(input("Ángulo mínimo (ej. -30): "))
                    ang_max = float(input("Ángulo máximo (ej. 30): "))
                    self.parametros_comando = [ang_min, ang_max]
                    self.comando_activo = 2

                elif opcion == '3':
                    grados = float(input("¿Cuántos grados quieres rotar? (Positivo=Izq, Negativo=Der): "))
                    self.parametros_comando = [grados]
                    self.comando_activo = 3

                elif opcion == '4':
                    x = float(input("Cuánto avanzar en X (Frente/Atrás) [en metros]: "))
                    y = float(input("Cuánto avanzar en Y (Izquierda/Derecha) [en metros]: "))
                    self.parametros_comando = [x, y]
                    self.comando_activo = 4

                elif opcion == '5':
                    numero = int(input("Ingresa el número de la escena (1-6): "))
                    self.iniciar_escena_completa(numero)

                elif opcion == '6':
                    direccion = input("¿Qué dirección? (frente, atras, izquierda, derecha): ").strip().lower()
                    if direccion in ['frente', 'atras', 'izquierda', 'derecha']:
                        self.parametros_comando = [direccion]
                        self.comando_activo = 6
                    else:
                        print("Dirección no válida. Intenta de nuevo.")

                elif opcion == '7':
                    numero = int(input("Ingresa el número de la escena SDF (1-6): "))
                    self.lanzar_gazebo_escena(numero)

                elif opcion == '8':
                    self.lanzar_bridge()

                elif opcion == '9':
                    self.probar_cspace_escena_actual()

                elif opcion == '10':
                    self.graficar_cspace_escena_actual()

                else:
                    print("Opción no válida. Intenta de nuevo.")

            except ValueError:
                print("Por favor, ingresa únicamente números válidos.")

    # =======================================================
    # CONTROL LOOP
    # =======================================================
    def control_loop(self):
        if self.esperando_conexion:
            if self.gz_process and self.gz_process.poll() is not None:
                self.mensaje_pendiente = "❌ Gazebo se cerró inesperadamente."
                self.esperando_conexion = False

            elif self.bridge_process and self.bridge_process.poll() is not None:
                self.mensaje_pendiente = "❌ El bridge se cerró inesperadamente."
                self.esperando_conexion = False

            elif self.odom_recibido and self.scan_recibido:
                self.mensaje_pendiente = "✅ Robot conectado en simulador"
                self.esperando_conexion = False

            elif time.time() - self.tiempo_inicio_espera > 15.0:
                self.mensaje_pendiente = "⚠️ No se pudo confirmar conexión con el robot"
                self.esperando_conexion = False

        if self.last_scan is None:
            return

        if self.comando_activo == 1:
            dist = self.leer_distancia_en_angulo(self.parametros_comando[0])
            self.get_logger().info(f"Distancia a {self.parametros_comando[0]}°: {dist:.2f} m")
            self.comando_activo = None

        elif self.comando_activo == 2:
            distancias = self.leer_distancias_en_rango(self.parametros_comando[0], self.parametros_comando[1])
            self.get_logger().info(f"Distancias detectadas: {distancias}")
            self.comando_activo = None

        elif self.comando_activo == 3:
            if self.rotar_relativo(self.parametros_comando[0]):
                self.get_logger().info("Rotación completada exitosamente.")
                self.comando_activo = None

        elif self.comando_activo == 4:
            estado = self.mover_relativo(self.parametros_comando[0], self.parametros_comando[1])

            if estado == 'COMPLETADO':
                self.get_logger().info("Desplazamiento relativo completado.")
                self.comando_activo = None
            elif estado == 'BLOQUEADO':
                self.get_logger().warn("¡Obstáculo detectado! Ruta bloqueada. Abortando movimiento.")
                self.comando_activo = None

        elif self.comando_activo == 6:
            direccion = self.parametros_comando[0]
            dist = self.leer_distancia_direccion(direccion)
            self.get_logger().info(f"Distancia hacia el {direccion.upper()}: {dist:.2f} m")
            self.comando_activo = None


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.detener_bridge()
        node.detener_gazebo()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()