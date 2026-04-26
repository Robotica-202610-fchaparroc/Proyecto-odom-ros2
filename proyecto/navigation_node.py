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
from .logic.movement import calcular_rotacion
from .logic.cspace_from_txt import generar_cspace_desde_texto_escena
from .logic.cspace_plot import mostrar_cspace, close_plot_grid
from .logic.astar_grid import (
    astar_orientado,
    theta_a_orientacion,
    orientacion_a_texto,
)


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
        self.lado_robot=0.3

        # Estado planificación A*
        self.plan_resultado = None
        self.plan_en_ejecucion = False
        self.acciones_plan_pendientes = []
        self.accion_plan_actual = None

        # Estado robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None

        # Estado movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None
        self.tiempo_maniobra = 0.0
        self.distancia_objetivo_relativa = None
        self.sentido_movimiento_relativo = None

        # Escena cargada
        self.texto_escena = ""

        # Estado menú/comandos
        self.comando_activo = None
        self.parametros_comando = []

        # ROS interfaces
        # self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom_cov', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer principal
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil iniciado.")

        # Menú en hilo aparte
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()


    # =======================================================
    # CALLBACKS ROS2 Subscriptores
    # =======================================================
    def odom_callback(self, msg):
        self.odom_recibido = True

        # Posición
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Orientación (yaw)
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
        package_share = get_package_share_directory('proyecto')
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

        close_plot_grid()

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

    # =======================================================
    ## CREAR MALLA EN ESCENA
    # =======================================================
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
        
        resultado = generar_cspace_desde_texto_escena(
            self.texto_escena,
            lado_robot=self.lado_robot,
            delta_x=0.1,
            delta_y=0.1
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

        plan = self.plan_resultado if hasattr(self, "plan_resultado") else None
        mostrar_cspace(self.cspace_resultado, plan=plan)

    # =======================================================
    ## Metodo planificación A*
    # =======================================================
    def planificar_ruta_astar(self):
        if self.cspace_resultado is None:
            print("⚠️ Primero genera el C-space.")
            return

        resultado = self.cspace_resultado
        matriz = resultado["matriz"]

        q0x, q0y, q0theta = resultado["q0"]
        qfx, qfy, qftheta = resultado["qf"]

        fila_i, col_i, estado_i = self.buscar_celda_punto_cspace(
            q0x, q0y,
            matriz,
            resultado["delta_x"], resultado["delta_y"],
            resultado["ancho"], resultado["alto"]
        )

        fila_f, col_f, estado_f = self.buscar_celda_punto_cspace(
            qfx, qfy,
            matriz,
            resultado["delta_x"], resultado["delta_y"],
            resultado["ancho"], resultado["alto"]
        )

        if estado_i != "L":
            print(f"⚠️ q0 no está en celda libre: {estado_i}")
            return

        if estado_f != "L":
            print(f"⚠️ qf no está en celda libre: {estado_f}")
            return

        try:
            orient_inicio = theta_a_orientacion(q0theta)
            orient_meta = theta_a_orientacion(qftheta)
        except ValueError as e:
            print(f"⚠️ {e}")
            return

        inicio = (fila_i, col_i, orient_inicio)
        meta = (fila_f, col_f, orient_meta)

        plan = astar_orientado(matriz, inicio, meta, estados_transitables=("L",))

        if plan is None:
            print("❌ No se encontró ruta con A*.")
            self.plan_resultado = None
            return

        self.plan_resultado = plan

        print("\n--- RUTA A* ENCONTRADA ---")
        print(f"Costo total: {plan['costo']}")
        print(f"Número de estados: {len(plan['camino'])}")
        print("Acciones:")

        for i, accion in enumerate(plan["acciones"], start=1):
            print(f"  {i}. {accion}")

        estado_final = plan["camino"][-1]
        print(
            f"Estado final: fila={estado_final[0]}, col={estado_final[1]}, "
            f"orientación={orientacion_a_texto(estado_final[2])}"
        )
        print("--------------------------\n")

    def compactar_acciones_plan(self, acciones):
        acciones_compactadas = []
        contador_avances = 0

        for accion in acciones:
            if accion == "AVANZAR":
                contador_avances += 1
            else:
                if contador_avances > 0:
                    acciones_compactadas.append(("AVANZAR", contador_avances))
                    contador_avances = 0

                acciones_compactadas.append((accion, 1))

        if contador_avances > 0:
            acciones_compactadas.append(("AVANZAR", contador_avances))

        return acciones_compactadas

    def ejecutar_plan_astar(self):
        if self.plan_resultado is None:
            print("⚠️ Primero planifica una ruta con A*.")
            return

        if self.cspace_resultado is None:
            print("⚠️ Primero genera el C-space.")
            return

        acciones = self.plan_resultado.get("acciones", [])
        if not acciones:
            print("⚠️ El plan no tiene acciones para ejecutar.")
            return

        # Importante: para mover una celda, asumimos delta_x == delta_y
        dx = self.cspace_resultado["delta_x"]
        dy = self.cspace_resultado["delta_y"]

        if abs(dx - dy) > 1e-9:
            print("⚠️ Para esta ejecución simple se requiere delta_x == delta_y.")
            return

        self.acciones_plan_pendientes = self.compactar_acciones_plan(acciones)
        self.accion_plan_actual = None
        self.plan_en_ejecucion = True

        print(f"\n▶ Ejecutando plan A* con {len(acciones)} acciones...\n")

    def ejecutar_siguiente_accion_plan(self):
        if not self.plan_en_ejecucion:
            return

        if self.accion_plan_actual is None:
            if not self.acciones_plan_pendientes:
                self.plan_en_ejecucion = False
                self.cmd_pub.publish(Twist())

                print("\n✅ Plan ejecutado completamente.\n")

                # volver a mostrar menú
                self.mostrar_menu()

                return

            self.accion_plan_actual = self.acciones_plan_pendientes.pop(0)
            accion, repeticiones = self.accion_plan_actual
            self.get_logger().info(f"Ejecutando acción: {accion} x{repeticiones}")

        accion, repeticiones = self.accion_plan_actual
        dx = self.cspace_resultado["delta_x"]

        if accion == "GIRAR_IZQUIERDA":
            completado = self.rotar_relativo(90.0)
            if completado:
                self.accion_plan_actual = None

        elif accion == "GIRAR_DERECHA":
            completado = self.rotar_relativo(-90.0)
            if completado:
                self.accion_plan_actual = None

        elif accion == "AVANZAR":
            distancia_total = dx * repeticiones

            estado = self.mover_relativo(
                distancia_total,
                0.0,
                vel_lineal=0.25
            )

            if estado == "COMPLETADO":
                self.accion_plan_actual = None

            elif estado == "BLOQUEADO":
                self.plan_en_ejecucion = False
                self.accion_plan_actual = None
                self.acciones_plan_pendientes = []
                self.cmd_pub.publish(Twist())
                print("\n⚠️ Plan abortado: obstáculo detectado.\n")


    # =======================================================
    # Comandos para movimiento de Robot
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        """Retorna la distancia (en metros) de un ángulo específico del Lidar."""
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancias_en_rango(self, grados_min, grados_max):
        """Retorna una lista con todas las detecciones en un rango visual."""
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        if self.target_theta_relativo is None:
            factor_correccion = 1.05

            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos * factor_correccion)

        cmd, completado = calcular_rotacion(
            self.current_theta,
            self.target_theta_relativo,
            tolerancia=tolerancia
        )

        self.cmd_pub.publish(cmd)

        if completado:
            self.cmd_pub.publish(Twist())
            self.target_theta_relativo = None

        return completado

    def mover_relativo(
        self,
        distancia_x_metros,
        distancia_y_metros,
        cono_vision=30,
        dist_segura=0.3,
        vel_lineal=0.3
    ):
        dist_total = math.sqrt(distancia_x_metros**2 + distancia_y_metros**2)

        if dist_total < 0.001:
            self.cmd_pub.publish(Twist())
            return "COMPLETADO"

        # Inicializar movimiento
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = (self.current_x, self.current_y)
            self.distancia_objetivo_relativa = dist_total

            self.sentido_movimiento_relativo = (
                distancia_x_metros / dist_total,
                distancia_y_metros / dist_total
            )

            self.get_logger().info(
                f"[MOVER] Pose inicial fijada en "
                f"({self.current_x:.3f}, {self.current_y:.3f})"
            )

        x0, y0 = self.pose_inicial_relativa

        distancia_recorrida = math.hypot(
            self.current_x - x0,
            self.current_y - y0
        )

        self.get_logger().info(
            f"[MOVER] objetivo={self.distancia_objetivo_relativa:.3f}, "
            f"recorrida={distancia_recorrida:.3f}"
        )

        tolerancia_distancia = min(0.03, max(0.005, dist_total * 0.1))

        # ✔ Condición de llegada
        if distancia_recorrida >= self.distancia_objetivo_relativa - tolerancia_distancia:
            self.cmd_pub.publish(Twist())

            self.pose_inicial_relativa = None
            self.distancia_objetivo_relativa = None
            self.sentido_movimiento_relativo = None

            return "COMPLETADO"

        # ==========================
        # ANTICHOQUES DIRECCIONAL
        # ==========================
        sx, sy = self.sentido_movimiento_relativo

        if abs(sx) >= abs(sy):
            if sx >= 0:
                cono = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                cono = (
                    self.leer_distancias_en_rango(180 - cono_vision, 180)
                    + self.leer_distancias_en_rango(-180, -180 + cono_vision)
                )
        else:
            if sy > 0:
                cono = self.leer_distancias_en_rango(90 - cono_vision, 90 + cono_vision)
            else:
                cono = self.leer_distancias_en_rango(270 - cono_vision, 270 + cono_vision)

        min_dist = min(cono) if cono else float("inf")

        if min_dist <= dist_segura:
            self.cmd_pub.publish(Twist())

            self.pose_inicial_relativa = None
            self.distancia_objetivo_relativa = None
            self.sentido_movimiento_relativo = None

            return "BLOQUEADO"

        # ✔ Movimiento
        cmd = Twist()
        cmd.linear.x = sx * vel_lineal
        cmd.linear.y = sy * vel_lineal
        self.cmd_pub.publish(cmd)

        return "EN_RUTA"


    # =======================================================
    # Cargar escena sdf y txt
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
    def mostrar_menu(self):
        print("\n" + "=" * 35)
        print("--- MENÚ DE NAVEGACIÓN ---")
        print("1. Cargar e iniciar simulador con escena")
        print("2. Generar C-space desde escena TXT, Planificar ruta con A* y Graficar malla C-space")
        print("3. Ejecutar plan A* en el robot")
        print("4. Salir")
        print("=" * 35)
        print("Elige una opción (1-4): ", end="", flush=True)

    def menu_interactivo(self):
        while rclpy.ok():
            if self.comando_activo is not None:
                time.sleep(0.1)
                continue

            if self.mensaje_pendiente:
                print(f"\n{self.mensaje_pendiente}\n")
                self.mensaje_pendiente = None

            self.mostrar_menu()

            try:
                opcion = input()

                if opcion == '1':
                    numero = int(input("Ingresa el número de la escena (1-6): "))
                    self.iniciar_escena_completa(numero)

                elif opcion == '2':
                    self.probar_cspace_escena_actual()
                    self.planificar_ruta_astar()
                    self.graficar_cspace_escena_actual()

                elif opcion == '3':
                    self.ejecutar_plan_astar()

                elif opcion == "4":
                    confirm = input("¿Seguro que quieres salir? (s/n): ").strip().lower()

                    if confirm == "s":
                        print("Saliendo del programa...")

                        self.cmd_pub.publish(Twist())
                        self.plan_en_ejecucion = False
                        self.accion_plan_actual = None
                        self.acciones_plan_pendientes = []

                        rclpy.shutdown()
                        return

                    else:
                        print("Cancelado. Volviendo al menú.\n")

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
                self.mensaje_pendiente = (
                    f"✅ Robot conectado en simulador"
                    f"(odom={self.odom_recibido}, scan={self.scan_recibido})"
                )
                self.esperando_conexion = False

            elif time.time() - self.tiempo_inicio_espera > 15.0:
                self.mensaje_pendiente = (
                    f"⚠️ No se pudo confirmar conexión con el robot "
                    f"(odom={self.odom_recibido}, scan={self.scan_recibido})"
                )
                self.esperando_conexion = False

        if self.last_scan is None:
            return

        if self.plan_en_ejecucion:
            self.ejecutar_siguiente_accion_plan()
            return


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