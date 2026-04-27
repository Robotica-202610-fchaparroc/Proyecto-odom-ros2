import os
import math
from .routes import obtener_ruta_repo

# =======================================================
## Reporte
# =======================================================
def informar_estado_final_qf(self):
    if self.cspace_resultado is None or self.last_scan is None:
        print("⚠️ No hay datos suficientes para informar qf.")
        return

    resultado = self.cspace_resultado
    ancho = resultado["ancho"]
    alto = resultado["alto"]
    dFrente_teorico=resultado["dFrente"]
    dDerecha_teorico=resultado["dDerecha"]

    # --- qf teórica ---
    qfx, qfy, qftheta = resultado["qf"]

    # --- qf estimada (odometría) ---
    qf_est_x = self.current_x
    qf_est_y = self.current_y
    qf_est_theta = math.degrees(self.current_theta)

    # --- mediciones LiDAR ---
    d_frente = self.leer_distancia_en_angulo(0.0)
    d_derecha = self.leer_distancia_en_angulo(270.0)
    d_atras = self.leer_distancia_en_angulo(180.0)
    d_izquierda = self.leer_distancia_en_angulo(90.0)

    # --- qact (real) ---
    qact_x = ancho - d_derecha
    qact_y = alto - d_frente
    qact_theta = qftheta

    print("\n=========== RESULTADOS EN qf ===========")
    print("\n(a) Configuración teórica:")
    print(f"qf = ({qfx:.3f}, {qfy:.3f}, {qftheta:.1f}°)")

    print("\n(b) Configuración estimada (odometría):")
    print(f"qf-est = ({qf_est_x:.3f}, {qf_est_y:.3f}, {qf_est_theta:.1f}°)")

    print("\n(c) Configuración real (LiDAR):")
    print(f"qact = ({qact_x:.3f}, {qact_y:.3f}, {qact_theta:.1f}°)")

    print(f"\n- Distancias medidas con LIDAR (Teorica, dFrente: {dFrente_teorico}, dDerecha: {dDerecha_teorico})-")
    print(f"dFrente   = {d_frente:.3f} m")
    print(f"dDerecha  = {d_derecha:.3f} m")

    print("\n--- Errores ---")
    print(f"Error odometría: dx={qf_est_x - qfx:.3f}, dy={qf_est_y - qfy:.3f}")
    print(f"Error LiDAR:     dx={qact_x - qfx:.3f}, dy={qact_y - qfy:.3f}")

    print("========================================\n")


def guardar_camino_txt(self):
    if self.plan_resultado is None or self.cspace_resultado is None:
        print("⚠️ No hay plan para guardar.")
        return

    if not hasattr(self, "escena_seleccionada"):
        print("⚠️ No se conoce la escena actual.")
        return

    ruta_repo = obtener_ruta_repo(self)
    ruta_results = os.path.join(ruta_repo, "results")
    os.makedirs(ruta_results, exist_ok=True)

    nombre_archivo = f"camino_escena{self.escena_seleccionada}.txt"
    ruta_archivo = os.path.join(ruta_results, nombre_archivo)

    resultado = self.cspace_resultado
    dx = resultado["delta_x"]
    dy = resultado["delta_y"]
    alto = resultado["alto"]

    def celda_a_xy(fila, col):
        # centro de la celda
        x = (col + 0.5) * dx
        y = alto - (fila + 0.5) * dy
        return x, y

    with open(ruta_archivo, "w", encoding="utf-8") as f:
        f.write("# Camino geométrico A*\n")

        for estado in self.plan_resultado["camino"]:
            fila, col, orient = estado
            x, y = celda_a_xy(fila, col)

            theta_deg = {
                0: 0,    # Este
                1: 90,   # Norte
                2: 180,  # Oeste
                3: 270   # Sur
            }[orient]

            f.write(f"{x:.3f},{y:.3f},{theta_deg}\n")

    print(f"✅ Camino geométrico guardado en: {ruta_archivo}")

def completar_camino_txt_con_resultados(self, nombre_archivo="camino_plan.txt"):
    if self.cspace_resultado is None or self.last_scan is None:
        print("⚠️ No hay datos para completar el archivo.")
        return

    if not hasattr(self, "escena_seleccionada"):
        print("⚠️ No se conoce la escena actual.")
        return

    ruta_repo = obtener_ruta_repo(self)
    ruta_results = os.path.join(ruta_repo, "results")

    nombre_archivo = f"camino_escena{self.escena_seleccionada}.txt"
    ruta_archivo = os.path.join(ruta_results, nombre_archivo)

    if not os.path.exists(ruta_archivo):
        print("⚠️ No existe el archivo del camino.")
        return

    resultado = self.cspace_resultado
    ancho = resultado["ancho"]
    alto = resultado["alto"]

    # qf teórica
    qfx, qfy, qftheta = resultado["qf"]

    # qf-est (odometría)
    qf_est_x = self.current_x
    qf_est_y = self.current_y
    qf_est_theta = math.degrees(self.current_theta)

    # mediciones LiDAR
    d_frente = self.leer_distancia_en_angulo(0.0)
    d_derecha = self.leer_distancia_en_angulo(270.0)

    # qact (según enunciado, robot orientado a 90°)
    qact_x = ancho - d_derecha
    qact_y = alto - d_frente
    qact_theta = qftheta

    with open(ruta_archivo, "a", encoding="utf-8") as f:
        f.write("\n# --- RESULTADOS FINALES ---\n")
        f.write(f"qf_teorica,{qfx:.3f},{qfy:.3f},{qftheta:.1f}\n")
        f.write(f"qf_estimada,{qf_est_x:.3f},{qf_est_y:.3f},{qf_est_theta:.1f}\n")
        f.write(f"qact_real,{qact_x:.3f},{qact_y:.3f},{qact_theta:.1f}\n")

    print(f"✅ Archivo actualizado con qf-est y qact. {ruta_archivo}")
