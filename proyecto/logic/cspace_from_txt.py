import math
from typing import Dict, List, Tuple

import numpy as np

from .geometry import rotar_puntos, suma_minkowski
from .c_space import calcular_limites_cspace
from .point_poligone import punto_dentro_o_frontera_poligono


EstadoCelda = str
Punto = Tuple[float, float]


def parsear_escena_txt(texto_escena: str) -> Dict:
    """
    Convierte el texto de la escena en una estructura de datos usable.

    Formato esperado:
    Dimensiones,4.0,5.0
    q0,0.75,0.75,0.0
    qf,3.25,4.25,90
    Obstaculos,4
    Obstaculo1_Pto1,3.0,0.5
    Obstaculo1_Pto2,3.5,1.0
    ...
    """
    lineas = [ln.strip() for ln in texto_escena.splitlines() if ln.strip()]
    datos = {}
    obstaculos_tmp = {}

    for linea in lineas:
        partes = [p.strip() for p in linea.split(",")]
        clave = partes[0]

        if clave == "Dimensiones":
            datos["ancho"] = float(partes[1])
            datos["alto"] = float(partes[2])

        elif clave == "q0":
            datos["q0"] = (float(partes[1]), float(partes[2]), float(partes[3]))

        elif clave == "qf":
            datos["qf"] = (float(partes[1]), float(partes[2]), float(partes[3]))

        elif clave == "dFrente":
            datos["dFrente"] = float(partes[1])

        elif clave == "dDerecha":
            datos["dDerecha"] = float(partes[1])

        elif clave == "Obstaculos":
            datos["num_obstaculos"] = int(partes[1])

        elif "_Pto1" in clave or "_Pto2" in clave:
            nombre, punto_id = clave.split("_")
            if nombre not in obstaculos_tmp:
                obstaculos_tmp[nombre] = {}

            obstaculos_tmp[nombre][punto_id] = (float(partes[1]), float(partes[2]))

    obstaculos = []
    for nombre in sorted(obstaculos_tmp.keys()):
        p1 = obstaculos_tmp[nombre].get("Pto1")
        p2 = obstaculos_tmp[nombre].get("Pto2")
        if p1 is None or p2 is None:
            raise ValueError(f"El obstáculo {nombre} no tiene Pto1 y Pto2 completos.")
        obstaculos.append(rectangulo_a_poligono(p1, p2))

    datos["obstaculos"] = obstaculos
    return datos


def rectangulo_a_poligono(p1: Punto, p2: Punto) -> np.ndarray:
    """
    Convierte dos esquinas opuestas de un rectángulo alineado a ejes
    en un polígono antihorario de 4 vértices.
    """
    x1, y1 = p1
    x2, y2 = p2

    x_min, x_max = min(x1, x2), max(x1, x2)
    y_min, y_max = min(y1, y2), max(y1, y2)

    return np.array([
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max],
    ], dtype=float)


def construir_robot_cuadrado(lado: float) -> np.ndarray:
    """
    Robot cuadrado centrado en el origen.
    """
    mitad = lado / 2.0
    return np.array([
        [-mitad, -mitad],
        [ mitad, -mitad],
        [ mitad,  mitad],
        [-mitad,  mitad],
    ], dtype=float)


def puntos_representativos_celda(x: float, y: float, dx: float, dy: float) -> np.ndarray:
    return np.array([
        [x, y],
        [x + dx, y],
        [x + dx, y + dy],
        [x, y + dy],
        [x + dx / 2.0, y + dy / 2.0],
    ], dtype=float)


def punto_invalido_multi(
    punto: np.ndarray,
    c_obstaculos: List[np.ndarray],
    c_x_min: float,
    c_x_max: float,
    c_y_min: float,
    c_y_max: float,
    eps: float = 1e-9,
) -> bool:
    x, y = float(punto[0]), float(punto[1])

    fuera_c = (x < c_x_min or x > c_x_max or y < c_y_min or y > c_y_max)
    en_frontera_c = (
        abs(x - c_x_min) <= eps
        or abs(x - c_x_max) <= eps
        or abs(y - c_y_min) <= eps
        or abs(y - c_y_max) <= eps
    )

    dentro_de_algun_c_obstaculo = any(
        punto_dentro_o_frontera_poligono(punto, c_obs) for c_obs in c_obstaculos
    )

    return fuera_c or en_frontera_c or dentro_de_algun_c_obstaculo


def clasificar_celda_multi(
    x: float,
    y: float,
    dx: float,
    dy: float,
    c_obstaculos: List[np.ndarray],
    c_x_min: float,
    c_x_max: float,
    c_y_min: float,
    c_y_max: float,
) -> EstadoCelda:
    puntos = puntos_representativos_celda(x, y, dx, dy)

    invalidos = [
        punto_invalido_multi(p, c_obstaculos, c_x_min, c_x_max, c_y_min, c_y_max)
        for p in puntos
    ]

    if all(invalidos):
        return "O"   # ocupada
    if not any(invalidos):
        return "L"   # libre
    return "S"       # semilibre


def construir_matriz_celdas_multi(
    ancho: float,
    alto: float,
    dx: float,
    dy: float,
    c_obstaculos: List[np.ndarray],
    c_x_min: float,
    c_x_max: float,
    c_y_min: float,
    c_y_max: float,
) -> List[List[EstadoCelda]]:
    columnas = int(round(ancho / dx))
    filas = int(round(alto / dy))

    matriz = []
    for fila_desde_arriba in range(filas):
        y0 = alto - (fila_desde_arriba + 1) * dy
        fila_actual = []

        for columna in range(columnas):
            x0 = columna * dx
            estado = clasificar_celda_multi(
                x0, y0, dx, dy,
                c_obstaculos,
                c_x_min, c_x_max, c_y_min, c_y_max
            )
            fila_actual.append(estado)

        matriz.append(fila_actual)

    return matriz


def generar_cspace_desde_texto_escena(
    texto_escena: str,
    lado_robot: float = 0.5,
    delta_x: float = 0.25,
    delta_y: float = 0.25,
):
    """
    Genera C-space discretizado a partir del .txt de escena.
    """
    escena = parsear_escena_txt(texto_escena)

    ancho = escena["ancho"]
    alto = escena["alto"]
    theta_grados = escena["q0"][2]

    robot = construir_robot_cuadrado(lado_robot)
    angulo = math.radians(theta_grados)

    robot_rotado = rotar_puntos(robot, angulo)
    robot_negativo = -robot_rotado

    c_obstaculos = [
        suma_minkowski(obstaculo, robot_negativo)
        for obstaculo in escena["obstaculos"]
    ]

    c_x_min, c_x_max, c_y_min, c_y_max = calcular_limites_cspace(
        ancho, alto, robot_rotado
    )

    matriz = construir_matriz_celdas_multi(
        ancho, alto,
        delta_x, delta_y,
        c_obstaculos,
        c_x_min, c_x_max, c_y_min, c_y_max
    )

    return {
        "ancho": ancho,
        "alto": alto,
        "lado_robot": lado_robot,
        "robot": robot,
        "robot_rotado": robot_rotado,
        "q0": escena["q0"],
        "qf": escena["qf"],
        "obstaculos": escena["obstaculos"],
        "c_obstaculos": c_obstaculos,
        "c_x_min": c_x_min,
        "c_x_max": c_x_max,
        "c_y_min": c_y_min,
        "c_y_max": c_y_max,
        "delta_x": delta_x,
        "delta_y": delta_y,
        "matriz": matriz,
    }