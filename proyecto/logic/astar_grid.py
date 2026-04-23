import heapq


# Orientaciones:
# 0 = Este, 1 = Norte, 2 = Oeste, 3 = Sur
DELTA_AVANCE = {
    0: (0, 1),    # Este  -> misma fila, columna + 1
    1: (-1, 0),   # Norte -> fila - 1
    2: (0, -1),   # Oeste -> columna - 1
    3: (1, 0),    # Sur   -> fila + 1
}


def theta_a_orientacion(theta_grados: float) -> int:
    theta = theta_grados % 360

    if theta == 0:
        return 0   # Este
    if theta == 90:
        return 1   # Norte
    if theta == 180:
        return 2   # Oeste
    if theta == 270:
        return 3   # Sur

    raise ValueError(
        f"Ángulo {theta_grados} no soportado. "
        "Solo se admiten 0, 90, 180, 270."
    )


def orientacion_a_texto(o: int) -> str:
    return {
        0: "Este",
        1: "Norte",
        2: "Oeste",
        3: "Sur",
    }.get(o, str(o))


def distancia_a_obstaculo_mas_cercano(matriz, fila, col):
    """
    Distancia Manhattan a la celda ocupada ('O') más cercana.
    """
    filas = len(matriz)
    columnas = len(matriz[0]) if filas > 0 else 0

    mejor = float("inf")

    for f in range(filas):
        for c in range(columnas):
            if matriz[f][c] == "O":
                d = abs(fila - f) + abs(col - c)
                if d < mejor:
                    mejor = d

    return mejor


def costo_avanzar_con_clearance(matriz, fila, col):
    """
    Menor distancia a obstáculos => mayor costo.
    Esto hace que las celdas mas cercanas a los obstáculos tengan mas costo, y asi se evita pasar 
    muy cerca de los obstaculos aunque se pueda, priorizando seguridad del robot.
    """
    d = distancia_a_obstaculo_mas_cercano(matriz, fila, col)

    if d <= 1:
        return 6.0
    if d == 2:
        return 4.0
    if d == 3:
        return 2.0
    return 1.0


def heuristica(estado, meta):
    """
    Heurística Manhattan sobre fila/columna.
    Ignora orientación para mantenerlo simple y admisible.
    """
    fila, col, _ = estado
    fila_meta, col_meta, _ = meta
    return abs(fila - fila_meta) + abs(col - col_meta)


def es_transitable(matriz, fila, col, estados_transitables=("L",)):
    filas = len(matriz)
    columnas = len(matriz[0]) if filas > 0 else 0

    if not (0 <= fila < filas and 0 <= col < columnas):
        return False

    return matriz[fila][col] in estados_transitables


def reconstruir_camino(padres, estado_final):
    camino = []
    estado = estado_final

    while estado is not None:
        camino.append(estado)
        estado = padres.get(estado)

    camino.reverse()
    return camino


def vecinos_estado(estado, matriz, estados_transitables=("L",)):
    fila, col, orient = estado
    vecinos = []

    vecinos.append((
        (fila, col, (orient + 1) % 4),
        "GIRAR_IZQUIERDA",
        1
    ))

    vecinos.append((
        (fila, col, (orient - 1) % 4),
        "GIRAR_DERECHA",
        1
    ))

    df, dc = DELTA_AVANCE[orient]
    nueva_fila = fila + df
    nueva_col = col + dc

    if es_transitable(matriz, nueva_fila, nueva_col, estados_transitables):
        costo_avance = costo_avanzar_con_clearance(matriz, nueva_fila, nueva_col)
        vecinos.append(((nueva_fila, nueva_col, orient), "AVANZAR", costo_avance))

    return vecinos


def astar_orientado(matriz, inicio, meta, estados_transitables=("L",)):
    """
    A* sobre estados (fila, columna, orientación).
    """
    abiertos = []
    heapq.heappush(abiertos, (heuristica(inicio, meta), 0, inicio))

    padres = {inicio: None}
    accion_hasta = {inicio: None}
    costo_g = {inicio: 0}

    cerrados = set()

    while abiertos:
        _, g_actual, actual = heapq.heappop(abiertos)

        if actual in cerrados:
            continue

        cerrados.add(actual)

        if actual == meta:
            camino = reconstruir_camino(padres, actual)
            acciones = [accion_hasta[estado] for estado in camino][1:]
            return {
                "camino": camino,
                "acciones": acciones,
                "costo": g_actual,
            }

        for vecino, accion, costo in vecinos_estado(actual, matriz, estados_transitables):
            nuevo_g = g_actual + costo

            if vecino in cerrados:
                continue

            if vecino not in costo_g or nuevo_g < costo_g[vecino]:
                costo_g[vecino] = nuevo_g
                padres[vecino] = actual
                accion_hasta[vecino] = accion
                f = nuevo_g + heuristica(vecino, meta)
                heapq.heappush(abiertos, (f, nuevo_g, vecino))

    return None