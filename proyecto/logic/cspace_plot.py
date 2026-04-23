import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PoligonoMpl, Rectangle


import numpy as np


def recortar_rectangulo_al_workspace(poligono, ancho, alto, eps=1e-9):
    """
    Recorta un rectángulo alineado a ejes al workspace [0, ancho] x [0, alto].

    Retorna un dict con:
    - {"tipo": "rect", "puntos": np.ndarray}
    - {"tipo": "linea_vertical", "x": x, "y1": y1, "y2": y2}
    - {"tipo": "linea_horizontal", "y": y, "x1": x1, "x2": x2}
    - None si no hay intersección
    """
    xs = poligono[:, 0]
    ys = poligono[:, 1]

    x_min = max(0.0, float(np.min(xs)))
    x_max = min(ancho, float(np.max(xs)))
    y_min = max(0.0, float(np.min(ys)))
    y_max = min(alto, float(np.max(ys)))

    ancho_clip = x_max - x_min
    alto_clip = y_max - y_min

    # Sin intersección
    if ancho_clip < -eps or alto_clip < -eps:
        return None

    # Rectángulo visible
    if ancho_clip > eps and alto_clip > eps:
        return {
            "tipo": "rect",
            "puntos": np.array([
                [x_min, y_min],
                [x_max, y_min],
                [x_max, y_max],
                [x_min, y_max],
            ], dtype=float)
        }

    # Línea vertical
    if abs(ancho_clip) <= eps and alto_clip > eps:
        return {
            "tipo": "linea_vertical",
            "x": x_min,
            "y1": y_min,
            "y2": y_max,
        }

    # Línea horizontal
    if abs(alto_clip) <= eps and ancho_clip > eps:
        return {
            "tipo": "linea_horizontal",
            "y": y_min,
            "x1": x_min,
            "x2": x_max,
        }

    # Punto degenerado
    if abs(ancho_clip) <= eps and abs(alto_clip) <= eps:
        return {
            "tipo": "punto",
            "x": x_min,
            "y": y_min,
        }

    return None

def dibujar_robot(eje, centro, lado, color):
    x, y = centro
    mitad = lado / 2.0
    rect = Rectangle(
        (x - mitad, y - mitad),
        lado,
        lado,
        fill=False,
        edgecolor=color,
        linewidth=2
    )
    eje.add_patch(rect)

def close_plot_grid():
    print(f"Cerrando Gráfico anterior con malla, si exite")
    plt.close()

def graficar_cspace_discretizado_multi(resultado, plan=None):
    """
    Dibuja la malla discretizada del C-space usando el diccionario
    retornado por generar_cspace_desde_texto_escena(...).
    """

    ancho = resultado["ancho"]
    alto = resultado["alto"]
    lado_robot = resultado["lado_robot"]
    delta_x = resultado["delta_x"]
    delta_y = resultado["delta_y"]
    matriz = resultado["matriz"]
    c_obstaculos = resultado["c_obstaculos"]
    c_x_min = resultado["c_x_min"]
    c_x_max = resultado["c_x_max"]
    c_y_min = resultado["c_y_min"]
    c_y_max = resultado["c_y_max"]
    q0x, q0y, _ = resultado["q0"]
    qfx, qfy, _ = resultado["qf"]

    close_plot_grid()
    figura, eje = plt.subplots(figsize=(8, 7))

    eje.set_title("C-space discretizado")
    eje.set_xlabel("X (m)")
    eje.set_ylabel("Y (m)")
    eje.set_xlim(0, ancho)
    eje.set_ylim(0, alto)
    eje.set_aspect("equal", adjustable="box")

    eje.set_xticks(np.arange(0, ancho + 0.5, 0.5))
    eje.set_yticks(np.arange(0, alto + 0.5, 0.5))
    eje.grid(True, color="#b0b0b0", linewidth=0.6, alpha=0.6)
    eje.tick_params(axis="both", which="both", labelsize=9)
    eje.spines['bottom'].set_linewidth(1.5)
    eje.spines['left'].set_linewidth(1.5)

    # Pintar celdas
    for fila_desde_arriba in range(len(matriz)):
        y0 = alto - (fila_desde_arriba + 1) * delta_y

        for columna in range(len(matriz[0])):
            x0 = columna * delta_x
            estado = matriz[fila_desde_arriba][columna]

            if estado == "L":
                color = "#ffffff"      # blanco
            elif estado == "S":
                color = "#bfbfbf"      # gris medio
            else:
                color = "#000000"      # negro

            rectangulo = Rectangle(
                (x0, y0),
                delta_x,
                delta_y,
                facecolor=color,
                edgecolor="#d0d0d0",
                linewidth=0.2
            )
            eje.add_patch(rectangulo)

    # Límite válido del C-space
    eje.add_patch(
        Rectangle(
            (c_x_min, c_y_min),
            c_x_max - c_x_min,
            c_y_max - c_y_min,
            fill=False,
            linestyle="--",
            edgecolor="orange",
            linewidth=2,
            label="Límite C-space"
        )
    )

    # Dibujar obstáculos originales
    for i, obs in enumerate(resultado["obstaculos"]):
        obs_clip = recortar_rectangulo_al_workspace(obs, ancho, alto)

        if obs_clip is None:
            continue

        label = "Obstáculo real" if i == 0 else None

        if obs_clip["tipo"] == "rect":
            eje.add_patch(
                PoligonoMpl(
                    obs_clip["puntos"],
                    closed=True,
                    fill=False,
                    edgecolor="red",
                    linewidth=3,
                    linestyle="-",
                    label=label,
                    zorder=7
                )
            )

        elif obs_clip["tipo"] == "linea_vertical":
            eje.plot(
                [obs_clip["x"], obs_clip["x"]],
                [obs_clip["y1"], obs_clip["y2"]],
                color="red",
                linewidth=6,
                solid_capstyle="butt",
                label=label,
                zorder=8
            )

        elif obs_clip["tipo"] == "linea_horizontal":
            eje.plot(
                [obs_clip["x1"], obs_clip["x2"]],
                [obs_clip["y"], obs_clip["y"]],
                color="red",
                linewidth=6,
                solid_capstyle="butt",
                label=label,
                zorder=8
            )

        elif obs_clip["tipo"] == "punto":
            eje.plot(
                obs_clip["x"],
                obs_clip["y"],
                marker="s",
                markersize=6,
                color="red",
                label=label,
                zorder=8
            )

    # Dibujar todos los C-obstáculos
    for i, c_obs in enumerate(c_obstaculos):
        eje.add_patch(
            PoligonoMpl(
                c_obs,
                closed=True,
                fill=False,
                edgecolor="cyan",
                linewidth=2.5,
                alpha=0.9,
                label="C-obstáculo" if i == 0 else None
            )
        )

    # Dibujar q0 y qf
    eje.plot(q0x, q0y, marker="o", markersize=8, color="green", label="q0")
    eje.plot(qfx, qfy, marker="x", markersize=8, color="blue", label="qf")
    # Dibujar el robot como cuadrado en q0 y qf
    dibujar_robot(eje, (q0x, q0y), lado_robot, "green")
    dibujar_robot(eje, (qfx, qfy), lado_robot, "blue")

    # Dibujar ruta A* si existe solución
    if plan is not None and "camino" in plan and plan["camino"]:
        puntos_x = []
        puntos_y = []

        ultima_celda = None

        for fila, columna, _orientacion in plan["camino"]:
            celda_actual = (fila, columna)

            if celda_actual == ultima_celda:
                continue

            x, y = celda_a_coordenada_centro(
                fila,
                columna,
                delta_x,
                delta_y,
                alto
            )
            puntos_x.append(x)
            puntos_y.append(y)

            ultima_celda = celda_actual

        q0x, q0y, _ = resultado["q0"]
        qfx, qfy, _ = resultado["qf"]

        if puntos_x and puntos_y:
            puntos_x[0] = q0x
            puntos_y[0] = q0y
            puntos_x[-1] = qfx
            puntos_y[-1] = qfy

        eje.plot(
            puntos_x,
            puntos_y,
            color="magenta",
            linewidth=2.5,
            label="Ruta A*",
            zorder=10
        )

        # Dibujar orientación del robot en el path hasta qf
        for fila, col, orient in plan["camino"][::3]:  # cada 3 puntos la flecha
            x, y = celda_a_coordenada_centro(fila, col, delta_x, delta_y, alto)

            dx_dir, dy_dir = {
                0: (1, 0),    # Este
                1: (0, 1),    # Norte
                2: (-1, 0),   # Oeste
                3: (0, -1),   # Sur
            }[orient]

            eje.arrow(
                x, y,
                dx_dir * 0.2,
                dy_dir * 0.2,
                head_width=0.05,
                color="purple",
                zorder=11
            )



    eje.legend(
        loc="upper left",
        bbox_to_anchor=(1.02, 1),  # Cuadro labels fuera del gráfico
        borderaxespad=0
    )
    return figura, eje


def mostrar_cspace(resultado, plan=None):
    graficar_cspace_discretizado_multi(resultado, plan=plan)
    plt.show(block=False)
    plt.pause(0.001)

def celda_a_coordenada_centro(fila, columna, dx, dy, alto):
    """
    Convierte una celda de la matriz (fila desde arriba, columna)
    al centro geométrico de la celda en coordenadas del workspace.
    """
    x = columna * dx + dx / 2.0
    y = alto - (fila + 1) * dy + dy / 2.0
    return x, y