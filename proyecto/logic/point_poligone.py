from .geometry import asegurar_antihorario
from .config_data import EPS

# Punto en poligono y C-space
def punto_sobre_segmento(punto, a, b, eps=EPS):
    px, py = punto
    ax, ay = a
    bx, by = b

    area_doble = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    if abs(area_doble) > eps:
        return False

    producto = (px - ax) * (px - bx) + (py - ay) * (py - by)
    return producto <= eps

def punto_en_frontera_poligono(punto, poligono, eps=EPS):
    n = len(poligono)
    for i in range(n):
        a = poligono[i]
        b = poligono[(i + 1) % n]
        if punto_sobre_segmento(punto, a, b, eps):
            return True
    return False

def punto_dentro_poligono_convexo(punto, poligono, eps=EPS):
    poligono = asegurar_antihorario(poligono)
    for i in range(len(poligono)):
        p1 = poligono[i]
        p2 = poligono[(i + 1) % len(poligono)]

        arista = p2 - p1
        vector = punto - p1

        cruz = arista[0] * vector[1] - arista[1] * vector[0]

        if cruz < -eps:
            return False

    return True

def punto_dentro_o_frontera_poligono(punto, poligono, eps=EPS):
    return punto_en_frontera_poligono(punto, poligono, eps) or punto_dentro_poligono_convexo(punto, poligono, eps)
