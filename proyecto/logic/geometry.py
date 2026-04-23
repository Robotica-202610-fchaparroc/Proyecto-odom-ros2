import numpy as np

def rotar_puntos(puntos, angulo):
    c = np.cos(angulo)
    s = np.sin(angulo)
    matriz = np.array([
        [c, -s],
        [s,  c]
    ])
    return (matriz @ puntos.T).T

def trasladar_puntos(puntos, tx, ty):
    return puntos + np.array([tx, ty])

def cerrar_poligono(poligono):
    return np.vstack([poligono, poligono[0]])

def es_antihorario(poligono):
    suma = 0.0
    for i in range(len(poligono)):
        x1, y1 = poligono[i]
        x2, y2 = poligono[(i + 1) % len(poligono)]
        suma += x1 * y2 - x2 * y1
    return suma > 0

def asegurar_antihorario(poligono):
    if es_antihorario(poligono):
        return poligono
    return poligono[::-1]

def obtener_aristas(poligono):
    aristas = []
    for i in range(len(poligono)):
        siguiente = (i + 1) % len(poligono)
        aristas.append(poligono[siguiente] - poligono[i])
    return np.array(aristas)

def producto_cruz(a, b):
    return a[0] * b[1] - a[1] * b[0]

def suma_minkowski(poligono1, poligono2):
    poligono1 = asegurar_antihorario(poligono1)
    poligono2 = asegurar_antihorario(poligono2)

    indice1 = np.lexsort((poligono1[:, 0], poligono1[:, 1]))[0]
    indice2 = np.lexsort((poligono2[:, 0], poligono2[:, 1]))[0]

    poligono1 = np.roll(poligono1, -indice1, axis=0)
    poligono2 = np.roll(poligono2, -indice2, axis=0)

    aristas1 = obtener_aristas(poligono1)
    aristas2 = obtener_aristas(poligono2)

    i = 0
    j = 0

    resultado = [poligono1[0] + poligono2[0]]

    while i < len(aristas1) or j < len(aristas2):
        if i == len(aristas1):
            resultado.append(resultado[-1] + aristas2[j])
            j += 1
        elif j == len(aristas2):
            resultado.append(resultado[-1] + aristas1[i])
            i += 1
        else:
            cruz = producto_cruz(aristas1[i], aristas2[j])

            if cruz > 0:
                resultado.append(resultado[-1] + aristas1[i])
                i += 1
            elif cruz < 0:
                resultado.append(resultado[-1] + aristas2[j])
                j += 1
            else:
                resultado.append(resultado[-1] + aristas1[i] + aristas2[j])
                i += 1
                j += 1

        if i == len(aristas1) and j == len(aristas2):
            break

    return np.array(resultado[:-1])