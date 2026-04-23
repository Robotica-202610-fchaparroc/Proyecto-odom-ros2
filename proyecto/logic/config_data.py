import numpy as np

EPS = 1e-9

# Valores de configuracion por defecto
ANCHO_DEFECTO = 20.0
ALTO_DEFECTO = 16.0

ROBOT_DEFECTO = np.array([
    [0.0, 1.0],
    [-1.0, -1.0],
    [1.0, -1.0]
], dtype=float)

OBSTACULO_DEFECTO = np.array([
    [8.0, 4.0],
    [11.0, 3.5],
    [13.0, 6.0],
    [11.5, 9.0],
    [8.5, 8.0]
], dtype=float)

THETA_GRADOS_DEFECTO = 30.0

DELTA_X_DEFECTO = 1.0
DELTA_Y_DEFECTO = 1.0

X_INICIAL_DEFECTO = 3.0
Y_INICIAL_DEFECTO = 3.0

X_FINAL_DEFECTO = 16.0
Y_FINAL_DEFECTO = 12.0