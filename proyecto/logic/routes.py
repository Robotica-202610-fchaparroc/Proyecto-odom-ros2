import os
from ament_index_python.packages import get_package_share_directory

def obtener_rutas_base_install(self):
    # Carpeta install/proyecto/shared
    package_share = get_package_share_directory('proyecto')
    ruta_data = os.path.join(package_share, 'data')
    ruta_worlds = os.path.join(package_share, 'worlds')
    return ruta_data, ruta_worlds

def obtener_ruta_repo(self):
    ruta = get_package_share_directory('proyecto')

    while not os.path.exists(os.path.join(ruta, "setup.py")):
        ruta = os.path.dirname(ruta)

    return ruta

