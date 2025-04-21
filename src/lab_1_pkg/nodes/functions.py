# Archivo de funciones, no es un nodo, solo contiene funciones que se pueden importar y usar en otros nodos


def get_distance(region, threshold):
    """Esta funcion recibira una matriz correspondiente a la region y el umbral de deteccion, debera retornar 1, si hay un obstaculo ren la region y 0 si no, se cuenta como obstaculo si al menos una celda por debajo del umbral"""

    try:
        for fila in region:
            for celda in fila:
                if celda < threshold:
                    return 1
        return 0
    except Exception as e:
        print(f'Error al obtener la distancia: {e}')
