from threading import Thread
from time import sleep

def contador(nombre):
    for i in range(10):
        print(f"Soy el hilo {nombre} contando {i}")
        sleep(1)

class Manager:


    def __init__(self, cantidad):
        self.cantidad = cantidad
        self.lista_hilos = []
        for i in range(cantidad):
            nombre = "hilo_"+str(i)
            hilo = Thread(target=contador, args=[i])
            self.lista_hilos.append(hilo)

manejador = Manager(3)
for i in range(3):
    manejador.lista_hilos[i].start()


