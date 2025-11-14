#ifndef HMI_MANAGER_H
#define HMI_MANAGER_H

#include <Controllino.h>
#include "Stone_HMI_Define.h"

// Configuraciones para el tamaño máximo de la cola y de los widgets
#define MAX_COLA_RESPUESTAS 10
#define MAX_WIDGETS         10

// Funciones principales de la librería
void HMI_init();                          // Inicializa las estructuras de datos
float HMI_get_value(const char* tipo, const char* nombre);  // Solicita el valor de un widget

#endif // HMI_MANAGER_H
