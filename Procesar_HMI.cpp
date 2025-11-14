// HMIManager.h
#ifndef HMI_MANAGER_H
#define HMI_MANAGER_H

#include <Controllino.h>
#include "Stone_HMI_Define.h"

#define MAX_COLA_RESPUESTAS 10
#define MAX_WIDGETS         10

// Inicializar cola y registros
void HMI_init();
HMI_ProtocolMsg hmi_msg;

// Solicita y devuelve el último valor conocido (no bloqueante)
float HMI_get_value(const char* tipo, const char* nombre);

#endif


#include "Procesar_HMI.h"

// Cola circular de respuestas
static HMI_ProtocolMsg cola_resp[MAX_COLA_RESPUESTAS];
static int cola_in = 0, cola_out = 0;

// Registro dinámico de widgets
struct Registro {
  const char* tipo;
  const char* nombre;
  bool        pendiente;
  float       valor;
};
static Registro registros[MAX_WIDGETS];
static int num_registros = 0;

// Prototipos internos
static void _enqueueResponse();
static void _processOneResponse();

void HMI_init() {
  // Función para inicializar los objetos y las variables para el control de colas
  cola_in = cola_out = 0;
  num_registros = 0;
  // No inicializa registros: valores a 0 y pendientes a false
  for(int i=0;i<MAX_WIDGETS;i++){
    registros[i].tipo = nullptr;
    registros[i].nombre = nullptr;
    registros[i].pendiente = false;
    registros[i].valor = 0;
  }
}

float HMI_get_value(const char* tipo, const char* nombre) {
  /*Función que:
    1) Busca si un widget se encuentra ya en un registro, caso
    contrario, crea el registro de ese widge.

    2) Encola las respuestas del HMI

    3) Si el valor de un objeto aún está pendiente, entonces lo solicuta, y
    lo marca como pendiente. 

    4) Procesa la respuestas

    5) Devuelve el último valor conocido para el widget solicitado.

    Recibe: 
      - const char* tipo   : Tipo de widget, hasta el momento solo slider y spinbox
      - const char* nombre : Nombre del widget que requiere obtener el valor
    Retorna:
      - El valor flotante del widget solicitado solo si está disponible 

  */
  // 1) Buscar el registro del widget solicitado o crear registro
  int idx = -1;
  for(int i=0;i<num_registros;i++){
    // Compara el nombre del registro actual con el nombre del widget solicitado
    if(strcmp(registros[i].nombre, nombre)==0) { idx=i; break; }
  }

  // Crea el registro en caso de no existir (Es decir, llena los datos de una estructura)
  if(idx<0 && num_registros<MAX_WIDGETS) {
    idx = num_registros++;
    registros[idx].tipo = tipo;
    registros[idx].nombre = nombre;
    registros[idx].pendiente = false;
    registros[idx].valor = 0;
  }
  if(idx<0) return 0;  // sin espacio

  // 2) Encolar respuestas nuevas
  _enqueueResponse();

  // 3) Solicitar si no pendiente
  if(!registros[idx].pendiente) {
    Stone_HMI_Get_Value(tipo, nombre);
    registros[idx].pendiente = true;
  }

  // 4) Procesar una respuesta
  _processOneResponse();

  // 5) Devolver último valor conocido
  return registros[idx].valor;
}

static void _enqueueResponse() {
  /*
    Función que se encarga de agregar los mensajes recibidos del HMI
    a la cola circular (un array de estructuras tipo HMI_ProtocolMsg).

    - Llama a Uart_HMI_Data_Receive(), que llena la variable global hmi_msg si hay datos.
    - Si el mensaje recibido tiene longitud válida (len > 0):
      - Calcula el siguiente índice de escritura (next) de forma circular.
      - Si la cola no está llena (next ≠ cola_in), guarda el mensaje en la cola.
      - Actualiza cola_out al siguiente índice.
      - Finalmente, borra la longitud de hmi_msg (len = 0) para marcar que ya fue consumido.
  */
  Uart_HMI_Data_Receive(); //
  if(hmi_msg.len>0){ // Si es que hay una respuesta del HMI
    int next = (cola_out+1)%MAX_COLA_RESPUESTAS; // Cola circular
    if(next!=cola_in) { // Si la cola no está llena
      cola_resp[cola_out] = hmi_msg;  // Agrega un nuevo elemeno a la cola (es decir, al array de estructuras)
      cola_out = next;
    }
    hmi_msg.len=0; // Se elimina el mensaje recibido porque ya se guardó en el array de estructuras
  }
}

static void _processOneResponse() {
  /*
    Función encargada de procesar una sola respuesta almacenada en la cola circular.
    Extrae el valor devuelto por el HMI, lo convierte y lo asigna al widget correspondiente.

    - Si la cola está vacía (cola_in == cola_out), no hace nada y retorna.
    - Extrae el mensaje más antiguo de la cola (cola_resp[cola_in]).
    - Avanza el puntero de lectura (cola_in) de forma circular.

    - Reconstruye el nombre del widget a partir de los primeros bytes del mensaje.
      (resp.len - 4) indica que los últimos 4 bytes contienen el valor, no el nombre.

    - Busca en los registros si hay un widget pendiente con ese nombre.
      Si lo encuentra:
      - Extrae los últimos 4 bytes (el valor) del mensaje y los convierte a `float` o `uint32_t`,
        dependiendo del tipo de comando (`cmd`).
        * Si el comando es 0x10A1 → se interpreta como un entero sin signo.
        * Si es otro comando → se interpreta como número flotante (float).
      - Guarda el valor en el campo `.valor` del registro correspondiente.
      - Marca el registro como ya no pendiente (`pendiente = false`).
      - Sale del ciclo (`break`) ya que solo se espera una coincidencia.
  */

  if(cola_in==cola_out) return;
  HMI_ProtocolMsg resp = cola_resp[cola_in];
  cola_in = (cola_in+1)%MAX_COLA_RESPUESTAS;

  // extraer nombre
  String nombreRec;
  for(int i=0;i<resp.len-4;i++) nombreRec += (char)resp.data[i];

  // buscar y actualizar
  for(int i=0;i<num_registros;i++){
    if(registros[i].pendiente && nombreRec==registros[i].nombre){
      uint8_t buf[4] = { resp.data[resp.len-1], resp.data[resp.len-2], resp.data[resp.len-3], resp.data[resp.len-4] };
      float v; uint32_t vi;
      if(resp.cmd==0x10A1) { 
        memcpy(&vi,buf,4); 
        v=float(vi); 
      }else { 
        memcpy(&v,buf,4); 
      }
      registros[i].valor=v;
      registros[i].pendiente=false;
      break;
    }
  }
}
