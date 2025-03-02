const char* FIRMWARE_VERSION = "1.0.0";

// ================== INCLUDES ==================
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <WebServer.h>
#include <ESPmDNS.h>  // Para resolución de nombres mDNS (ya no es necesario incluirlo dos veces)
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Preferences.h>  // Biblioteca para almacenamiento persistente
#include "esp_timer.h"

// Instancia de Preferences para almacenamiento
Preferences preferenciasTK2;
// ================== DEFINICIÓN DE PINES ==================
#define TRIG_PIN 13        // Pin trigger sensor ultrasónico
#define ECHO_PIN 14        // Pin echo sensor ultrasónico
#define FLOAT_SWITCH_2 34  // Pin flotador TdireccionIPBackupK2
#define RESET_BUTTON 4     // Pin botón reset WiFiManager
#define LED_STATUS 15      // GPIO15 - LED de comexion wifi

// ================== CONSTANTES DEL SISTEMA ==================
// ================== CONSTANTES DEL SISTEMA ==================
// Constantes para el tanque y mediciones
const int ALTURA_TANQUE = 100;         // Altura máxima del tanque en cm
const float MAX_INSTANT_CHANGE = 5.0;  // Máximo cambio repentino permitido en cm
const int VENTANA_PROMEDIO = 6;        // Tamaño de la ventana de mediciones

// Constantes para manejo de WiFi y conexión
const int MAX_INTENTOS = 3;                       // Máximo intentos de reconexión
const unsigned long WIFI_CHECK_INTERVAL = 30000;  // Revisar WiFi cada 30 segundos
const unsigned long WIFI_RETRY_DELAY = 5000;      // Esperar 5 segundos entre intentos
const int MAX_WIFI_RECONNECT_ATTEMPTS = 3;        // Máximo de intentos antes de reiniciar
const int WDT_TIMEOUT = 30;                       // Watchdog de 30 segundos

// Direcciones y nombres de red
const char* nombreServidor = "tanque-principal";  // Nombre mDNS del servidor
char direccionIPBackup[16] = "192.168.100.205";   // IP de respaldo

// Constantes para el LED
#define LED_STATE_ON 1              // Estado LED constantemente encendido
#define LED_STATE_BLINKING 2        // Estado LED parpadeando
const int LED_BLINK_NORMAL = 2000;  // Parpadeo lento: 2 segundos
// No redefinir LED_BLINK_ERROR si ya está definido como macro
#ifndef LED_BLINK_ERROR
const int LED_BLINK_ERROR = 300;  // Parpadeo rápido: 300ms
#endif

// ================== OBJETOS DEL SISTEMA ==================
WebServer server(80);            // Servidor web en puerto 80
WiFiClient clienteConexion;      // Cliente global para probar conexión
HTTPClient httpClienteConexion;  // Cliente HTTP global para conexiones

// ================== VARIABLES DE MEDICIÓN ==================
float medidasVentana[VENTANA_PROMEDIO];  // Array para ventana de mediciones
float nivelTK2 = 0;                      // Nivel actual del tanque en litros
float nivelActual = 0;                   // Nivel para el endpoint status
float ultimaMedicionDistancia = 0;       // Última medición válida en cm

// ================== VARIABLES DE ESTADO DEL TANQUE ==================
bool flotadorTK2 = false;             // Estado del flotador
bool flotadorActivoConexion = false;  // Estado del flotador para la conexión
float distanciaConexion = 0;          // Distancia para enviar en la conexión
bool medicionesIniciadas = false;     // Control de inicio de mediciones
int indiceVentana = 0;                // Índice actual en ventana de mediciones

// ================== VARIABLES PARA CONTROL DEL LED ==================
esp_timer_handle_t timer_handle = NULL;      // Handle para el timer del LED
volatile bool ledFisicoEncendido = false;    // Estado físico actual del LED
int ledEstadoActual = LED_STATE_BLINKING;    // Estado lógico del LED (ON o BLINKING)
int velocidadParpadeoActual = 100;           // Velocidad actual del parpadeo en ms
volatile bool comunicacionFallando = false;  // Indica si no hay respuesta del servidor
bool ledParpadeoActivo = false;              // Estado de activación del parpadeo
unsigned long ultimoParpadeo = 0;            // Último cambio de estado del LED
int intervaloParpadeo = 300;                 // Intervalo de parpadeo en milisegundos

// ================== VARIABLES PARA MÁQUINA DE ESTADOS WIFI ==================
typedef enum {
  WIFI_RECONEXION_INACTIVA,
  WIFI_RECONEXION_INICIANDO,
  WIFI_RECONEXION_ESPERANDO,
  WIFI_RECONEXION_VERIFICANDO
} WiFiReconexionEstado;

WiFiReconexionEstado estadoReconexionWiFi = WIFI_RECONEXION_INACTIVA;
unsigned long tiempoInicioReconexionWiFi = 0;  // Tiempo en que inició la reconexión
int intentosReconexionWiFi = 0;                // Contador de intentos de reconexión
int wifiReconnectAttempts = 0;                 // Contador adicional para reconexión

// ================== VARIABLES PARA MÁQUINA DE ESTADOS DE CONEXIÓN ==================
typedef enum {
  CONEXION_INACTIVA,
  CONEXION_PROBANDO_HOSTNAME,
  CONEXION_PROBANDO_IP,
  CONEXION_ENVIANDO_HOSTNAME,
  CONEXION_ENVIANDO_IP,
  CONEXION_COMPLETADA
} ConexionEstado;

ConexionEstado estadoConexion = CONEXION_INACTIVA;  // Estado actual de la máquina de estados
unsigned long tiempoInicioConexion = 0;             // Tiempo en que inició la conexión
bool resultadoConexion = false;                     // Resultado de la última conexión

// ================== TEMPORIZADORES Y TIEMPOS ==================
unsigned long ultimaMedicion = 0;          // Tiempo de la última medición
unsigned long ultimoEnvioServidor = 0;     // Tiempo del último envío al servidor
unsigned long lastWiFiCheck = 0;           // Último chequeo de conexión WiFi
unsigned long ultimaRespuestaExitosa = 0;  // Tiempo de última respuesta del servidor
unsigned long ultimaActualizacionLed = 0;  // Última actualización del LED

// ================== ESTRUCTURAS DE DATOS ==================
struct ValidacionCambio {
  float nuevasLecturas[4];        // Buffer para nuevas lecturas
  int contadorLecturas = 0;       // Contador de lecturas consistentes
  bool enValidacion = false;      // Indica si estamos validando un cambio
  float ultimoValorValidado = 0;  // Último valor validado
};

ValidacionCambio validacionTK2;  // Estructura para validar cambios en TK2
// Default speed (faster blink)
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

void IRAM_ATTR alternarLED(void* arg);

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// TERCERO: Función de callback del timer que alterna el LED
void IRAM_ATTR alternarLED(void* arg) {
  ledFisicoEncendido = !ledFisicoEncendido;
  digitalWrite(LED_STATUS, ledFisicoEncendido);
}

// CUARTO: Función para iniciar parpadeo
// Modifica iniciarParpadeo() para ser más eficiente
void iniciarParpadeo(int intervalo) {
  // Si ya estamos parpadeando a esta velocidad, no hacer nada
  if (ledEstadoActual == LED_STATE_BLINKING && velocidadParpadeoActual == intervalo) {
    return;
  }

  ledEstadoActual = LED_STATE_BLINKING;
  velocidadParpadeoActual = intervalo;

  // Detener timer existente si lo hay, pero sin recrearlo innecesariamente
  if (timer_handle) {
    esp_timer_stop(timer_handle);
  } else {
    // Solo crear el timer si no existe
    esp_timer_create_args_t timer_args = {
      .callback = &alternarLED,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,  // Mantener el valor original
      .name = "LED_Timer"
    };
    esp_timer_create(&timer_args, &timer_handle);
  }

  // Reiniciar el timer con la nueva velocidad
  esp_timer_start_periodic(timer_handle, intervalo * 1000);  // ms a µs

  // Asegurar que el LED esté visible inmediatamente
  ledFisicoEncendido = true;
  digitalWrite(LED_STATUS, HIGH);

  Serial.printf("LED: Modo parpadeo iniciado (intervalo: %d ms)\n", intervalo);
}

// QUINTO: Función para LED fijo
void ledFijo(bool encendido) {
  if (encendido) {
    ledEstadoActual = LED_STATE_ON;
    // Detener el timer si está funcionando, pero no eliminarlo
    if (timer_handle) {
      esp_timer_stop(timer_handle);
    }
    // Encender el LED
    digitalWrite(LED_STATUS, HIGH);
    ledFisicoEncendido = true;
    Serial.println("LED: Modo fijo encendido");
  } else {
    // Si alguien intenta apagar el LED completamente, usamos parpadeo en su lugar
    iniciarParpadeo(100);
    Serial.println("LED: Intento de apagado interceptado, cambiado a parpadeo rápido");
  }
}

// SEXTO: Función para actualizar estado del LED en loop()
void actualizarEstadoLED() {
  if (WiFi.status() != WL_CONNECTED) {
    // Sin conexión WiFi - parpadeo rápido
    if (ledEstadoActual != LED_STATE_BLINKING || velocidadParpadeoActual != 100) {
      iniciarParpadeo(100);
    }
  } else if (comunicacionFallando) {
    // WiFi conectado pero falla comunicación con servidor - parpadeo medio
    if (ledEstadoActual != LED_STATE_BLINKING || velocidadParpadeoActual != 300) {
      iniciarParpadeo(300);
    }
  } else {
    // Todo operacional - LED fijo
    if (ledEstadoActual != LED_STATE_ON) {
      ledFijo(true);
    }
  }
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

float validarMedicion(float nuevaMedida, ValidacionCambio& validacion) {
  // Si es la primera medición
  if (validacion.ultimoValorValidado == 0) {
    validacion.ultimoValorValidado = nuevaMedida;
    return nuevaMedida;
  }

  float cambioActual = abs(nuevaMedida - validacion.ultimoValorValidado);

  // Si detectamos un cambio brusco
  if (cambioActual > MAX_INSTANT_CHANGE) {
    if (!validacion.enValidacion) {
      validacion.enValidacion = true;
      validacion.contadorLecturas = 0;
      for (int i = 0; i < 4; i++) {
        validacion.nuevasLecturas[i] = 0;
      }
    }

    validacion.nuevasLecturas[validacion.contadorLecturas] = nuevaMedida;
    validacion.contadorLecturas++;

    if (validacion.contadorLecturas >= 3) {
      bool lecturas_consistentes = true;
      float suma = 0;

      for (int i = 0; i < validacion.contadorLecturas; i++) {
        suma += validacion.nuevasLecturas[i];
        if (i > 0 && abs(validacion.nuevasLecturas[i] - validacion.nuevasLecturas[i - 1]) > 2.0) {
          lecturas_consistentes = false;
        }
      }

      if (lecturas_consistentes) {
        float promedioNuevo = suma / validacion.contadorLecturas;
        Serial.printf("Cambio de posición confirmado: %.1f cm -> %.1f cm\n",
                      validacion.ultimoValorValidado, promedioNuevo);
        validacion.ultimoValorValidado = promedioNuevo;
        validacion.enValidacion = false;
        return promedioNuevo;
      }

      if (validacion.contadorLecturas >= 4) {
        validacion.enValidacion = false;
      }
    }

    Serial.printf("Validando cambio: %.1f cm (lectura %d/3)\n",
                  nuevaMedida, validacion.contadorLecturas);
    return nuevaMedida;
  }

  if (validacion.enValidacion) {
    validacion.enValidacion = false;
  }

  validacion.ultimoValorValidado = nuevaMedida;
  return nuevaMedida;
}

float medirNivelTK2() {
  static float ultimaDistanciaValida = 0;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 23529);
  float distancia = 0;

  if (duracion > 300 && duracion < 4900) {
    distancia = (duracion * 0.0343) / 2;
    Serial.printf("Lectura - Duration: %ld μs, Distancia: %.1f cm (Válida)\n",
                  duracion, distancia);

    distancia = validarMedicion(distancia, validacionTK2);
    distancia = constrain(distancia, 0, ALTURA_TANQUE);

    ultimaDistanciaValida = distancia;
    return distancia;
  } else if (duracion > 0) {
    Serial.printf("Lectura - Duration: %ld μs (Fuera de rango)\n", duracion);
  } else {
    Serial.println("Lectura - Sin eco");
  }

  return ultimaDistanciaValida;
}

// Nueva función para iniciar el proceso de envío sin bloquear
void iniciarEnvioServidor(float distancia, bool flotadorActivo) {
  // Guardar los parámetros para usarlos en cada estado
  distanciaConexion = distancia;
  flotadorActivoConexion = flotadorActivo;

  // Cambiar para probar primero la IP
  estadoConexion = CONEXION_PROBANDO_IP;  // Empezar con IP para evitar la espera
  tiempoInicioConexion = millis();

  // Activar parpadeo del LED mientras se procesa
  iniciarParpadeo(true);

  comunicacionFallando = true;

  Serial.println("\n--- INICIANDO ENVÍO DE DATOS (NO BLOQUEANTE) ---");
  Serial.printf("• Nivel: %.1f cm\n", distancia);
  Serial.printf("• Estado flotante: %s\n",
                flotadorActivo ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");
}

// Función para procesar la máquina de estados de conexión
bool procesarConexion() {
  // Si no hay una conexión en progreso, no hacer nada
  if (estadoConexion == CONEXION_INACTIVA || estadoConexion == CONEXION_COMPLETADA) {
    return false;
  }

  unsigned long tiempoActual = millis();

  // Timeout general para todo el proceso (30 segundos)
  if (tiempoActual - tiempoInicioConexion > 30000) {
    Serial.println("Timeout general de conexión. Abortando.");
    estadoConexion = CONEXION_COMPLETADA;
    resultadoConexion = false;
    return true;
  }

  // Procesar según el estado actual
  switch (estadoConexion) {
    case CONEXION_PROBANDO_HOSTNAME:
      // Probar conectividad al hostname
      Serial.print("Probando conectividad a ");
      Serial.print(nombreServidor);
      Serial.print("...");

      clienteConexion.setTimeout(1000);
      if (clienteConexion.connect(nombreServidor, 80)) {
        Serial.println("Alcanzable");
        clienteConexion.stop();

        // Avanzar al envío usando hostname
        estadoConexion = CONEXION_ENVIANDO_HOSTNAME;
      } else {
        Serial.println("No alcanzable");
        clienteConexion.stop();

        // Avanzar a probar la IP de respaldo
        estadoConexion = CONEXION_PROBANDO_IP;
      }
      break;

    case CONEXION_PROBANDO_IP:
      // Probar conectividad a la IP de respaldo
      Serial.print("Probando conectividad a ");
      Serial.print(direccionIPBackup);
      Serial.print("...");

      clienteConexion.setTimeout(1000);
      if (clienteConexion.connect(direccionIPBackup, 80)) {
        Serial.println("Alcanzable");
        clienteConexion.stop();

        // Avanzar al envío usando IP
        estadoConexion = CONEXION_ENVIANDO_IP;
      } else {
        Serial.println("No alcanzable");
        clienteConexion.stop();

        // Finalizar con error
        Serial.println("No se pudo conectar ni al hostname ni a la IP de respaldo");
        estadoConexion = CONEXION_COMPLETADA;
        resultadoConexion = false;
        return true;
      }
      break;

    case CONEXION_ENVIANDO_HOSTNAME:
      // Enviar datos usando hostname
      {
        String url = "http://" + String(nombreServidor) + "/updateTK2";
        url += "?origen=TK2";
        url += "&nivel=" + String(distanciaConexion, 1);
        url += "&flotante=" + String(flotadorActivoConexion ? "1" : "0");

        Serial.println("Enviando: " + url);

        WiFiClient client;
        HTTPClient http;
        http.setTimeout(2000);
        http.begin(client, url);

        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
          String respuesta = http.getString();
          Serial.println("Respuesta: " + respuesta);

          // AQUÍ: Extraer y almacenar IP si está presente en la respuesta
          int posIP = respuesta.indexOf("TK1-IP:");
          if (posIP >= 0) {
            String ipStr = respuesta.substring(posIP + 7);
            // Limpiar la cadena y extraer solo la IP
            ipStr.trim();

            // Si hay otros datos después de la IP (como | Timestamp), cortarlos
            int posSep = ipStr.indexOf('|');
            if (posSep > 0) {
              ipStr = ipStr.substring(0, posSep);
              ipStr.trim();
            }

            // Almacenar la IP
            almacenarIPServidor(ipStr.c_str());
          }

          http.end();

          // Conexión exitosa
          ledFijo(true);
          comunicacionFallando = false;
          estadoConexion = CONEXION_COMPLETADA;
          resultadoConexion = true;
          return true;
        } else {
          Serial.printf("Error HTTP: %d\n", httpCode);
          http.end();

          // Falló usando hostname, intentar con IP
          estadoConexion = CONEXION_PROBANDO_IP;
        }
      }
      break;

    case CONEXION_ENVIANDO_IP:
      // Enviar datos usando IP de respaldo
      {
        String url = "http://" + String(direccionIPBackup) + "/updateTK2";
        url += "?origen=TK2";
        url += "&nivel=" + String(distanciaConexion, 1);
        url += "&flotante=" + String(flotadorActivoConexion ? "1" : "0");

        Serial.println("Enviando: " + url);

        WiFiClient client;
        HTTPClient http;
        http.setTimeout(2000);
        http.begin(client, url);

        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
          String respuesta = http.getString();
          Serial.println("Respuesta: " + respuesta);

          // AQUÍ: Extraer y almacenar IP si está presente en la respuesta
          int posIP = respuesta.indexOf("TK1-IP:");
          if (posIP >= 0) {
            String ipStr = respuesta.substring(posIP + 7);
            // Limpiar la cadena y extraer solo la IP
            ipStr.trim();

            // Si hay otros datos después de la IP (como | Timestamp), cortarlos
            int posSep = ipStr.indexOf('|');
            if (posSep > 0) {
              ipStr = ipStr.substring(0, posSep);
              ipStr.trim();
            }

            // Almacenar la IP
            almacenarIPServidor(ipStr.c_str());
          }

          http.end();

          // Conexión exitosa
          ledFijo(true);
          comunicacionFallando = false;
          estadoConexion = CONEXION_COMPLETADA;
          resultadoConexion = true;
          return true;
        } else {
          Serial.printf("Error HTTP: %d\n", httpCode);
          http.end();

          // Falló todo, terminar con error
          estadoConexion = CONEXION_COMPLETADA;
          resultadoConexion = false;
          return true;
        }
      }
      break;

    default:
      // Estado no reconocido,
      iniciarParpadeo(true);
      estadoConexion = CONEXION_INACTIVA;
      break;
  }

  // Todavía estamos en proceso, retornar false
  return false;
}

// Modificar la función procesarYEnviarDatos() para usar iniciarEnvioServidor
void procesarYEnviarDatos() {
  static unsigned long ultimaMedicion = 0;
  static int erroresConsecutivos = 0;
  static unsigned long tiempoEsperaReintento = 0;
  static bool primerEnvio = true;
  static bool ultimoEstadoEnviado = false;
  unsigned long tiempoActual = millis();

  // Si estamos procesando una conexión, continuar con ese proceso
  if (estadoConexion != CONEXION_INACTIVA && estadoConexion != CONEXION_COMPLETADA) {
    if (procesarConexion()) {
      // La conexión ha terminado, procesar el resultado
      if (resultadoConexion) {
        // Éxito
        primerEnvio = false;
        ultimoEstadoEnviado = flotadorActivoConexion;
        erroresConsecutivos = 0;
        nivelTK2 = distanciaConexion;
        ultimoEnvioServidor = tiempoActual;

        // Actualizar estado de comunicación
        comunicacionFallando = false;
        ultimaRespuestaExitosa = tiempoActual;

        Serial.println("✓ Comunicación con servidor EXITOSA");
        Serial.printf("→ Estado del flotante enviado: %s\n",
                      flotadorActivoConexion ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");
      } else {
        // Error
        erroresConsecutivos++;
        Serial.printf("✗ Error de envío #%d\n", erroresConsecutivos);

        // Mantener parpadeo para error
        comunicacionFallando = true;
        iniciarParpadeo(300);

        if (erroresConsecutivos >= 3) {
          Serial.println("⚠ ALERTA: Comunicación con servidor fallando");
          Serial.println("⚠ Estado del flotante NO SE ESTÁ ENVIANDO");
          tiempoEsperaReintento = tiempoActual + 5000;
          Serial.println("→ Próximo intento en 5 segundos...");
        }
      }

      // Resetear estado de conexión
      estadoConexion = CONEXION_INACTIVA;
    }

    // No continuar con el resto si hay una conexión en proceso
    return;
  }

  // Si estamos en periodo de espera después de errores
  if (erroresConsecutivos >= 3 && tiempoActual < tiempoEsperaReintento) {
    return;
  }

  // Medición cada 5 segundos
  if (tiempoActual - ultimaMedicion >= 5000) {
    float distancia = medirNivelTK2();
    ultimaMedicion = tiempoActual;

    if (distancia <= 0) {
      return;  // Lectura inválida
    }

    Serial.printf("\nMedición TK2: %.1f cm\n", distancia);
    ultimaMedicionDistancia = distancia;
    nivelActual = distancia;

    // Leer estado del flotador
    bool estadoActualFlotante = digitalRead(FLOAT_SWITCH_2) == LOW;

    // Diagnóstico del flotador
    Serial.printf("Estado ACTUAL del flotante: %s (Pin: %d, Valor: %d)\n",
                  estadoActualFlotante ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)",
                  FLOAT_SWITCH_2,
                  digitalRead(FLOAT_SWITCH_2));

    // Detectar cambios en el flotante
    if (estadoActualFlotante != flotadorTK2) {
      Serial.println("¡ALERTA! Cambio en estado del flotante detectado:");
      Serial.printf("  - Estado anterior: %s\n",
                    flotadorTK2 ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");
      Serial.printf("  - Nuevo estado: %s\n",
                    estadoActualFlotante ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");

      flotadorTK2 = estadoActualFlotante;
    }

    // Decidir si enviamos datos
    bool debeEnviar =
      primerEnvio ||                                    // Forzar el primer envío
      (tiempoActual - ultimoEnvioServidor >= 30000) ||  // Tiempo desde último envío
      (abs(distancia - nivelTK2) >= 5.0) ||             // Cambio significativo
      (estadoActualFlotante != ultimoEstadoEnviado);    // Cambio en último estado enviado

    // Mensaje especial para primer envío
    if (primerEnvio) {
      Serial.println("╔═══════════════════════════════════════════╗");
      Serial.println("║ PRIMER ENVÍO DESPUÉS DE INICIO/REINICIO   ║");
      Serial.println("╚═══════════════════════════════════════════╝");
      Serial.printf("→ Estado inicial del flotante: %s\n",
                    flotadorTK2 ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");
    }

    // Intentar envío solo si es necesario y WiFi está conectado
    if (debeEnviar && WiFi.status() == WL_CONNECTED) {
      // AQUÍ ES DONDE DEBE LLAMARSE A iniciarEnvioServidor
      iniciarEnvioServidor(distancia, flotadorTK2);
    }
  }
}

bool handleWiFiConnection() {
  static unsigned long lastPingTime = 0;
  static unsigned long lastReconnectAttempt = 0;
  static int failedPings = 0;
  static bool inReconnectionMode = false;
  static bool lastWiFiState = false;
  unsigned long currentMillis = millis();

  // Estado actual de WiFi
  bool currentWiFiState = (WiFi.status() == WL_CONNECTED);

  // Detectar cambios en el estado WiFi
  if (currentWiFiState != lastWiFiState) {
    if (!currentWiFiState) {
      // Perdimos WiFi - solo registramos el evento
      Serial.println("WiFi desconectado - LED cambiará a parpadeo rápido");
    } else {
      // Recuperamos WiFi - solo registramos el evento
      if (!comunicacionFallando) {
        Serial.println("WiFi conectado - LED cambiará a fijo encendido");
      } else {
        Serial.println("WiFi conectado pero con problemas de comunicación - LED en parpadeo normal");
      }
    }
    lastWiFiState = currentWiFiState;

    // No manipulamos directamente el LED, actualizarEstadoLED se encargará
  }

  // Manejar la máquina de estados de reconexión WiFi
  if (estadoReconexionWiFi != WIFI_RECONEXION_INACTIVA) {
    switch (estadoReconexionWiFi) {
      case WIFI_RECONEXION_INICIANDO:
        // Iniciar el proceso de reconexión
        Serial.println("Iniciando reconexión WiFi...");
        WiFi.disconnect(true);

        // Pasar al siguiente estado después de un breve tiempo
        estadoReconexionWiFi = WIFI_RECONEXION_ESPERANDO;
        tiempoInicioReconexionWiFi = currentMillis;

        // Configurar WiFi
        WiFi.mode(WIFI_STA);
        WiFi.setSleep(false);
        WiFi.begin();  // Usar credenciales guardadas
        break;

      case WIFI_RECONEXION_ESPERANDO:
        // Esperar un tiempo para que se conecte
        if (currentMillis - tiempoInicioReconexionWiFi >= 5000) {
          // Verificar si se conectó
          estadoReconexionWiFi = WIFI_RECONEXION_VERIFICANDO;
        }
        break;

      case WIFI_RECONEXION_VERIFICANDO:
        // Verificar si tuvimos éxito
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("¡WiFi reconectado exitosamente!");
          inReconnectionMode = false;

          // Configurar máxima potencia
          esp_wifi_set_max_tx_power(78);

          // Resetear contadores
          intentosReconexionWiFi = 0;
          estadoReconexionWiFi = WIFI_RECONEXION_INACTIVA;
          return true;
        } else {
          Serial.println("Intento de reconexión fallido");
          intentosReconexionWiFi++;

          if (intentosReconexionWiFi >= MAX_WIFI_RECONNECT_ATTEMPTS) {
            Serial.println("Demasiados intentos fallidos. Reiniciando dispositivo...");
            ESP.restart();
          }

          // Programar próximo intento
          estadoReconexionWiFi = WIFI_RECONEXION_INACTIVA;
          lastReconnectAttempt = currentMillis;
        }
        break;
    }

    // Mientras estamos en cualquier estado de reconexión, retornar false
    return false;
  }

  // Si estamos en modo de reconexión pero no hay proceso activo
  if (inReconnectionMode && estadoReconexionWiFi == WIFI_RECONEXION_INACTIVA) {
    // Verificar si es tiempo de intentar reconectar
    if (currentMillis - lastReconnectAttempt >= 15000) {  // Cada 15 segundos
      // Iniciar el proceso de reconexión no bloqueante
      estadoReconexionWiFi = WIFI_RECONEXION_INICIANDO;
    }
    return false;
  }

  // Si estamos conectados, verificar periódicamente la conexión
  if (WiFi.status() == WL_CONNECTED) {
    // Verificación cada 15 segundos
    if (currentMillis - lastPingTime >= 15000) {
      lastPingTime = currentMillis;

      // Verificar IP válida
      IPAddress ip = WiFi.localIP();
      if (ip[0] == 0) {
        Serial.println("Anomalía: WiFi reporta conectado pero IP inválida");
        inReconnectionMode = true;
        return false;
      }

      // Hacer ping al gateway para verificar conectividad
      IPAddress gateway = WiFi.gatewayIP();
      if (gateway[0] != 0) {
        WiFiClient testClient;
        testClient.setTimeout(500);  // Timeout corto
        bool pingSuccess = testClient.connect(gateway, 80);
        testClient.stop();

        if (!pingSuccess) {
          failedPings++;
          Serial.printf("Ping al gateway fallido (%d/3)\n", failedPings);

          if (failedPings >= 3) {
            Serial.println("Múltiples pings fallidos al gateway. Entrando en modo de reconexión.");
            inReconnectionMode = true;
            return false;
          }
        } else {
          failedPings = 0;  // Reset contador si ping exitoso
        }
      }
    }

    // Si llegamos aquí, la conexión WiFi está bien
    wifiReconnectAttempts = 0;
    return true;
  }

  // WiFi desconectado y no en modo reconexión - iniciar modo reconexión
  if (!inReconnectionMode) {
    Serial.println("WiFi desconectado. Entrando en modo de reconexión automática.");
    inReconnectionMode = true;
    lastReconnectAttempt = currentMillis - 10000;  // Intentar pronto
  }

  return false;
}

// Función para extraer y almacenar la IP del servidor
void almacenarIPServidor(const char* ip) {
  // Validar que la IP tenga un formato válido
  IPAddress testIP;
  if (testIP.fromString(ip)) {
    // Solo actualizar si la IP es diferente a la actual
    if (strcmp(ip, direccionIPBackup) != 0) {
      strcpy(direccionIPBackup, ip);
      Serial.printf("Nueva IP de TK1 almacenada: %s\n", direccionIPBackup);

      // Guardar en memoria persistente
      preferenciasTK2.begin("tk2config", false);
      preferenciasTK2.putString("serverIP", ip);
      preferenciasTK2.end();
    }
  }
}

//========= AÑADIR ESTA FUNCIÓN PARA CONFIGURAR OTA ==========
void setupOTA() {
  // Establecer hostname para OTA
  String hostname = "tanque-secundario";
  ArduinoOTA.setHostname(hostname.c_str());

  // Configurar contraseña (opcional)
  // ArduinoOTA.setPassword("contraseña");

  // Eventos de callback
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Iniciando actualización " + type);

    // Iniciar parpadeo rápido para indicar OTA en progreso
    iniciarParpadeo(100);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nActualización finalizada");
    // Led fijo para indicar finalización
    ledFijo(true);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
    // Alternar LED cada 10% para mostrar progreso
    if ((progress / (total / 100)) % 10 == 0) {
      digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Autenticación fallida");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Inicio fallido");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Conexión fallida");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Recepción fallida");
    else if (error == OTA_END_ERROR) Serial.println("Finalización fallida");

    // Parpadeo rápido para indicar error
    iniciarParpadeo(300);
  });

  // Iniciar OTA
  ArduinoOTA.begin();

  Serial.println("OTA configurado. Nombre: " + hostname);
  Serial.println("IP: " + WiFi.localIP().toString());
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Tiempo para que se estabilice el serial
  Serial.println("Iniciando setup...");

  // Configuración de pines
  pinMode(RESET_BUTTON, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FLOAT_SWITCH_2, INPUT);  // Sin pullup ya que tiene resistencia externa
  digitalWrite(TRIG_PIN, LOW);

  // Configuración del LED de estado
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);  // Inicialmente apagado



  // Inicializar variables de control del LED
  ledParpadeoActivo = false;
  ultimoParpadeo = 0;
  comunicacionFallando = false;

  Serial.println("Configuración de pines completada");

  // Cargar la IP del servidor desde la memoria persistente
  preferenciasTK2.begin("tk2config", true);  // Solo lectura
  String savedIP = preferenciasTK2.getString("serverIP", "");
  preferenciasTK2.end();

  if (savedIP.length() > 0) {
    strcpy(direccionIPBackup, savedIP.c_str());
    Serial.println("IP del servidor cargada desde memoria: " + savedIP);
  } else {
    Serial.println("No se encontró IP guardada, usando valor predeterminado: " + String(direccionIPBackup));
  }

  // Verificar si el botón está presionado al arrancar
  bool resetWiFiConfig = false;
  if (digitalRead(RESET_BUTTON) == LOW) {
    delay(1000);
    if (digitalRead(RESET_BUTTON) == LOW) {
      Serial.println("Botón presionado - Reseteando configuración WiFi");
      resetWiFiConfig = true;

      // Opcionalmente, también resetear la IP guardada
      if (resetWiFiConfig && server.hasArg("reset_ip")) {
        preferenciasTK2.begin("tk2config", false);
        preferenciasTK2.remove("serverIP");
        preferenciasTK2.end();
        Serial.println("IP del servidor también ha sido reseteada");
      }
    }
  }

  // Inicializar array de mediciones
  for (int i = 0; i < VENTANA_PROMEDIO; i++) {
    medidasVentana[i] = 0;
  }

  // Método simplificado de conexión WiFi primero
  Serial.println("Intentando conectar a WiFi guardado...");
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);   // Configurar como estación (client) solamente
  WiFi.setSleep(false);  // Desactivar modo de ahorro de energía

  WiFi.begin();  // Intentar conectar con credenciales guardadas

  // Esperar por conexión con un timeout más largo y reiniciando el watchdog
  int directAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && directAttempts < 20) {
    delay(500);
    Serial.print(".");
    directAttempts++;
  }
  Serial.println();

  bool wifiDirectConnect = (WiFi.status() == WL_CONNECTED);

  // Si la conexión directa falló
  if (!wifiDirectConnect) {
    iniciarParpadeo(100);  // Parpadeo cada 300ms por defecto
    Serial.println("Conexión directa falló, intentando reconexión simple...");
    // Intentar simplemente reconectar sin WiFiManager
    WiFi.disconnect();
    delay(500);
    WiFi.begin();  // Usar credenciales guardadas
    // Esperar máximo 5 segundos
    int waitCount = 0;
    while (WiFi.status() != WL_CONNECTED && waitCount < 10) {
      delay(500);
      Serial.print(".");
      waitCount++;
    }
    // Solo usar WiFiManager si el botón de reset está presionado
    if (WiFi.status() != WL_CONNECTED && resetWiFiConfig) {
      WiFiManager wifiManager;
      wifiManager.setConfigPortalTimeout(60);  // 60 segundos máximo
      wifiManager.startConfigPortal("TanqueAgua-TK2-Setup");
    }
  }

  // Inicializar estado de comunicación
  ultimaRespuestaExitosa = 0;
  comunicacionFallando = true;  // Iniciar asumiendo problemas hasta primera comunicación exitosa

  // ELIMINAR ESTA SECCIÓN REDUNDANTE
  // La configuración del LED según WiFi se hará solo al final del setup

  // Configurar potencia máxima de transmisión después de conectarse
  esp_wifi_set_max_tx_power(78);  // 78 = 19.5dBm, el máximo permitido

  // Mostrar información sobre la señal WiFi
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Potencia de señal (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    Serial.print("Canal WiFi: ");
    Serial.println(WiFi.channel());

    Serial.println("Configurada potencia de transmisión al máximo (19.5dBm)");
  }

  // Intenta descubrir el servidor TK1 vía mDNS
  Serial.println("Buscando servidor TK1 por mDNS...");
  IPAddress serverIP = MDNS.queryHost("tanque-principal", 1000);

  // Verificar si la dirección IP es válida (no 0.0.0.0)
  if (serverIP.toString() != "0.0.0.0") {
    Serial.println("Servidor TK1 encontrado en: " + serverIP.toString());
    // Si es diferente a la guardada, actualizarla
    if (serverIP.toString() != String(direccionIPBackup)) {
      strcpy(direccionIPBackup, serverIP.toString().c_str());
      Serial.println("IP actualizada por descubrimiento mDNS: " + serverIP.toString());

      // Guardar la nueva IP
      preferenciasTK2.begin("tk2config", false);
      preferenciasTK2.putString("serverIP", serverIP.toString());
      preferenciasTK2.end();
    }
  } else {
    Serial.println("No se pudo encontrar TK1 por mDNS. Usando IP almacenada: " + String(direccionIPBackup));
  }

  // Configurar endpoint de estado con información WiFi mejorada
  server.on("/status", HTTP_GET, [&]() {
    DynamicJsonDocument doc(384);  // Aumentamos el tamaño para incluir más información
    doc["wifi_ok"] = WiFi.status() == WL_CONNECTED;
    doc["rssi"] = WiFi.RSSI();
    doc["canal"] = WiFi.channel();
    doc["uptime"] = millis() / 1000;
    doc["heap"] = ESP.getFreeHeap();
    doc["nivel"] = ultimaMedicionDistancia;  // Usar la última medición válida
    doc["flotador"] = digitalRead(FLOAT_SWITCH_2) == LOW;
    doc["tx_power"] = "max";  // Indicador de que estamos a potencia máxima
    doc["ip"] = WiFi.localIP().toString();
    doc["led_parpadeo"] = ledParpadeoActivo;                 // Estado de parpadeo
    doc["led_encendido"] = digitalRead(LED_STATUS) == HIGH;  // Estado físico del LED
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Añadir un endpoint específico para información WiFi
  server.on("/wifi", HTTP_GET, []() {
    DynamicJsonDocument doc(512);
    doc["ssid"] = WiFi.SSID();
    doc["bssid"] = WiFi.BSSIDstr();
    doc["rssi"] = WiFi.RSSI();
    doc["canal"] = WiFi.channel();
    doc["ip"] = WiFi.localIP().toString();
    doc["mascara"] = WiFi.subnetMask().toString();
    doc["gateway"] = WiFi.gatewayIP().toString();
    doc["dns"] = WiFi.dnsIP().toString();
    doc["mac"] = WiFi.macAddress();
    doc["tx_power"] = "19.5dBm";

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Añadir un endpoint para forzar el LED a encendido (para pruebas)
  server.on("/led/on", HTTP_GET, []() {
    ledFijo(true);
    server.send(200, "text/plain", "LED forzado a encendido");
    Serial.println("LED forzado a encendido por petición HTTP");
  });

  // Añadir un endpoint para parpadeo del LED (para pruebas)
  server.on("/led/blink", HTTP_GET, []() {
    int velocidad = server.hasArg("speed") ? server.arg("speed").toInt() : 300;
    // Ensure velocity is within reasonable limits
    if (velocidad < 50) velocidad = 50;
    if (velocidad > 2000) velocidad = 2000;

    iniciarParpadeo(velocidad);
    server.send(200, "text/plain", "LED en modo parpadeo con velocidad: " + String(velocidad) + "ms");
    Serial.println("LED en modo parpadeo por petición HTTP. Velocidad: " + String(velocidad) + "ms");
  });
  // Añadir un endpoint para reiniciar el dispositivo (para pruebas)
  server.on("/reset", HTTP_GET, []() {
    server.send(200, "text/plain", "Reiniciando dispositivo...");
    Serial.println("Reinicio solicitado por petición HTTP");
    delay(1000);
    ESP.restart();
  });

  // Añadir endpoint para diagnóstico del LED
  server.on("/led/status", HTTP_GET, []() {
    DynamicJsonDocument doc(256);
    doc["estado_actual"] = ledEstadoActual == LED_STATE_ON ? "FIJO" : "PARPADEO";
    doc["velocidad_ms"] = ledEstadoActual == LED_STATE_BLINKING ? velocidadParpadeoActual : 0;
    doc["estado_fisico"] = ledFisicoEncendido;
    doc["wifi_conectado"] = WiFi.status() == WL_CONNECTED;
    doc["comunicacion_fallando"] = comunicacionFallando;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
    // Registrar diagnóstico en serial
    Serial.println("Diagnóstico de LED solicitado por HTTP:");
    Serial.println(response);
  });

  server.on("/server-ip", HTTP_GET, []() {
    if (server.hasArg("ip")) {
      String newIP = server.arg("ip");
      IPAddress testIP;
      if (testIP.fromString(newIP)) {
        almacenarIPServidor(newIP.c_str());
        server.send(200, "text/plain", "IP del servidor actualizada a: " + newIP);
      } else {
        server.send(400, "text/plain", "Formato de IP inválido");
      }
    } else {
      server.send(200, "text/plain", "IP actual del servidor TK1: " + String(direccionIPBackup));
    }
  });


  //========= AÑADIR ESTE ENDPOINT AL WEBSERVER EN setup() ==========
  server.on("/ota-info", HTTP_GET, []() {
    DynamicJsonDocument doc(256);
    doc["device"] = "TK2";
    doc["hostname"] = ArduinoOTA.getHostname();
    doc["ip"] = WiFi.localIP().toString();
    doc["flash_size"] = ESP.getFlashChipSize() / 1024;
    doc["free_heap"] = ESP.getFreeHeap() / 1024;
    doc["sketch_size"] = ESP.getSketchSize() / 1024;
    doc["free_sketch_space"] = ESP.getFreeSketchSpace() / 1024;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);

    Serial.println("Información OTA solicitada via HTTP");
  });


  // Iniciar el servidor web
  server.begin();
  Serial.println("Servidor web iniciado - Endpoints configurados: /status, /wifi, /led/on, /led/off, /led/blink, /led/status, /reset");

  // Iniciar mDNS para descubrimiento en la red local
  if (MDNS.begin("tanque-secundario")) {
    Serial.println("mDNS responder iniciado");
    MDNS.addService("http", "tcp", 80);
  }

  // Leer el estado inicial del flotante
  flotadorTK2 = digitalRead(FLOAT_SWITCH_2);
  Serial.printf("Estado inicial del flotante TK2: %s\n",
                flotadorTK2 == LOW ? "ACTIVO (nivel bajo)" : "INACTIVO (nivel normal)");

  // Diagnóstico del pin del flotador
  Serial.printf("DIAGNÓSTICO CRÍTICO: Pin FLOAT_SWITCH_2 (GPIO %d) = %d → Interpretado como: %s\n",
                FLOAT_SWITCH_2,
                digitalRead(FLOAT_SWITCH_2),
                digitalRead(FLOAT_SWITCH_2) == LOW ? "ACTIVO (tanque vacío)" : "INACTIVO (tanque con agua)");

  // Diagnóstico de estado del LED al final del setup
  Serial.println("\n=== DIAGNÓSTICO INICIAL DEL LED ===");
  Serial.printf("Estado físico del LED: %s\n", digitalRead(LED_STATUS) == HIGH ? "ENCENDIDO" : "APAGADO");
  Serial.printf("Parpadeo activo: %s\n", ledParpadeoActivo ? "SÍ" : "NO");
  Serial.printf("Velocidad parpadeo: %d ms\n", intervaloParpadeo);
  Serial.printf("Último cambio hace: %lu ms\n", millis() - ultimoParpadeo);
  Serial.println("=== FIN DIAGNÓSTICO LED ===\n");

  // Establecer el estado inicial del LED según la conexión WiFi
  if (WiFi.status() == WL_CONNECTED) {
    // LED parpadeo medio para indicar "esperando primera comunicación"
    Serial.println("LED: Parpadeo medio - Esperando comunicación con servidor");
  } else {
    // LED parpadeo rápido para indicar "sin WiFi"
    iniciarParpadeo(true);

    Serial.println("LED: Parpadeo rápido - Sin conexión WiFi");
  }
  // Añadir esta línea al final del setup
  setupOTA();
  Serial.println("Setup completado");
}

void loop() {
  // Manejar peticiones OTA
  ArduinoOTA.handle();

  // Update LED state based on system conditions
  actualizarEstadoLED();

  // Handle WiFi connection and server communication
  handleWiFiConnection();
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
  }
  procesarYEnviarDatos();
}
