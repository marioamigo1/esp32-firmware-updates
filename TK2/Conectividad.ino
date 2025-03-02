// Función optimizada para probar conectividad y obtener la URL correcta
String obtenerURLTanque(const char* nombreMDNS, const char* ipPredeterminada, const char* rutaBase) {
  // Primero intentar con mDNS con timeout reducido
  WiFiClient client;
  client.setTimeout(1000);  // Reducir a 1 segundo

  // Intentar con mDNS primero
  String hostnameMDNS = String(nombreMDNS) + ".local";
  Serial.print("Probando conectividad a " + hostnameMDNS + "...");

  if (client.connect(hostnameMDNS.c_str(), 80)) {
    client.stop();
    Serial.println("Alcanzable");
    return "http://" + hostnameMDNS + rutaBase;
  } else {
    client.stop();
    Serial.println("No alcanzable");
  }

  // Si falla, intentar con IP directa
  Serial.print("Probando conectividad a " + String(ipPredeterminada) + "...");
  if (client.connect(ipPredeterminada, 80)) {
    client.stop();
    Serial.println("Alcanzable");
    return "http://" + String(ipPredeterminada) + rutaBase;
  } else {
    client.stop();
    Serial.println("No alcanzable");
    return "";  // Retornar cadena vacía si ambos fallan
  }
}

// Función optimizada para enviar datos al servidor
bool enviarDatosOptimizado(float distancia, bool flotadorActivo) {
  // Usar la función optimizada para obtener la URL correcta
  String urlBase = obtenerURLTanque(nombreServidor, direccionIPBackup, "/test");

  // Si la URL está vacía, ambos métodos fallaron
  if (urlBase.isEmpty()) {
    Serial.println("Error: No se pudo establecer conexión con el servidor");
    return false;
  }

  // Construir URL completa para enviar datos
  String urlUpdateTK2 = urlBase;
  if (urlBase.endsWith("/test")) {
    urlUpdateTK2 = urlBase.substring(0, urlBase.length() - 5) + "/updateTK2";
  } else {
    urlUpdateTK2 = urlBase + "/updateTK2";
  }

  // Añadir parámetros
  urlUpdateTK2 += "?origen=TK2";
  urlUpdateTK2 += "&nivel=" + String(distancia, 1);
  urlUpdateTK2 += "&flotante=" + String(flotadorActivo ? 1 : 0);

  // Enviar la solicitud
  Serial.println("Enviando: " + urlUpdateTK2);

  HTTPClient http;
  http.setTimeout(2000);  // Timeout razonable
  http.begin(urlUpdateTK2);

  int httpCode = http.GET();
  bool exito = false;

  if (httpCode == HTTP_CODE_OK) {
    String respuesta = http.getString();
    Serial.println("Respuesta: " + respuesta);
    exito = true;

    // Actualizar estado de comunicación
    comunicacionFallando = false;
    ultimaRespuestaExitosa = millis();
  } else {
    Serial.printf("Error HTTP: %d\n", httpCode);
    comunicacionFallando = true;
  }

  http.end();
  return exito;
}