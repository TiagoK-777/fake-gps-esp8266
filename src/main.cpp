#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

// --- Configurações WiFi ---
#define WIFI_SSID "Quer Navegar compra um barco"
#define WIFI_PASS "salgado40"

// --- Objetos Globais ---
ESP8266WebServer server(80);

// --- Hardware ---
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 
#endif
unsigned long ledOnTime = 0;
const unsigned long LED_BLINK_DURATION = 10; // ms

// --- Variáveis de Estado do GPS ---
double latitude = -23.550520;
double longitude = -46.633308;
double altitude = 760.0;
float speed = 0.0;    // km/h
float heading = 90.0; // graus
int satellites = 12;
float hdop = 0.8;
int gpsQuality = 4; // 0=Invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float

// --- Variáveis de Estado IMU ---
float pitch = 0.0;   // graus (-90 a 90)
float roll = 0.0;    // graus (-180 a 180)
float yawRate = 0.0; // graus/s

// --- Configurações do Sistema ---
unsigned long updateRate = 100; // ms
long serialBaud = 115200;
bool simulateMotion = false;

// --- Controle de Mensagens NMEA ---
bool enableGGA = true;
bool enableGSA = true;
bool enableVTG = true;
bool enableRMC = true;
bool enableGSV = false;
bool enableZDA = false;
bool enableHPR = false;
bool enableKSXT = false;

// --- Controle de Tempo ---
unsigned long lastUpdate = 0;

// --- Estrutura para EEPROM ---
struct Settings {
  double lat;
  double lon;
  double alt;
  float spd;
  float hdg;
  int sats;
  float hdop;
  int quality;
  float pitch;
  float roll;
  float yawrate;
  unsigned long rate;
  long baud;
  uint8_t flags; // sim, gga, gsa, vtg, rmc, gsv, zda, hpr
  bool en_ksxt;
  uint32_t magic;
};

#define EEPROM_MAGIC 0xDEADBEEF

// --- HTML da Página Web ---
const char *htmlPage = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP8266 GPS Simulator</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background-color: #f0f0f0; }
    h2 { color: #333; }
    form { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }
    label { display: block; margin-top: 10px; font-weight: bold; }
    input, select { width: 100%; padding: 8px; margin-top: 5px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
    input[type="checkbox"] { width: auto; margin-top: 10px; }
    input[type="range"] { padding: 0; border: none; }
    input[type="submit"] { background-color: #4CAF50; color: white; border: none; cursor: pointer; margin-top: 20px; font-size: 16px; }
    input[type="submit"]:hover { background-color: #45a049; }
    .note { font-size: 0.9em; color: #666; margin-top: 5px; }
    #status { position: fixed; top: 10px; right: 10px; background: #4CAF50; color: white; padding: 8px 16px; border-radius: 4px; display: none; }
  </style>
  <script>
    var updateTimer;
    function autoUpdate(immediate) {
      if(immediate) {
        clearTimeout(updateTimer);
        sendUpdate();
      } else {
        clearTimeout(updateTimer);
        updateTimer = setTimeout(sendUpdate, 500);
      }
    }
    function sendUpdate() {
      var form = document.getElementById('gpsForm');
      var formData = new FormData(form);
      var xhr = new XMLHttpRequest();
      xhr.open('POST', '/update', true);
      xhr.onload = function() {
        var status = document.getElementById('status');
        status.style.display = 'block';
        status.innerHTML = 'Atualizado!';
        setTimeout(function(){ status.style.display = 'none'; }, 1000);
      };
      xhr.send(formData);
    }
  </script>
</head>
<body>
  <div id="status">Atualizado!</div>
  <center>
    <h2>Configuracao GPS Fake (ESP8266)</h2>
    <p>IP: %IP_ADDRESS%</p>
  </center>
  <form id="gpsForm" action="/save" method="POST">
    <h3>Posicao e Movimento</h3>
    <label>Latitude (graus):</label><input type="number" step="any" name="lat" value="%LAT%">
    <label>Longitude (graus):</label><input type="number" step="any" name="lon" value="%LON%">
    <label>Altitude (m):</label><input type="number" step="any" name="alt" value="%ALT%">
    
    <label>Velocidade: <span id="s_spd">%SPD%</span> km/h</label>
    <input type="range" min="0" max="50" step="0.1" name="spd" value="%SPD%" oninput="document.getElementById('s_spd').innerHTML=this.value;autoUpdate()">
    
    <label>Direcao: <span id="s_hdg">%HDG%</span> graus</label>
    <input type="range" min="0" max="360" step="1" name="hdg" value="%HDG%" oninput="document.getElementById('s_hdg').innerHTML=this.value;autoUpdate()">
    
    <label>Simular Movimento:</label>
    <input type="checkbox" name="sim" %SIM_CHECKED% onchange="autoUpdate(true)"> Ativar simulacao linear
    
    <h3>Dados de Atitude (IMU)</h3>
    <label>Pitch (inclinacao): <span id="s_pitch">%PITCH%</span> graus</label>
    <input type="range" min="-90" max="90" step="0.1" name="pitch" value="%PITCH%" oninput="document.getElementById('s_pitch').innerHTML=this.value;autoUpdate()">
    
    <label>Roll (inclinacao lateral): <span id="s_roll">%ROLL%</span> graus</label>
    <input type="range" min="-180" max="180" step="0.1" name="roll" value="%ROLL%" oninput="document.getElementById('s_roll').innerHTML=this.value;autoUpdate()">
    
    <label>Yaw Rate (taxa guinada): <span id="s_yawrate">%YAWRATE%</span> graus/s</label>
    <input type="range" min="-30" max="30" step="0.1" name="yawrate" value="%YAWRATE%" oninput="document.getElementById('s_yawrate').innerHTML=this.value;autoUpdate()">
    
    <button type="button" onclick="document.querySelector('[name=pitch]').value=0;document.getElementById('s_pitch').innerHTML='0';document.querySelector('[name=roll]').value=0;document.getElementById('s_roll').innerHTML='0';document.querySelector('[name=yawrate]').value=0;document.getElementById('s_yawrate').innerHTML='0';autoUpdate(true);" style="width: 100%; margin-top: 10px; padding: 10px; background-color: #ff9800; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">Zerar IMU (Pitch/Roll/Yaw Rate)</button>
    
    <h3>Sinal GPS</h3>
    <label>Satelites:</label><input type="number" name="sats" value="%SATS%" onchange="autoUpdate(true)">
    <label>HDOP:</label><input type="number" step="0.1" name="hdop" value="%HDOP%" onchange="autoUpdate(true)">
    <label>Qualidade do Sinal:</label>
    <select name="quality" onchange="autoUpdate(true)">
      <option value="0" %Q_0%>0 - Invalido</option>
      <option value="1" %Q_1%>1 - GPS (SPS)</option>
      <option value="2" %Q_2%>2 - DGPS</option>
      <option value="4" %Q_4%>4 - RTK Fixed</option>
      <option value="5" %Q_5%>5 - RTK Float</option>
    </select>
    
    <h3>Mensagens NMEA</h3>
    <label><input type="checkbox" name="en_gga" %EN_GGA% onchange="autoUpdate(true)"> GGA - Posicao e altitude</label>
    <label><input type="checkbox" name="en_gsa" %EN_GSA% onchange="autoUpdate(true)"> GSA - DOP e satelites ativos</label>
    <label><input type="checkbox" name="en_vtg" %EN_VTG% onchange="autoUpdate(true)"> VTG - Velocidade e direcao</label>
    <label><input type="checkbox" name="en_rmc" %EN_RMC% onchange="autoUpdate(true)"> RMC - Dados minimos recomendados</label>
    <label><input type="checkbox" name="en_gsv" %EN_GSV% onchange="autoUpdate(true)"> GSV - Satelites em vista</label>
    <label><input type="checkbox" name="en_zda" %EN_ZDA% onchange="autoUpdate(true)"> ZDA - Data e hora completa</label>
    <label><input type="checkbox" name="en_hpr" %EN_HPR% onchange="autoUpdate(true)"> HPR - Atitude (pitch/roll/heading)</label>
    <label><input type="checkbox" name="en_ksxt" %EN_KSXT% onchange="autoUpdate(true)"> KSXT - Unicore Proprietary</label>
    
    <h3>Configuracoes do Sistema</h3>
    <label>Taxa de Atualizacao (Hz):</label>
    <select name="hz" onchange="autoUpdate(true)">
      <option value="1" %HZ_1%>1 Hz</option>
      <option value="5" %HZ_5%>5 Hz</option>
      <option value="10" %HZ_10%>10 Hz</option>
      <option value="20" %HZ_20%>20 Hz</option>
    </select>
    <label>Baud Rate Serial:</label>
    <select name="baud" onchange="autoUpdate(true)">
      <option value="9600" %BAUD_9600%>9600</option>
      <option value="38400" %BAUD_38400%>38400</option>
      <option value="57600" %BAUD_57600%>57600</option>
      <option value="115200" %BAUD_115200%>115200</option>
      <option value="230400" %BAUD_230400%>230400</option>
      <option value="460800" %BAUD_460800%>460800</option>
    </select>
    <div class="note">Nota: Alterar o Baud Rate reiniciara a Serial.</div>
    
    <input type="submit" value="Salvar e Aplicar">
  </form>
</body>
</html>
)rawliteral";

// --- Funções de Persistência ---
void loadSettings() {
  EEPROM.begin(512);
  Settings s;
  EEPROM.get(0, s);

  if (s.magic == EEPROM_MAGIC) {
    latitude = s.lat;
    longitude = s.lon;
    altitude = s.alt;
    speed = s.spd;
    heading = s.hdg;
    satellites = s.sats;
    hdop = s.hdop;
    gpsQuality = s.quality;
    pitch = s.pitch;
    roll = s.roll;
    yawRate = s.yawrate;
    updateRate = s.rate;
    serialBaud = s.baud;
    simulateMotion = s.flags & 0x01;
    enableGGA = s.flags & 0x02;
    enableGSA = s.flags & 0x04;
    enableVTG = s.flags & 0x08;
    enableRMC = s.flags & 0x10;
    enableGSV = s.flags & 0x20;
    enableZDA = s.flags & 0x40;
    enableHPR = s.flags & 0x80;
    enableKSXT = s.en_ksxt;
  }
}

void saveSettings() {
  Settings s;
  s.lat = latitude;
  s.lon = longitude;
  s.alt = altitude;
  s.spd = speed;
  s.hdg = heading;
  s.sats = satellites;
  s.hdop = hdop;
  s.quality = gpsQuality;
  s.pitch = pitch;
  s.roll = roll;
  s.yawrate = yawRate;
  s.rate = updateRate;
  s.baud = serialBaud;

  s.flags = 0;
  if (simulateMotion) s.flags |= 0x01;
  if (enableGGA) s.flags |= 0x02;
  if (enableGSA) s.flags |= 0x04;
  if (enableVTG) s.flags |= 0x08;
  if (enableRMC) s.flags |= 0x10;
  if (enableGSV) s.flags |= 0x20;
  if (enableZDA) s.flags |= 0x40;
  if (enableHPR) s.flags |= 0x80;

  s.en_ksxt = enableKSXT;
  s.magic = EEPROM_MAGIC;

  EEPROM.put(0, s);
  EEPROM.commit();
}

// --- Handlers do Servidor Web ---
void handleRoot() {
  String s = htmlPage;
  s.replace("%IP_ADDRESS%", WiFi.localIP().toString());
  s.replace("%LAT%", String(latitude, 6));
  s.replace("%LON%", String(longitude, 6));
  s.replace("%ALT%", String(altitude, 2));
  s.replace("%SPD%", String(speed, 2));
  s.replace("%HDG%", String(heading, 2));
  s.replace("%SATS%", String(satellites));
  s.replace("%HDOP%", String(hdop, 1));
  s.replace("%PITCH%", String(pitch, 2));
  s.replace("%ROLL%", String(roll, 2));
  s.replace("%YAWRATE%", String(yawRate, 2));

  s.replace("%Q_0%", gpsQuality == 0 ? "selected" : "");
  s.replace("%Q_1%", gpsQuality == 1 ? "selected" : "");
  s.replace("%Q_2%", gpsQuality == 2 ? "selected" : "");
  s.replace("%Q_4%", gpsQuality == 4 ? "selected" : "");
  s.replace("%Q_5%", gpsQuality == 5 ? "selected" : "");

  int hz = (updateRate > 0) ? 1000 / updateRate : 1;
  s.replace("%HZ_1%", hz == 1 ? "selected" : "");
  s.replace("%HZ_5%", hz == 5 ? "selected" : "");
  s.replace("%HZ_10%", hz == 10 ? "selected" : "");
  s.replace("%HZ_20%", hz == 20 ? "selected" : "");

  s.replace("%SIM_CHECKED%", simulateMotion ? "checked" : "");

  s.replace("%BAUD_9600%", serialBaud == 9600 ? "selected" : "");
  s.replace("%BAUD_38400%", serialBaud == 38400 ? "selected" : "");
  s.replace("%BAUD_57600%", serialBaud == 57600 ? "selected" : "");
  s.replace("%BAUD_115200%", serialBaud == 115200 ? "selected" : "");
  s.replace("%BAUD_230400%", serialBaud == 230400 ? "selected" : "");
  s.replace("%BAUD_460800%", serialBaud == 460800 ? "selected" : "");

  s.replace("%EN_GGA%", enableGGA ? "checked" : "");
  s.replace("%EN_GSA%", enableGSA ? "checked" : "");
  s.replace("%EN_VTG%", enableVTG ? "checked" : "");
  s.replace("%EN_RMC%", enableRMC ? "checked" : "");
  s.replace("%EN_GSV%", enableGSV ? "checked" : "");
  s.replace("%EN_ZDA%", enableZDA ? "checked" : "");
  s.replace("%EN_HPR%", enableHPR ? "checked" : "");
  s.replace("%EN_KSXT%", enableKSXT ? "checked" : "");

  server.send(200, "text/html", s);
}

void handleSave() {
  if (server.hasArg("lat"))
    latitude = server.arg("lat").toDouble();
  if (server.hasArg("lon"))
    longitude = server.arg("lon").toDouble();
  if (server.hasArg("alt"))
    altitude = server.arg("alt").toDouble();
  if (server.hasArg("spd"))
    speed = server.arg("spd").toFloat();
  if (server.hasArg("hdg"))
    heading = server.arg("hdg").toFloat();

  if (server.hasArg("sats"))
    satellites = server.arg("sats").toInt();
  if (server.hasArg("hdop"))
    hdop = server.arg("hdop").toFloat();
  if (server.hasArg("pitch"))
    pitch = server.arg("pitch").toFloat();
  if (server.hasArg("roll"))
    roll = server.arg("roll").toFloat();
  if (server.hasArg("yawrate"))
    yawRate = server.arg("yawrate").toFloat();
  if (server.hasArg("quality"))
    gpsQuality = server.arg("quality").toInt();

  if (server.hasArg("hz")) {
    int hz = server.arg("hz").toInt();
    if (hz > 0)
      updateRate = 1000 / hz;
  }

  simulateMotion = server.hasArg("sim");
  enableGGA = server.hasArg("en_gga");
  enableGSA = server.hasArg("en_gsa");
  enableVTG = server.hasArg("en_vtg");
  enableRMC = server.hasArg("en_rmc");
  enableGSV = server.hasArg("en_gsv");
  enableZDA = server.hasArg("en_zda");
  enableHPR = server.hasArg("en_hpr");
  enableKSXT = server.hasArg("en_ksxt");

  if (server.hasArg("baud")) {
    long newBaud = server.arg("baud").toInt();
    if (newBaud != serialBaud) {
      serialBaud = newBaud;
      Serial.end();
      Serial.begin(serialBaud);
    }
  }

  saveSettings();

  String message = "<!DOCTYPE HTML><html><head><meta http-equiv='refresh' "
                   "content='2;url=/'></head><body><h2>Configuracoes "
                   "Salvas!</h2><p>Redirecionando...</p></body></html>";
  server.send(200, "text/html", message);
}

void handleUpdate() {
  if (server.hasArg("spd"))
    speed = server.arg("spd").toFloat();
  if (server.hasArg("hdg"))
    heading = server.arg("hdg").toFloat();

  if (server.hasArg("sats"))
    satellites = server.arg("sats").toInt();
  if (server.hasArg("hdop"))
    hdop = server.arg("hdop").toFloat();
  if (server.hasArg("pitch"))
    pitch = server.arg("pitch").toFloat();
  if (server.hasArg("roll"))
    roll = server.arg("roll").toFloat();
  if (server.hasArg("yawrate"))
    yawRate = server.arg("yawrate").toFloat();
  if (server.hasArg("quality"))
    gpsQuality = server.arg("quality").toInt();

  if (server.hasArg("hz")) {
    int hz = server.arg("hz").toInt();
    if (hz > 0)
      updateRate = 1000 / hz;
  }

  simulateMotion = server.hasArg("sim");
  enableGGA = server.hasArg("en_gga");
  enableGSA = server.hasArg("en_gsa");
  enableVTG = server.hasArg("en_vtg");
  enableRMC = server.hasArg("en_rmc");
  enableGSV = server.hasArg("en_gsv");
  enableZDA = server.hasArg("en_zda");
  enableHPR = server.hasArg("en_hpr");
  enableKSXT = server.hasArg("en_ksxt");

  if (server.hasArg("baud")) {
    long newBaud = server.arg("baud").toInt();
    if (newBaud != serialBaud) {
      serialBaud = newBaud;
      Serial.end();
      Serial.begin(serialBaud);
    }
  }

  server.send(200, "text/plain", "OK");
}

// --- Funções GPS NMEA ---
String calculateChecksum(String sentence) {
  int checksum = 0;
  for (int i = 1; i < sentence.length(); i++) {
    checksum ^= sentence.charAt(i);
  }
  String checksumStr = String(checksum, HEX);
  checksumStr.toUpperCase();
  if (checksumStr.length() == 1) {
    checksumStr = "0" + checksumStr;
  }
  return checksumStr;
}

String formatLatitude(double lat) {
  char latDir = lat >= 0 ? 'N' : 'S';
  lat = abs(lat);
  int degrees = (int)lat;
  double minutes = (lat - degrees) * 60.0;

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%02d%02.6f,%c", degrees, minutes, latDir);
  return String(buffer);
}

String formatLongitude(double lon) {
  char lonDir = lon >= 0 ? 'E' : 'W';
  lon = abs(lon);
  int degrees = (int)lon;
  double minutes = (lon - degrees) * 60.0;

  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%03d%02.6f,%c", degrees, minutes, lonDir);
  return String(buffer);
}

String getTimeUTC() {
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;

  int hours = (seconds / 3600) % 24;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  int centisecs = (currentMillis % 1000) / 10;

  char buffer[15];
  snprintf(buffer, sizeof(buffer), "%02d%02d%02d.%02d", hours, minutes, secs,
           centisecs);
  return String(buffer);
}

void sendGGA() {
  String sentence = "GPGGA,";
  sentence += getTimeUTC() + ",";
  sentence += formatLatitude(latitude) + ",";
  sentence += formatLongitude(longitude) + ",";
  sentence += String(gpsQuality) + ",";

  char satBuffer[4];
  snprintf(satBuffer, sizeof(satBuffer), "%02d", satellites);
  sentence += String(satBuffer) + ",";

  sentence += String(hdop, 2) + ",";
  sentence += String(altitude, 2) + ",M,";
  sentence += "0.0,M,,";

  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendGSA() {
  String sentence = "GPGSA,A,3,";
  int satsToShow = min(satellites, 12);
  for (int i = 0; i < 12; i++) {
    if (i < satsToShow) {
      char satId[4];
      snprintf(satId, sizeof(satId), "%02d", i + 1);
      sentence += String(satId);
    }
    if (i < 11)
      sentence += ",";
  }
  sentence += "," + String(hdop * 1.5, 2) + ",";
  sentence += String(hdop, 2) + ",";
  sentence += String(1.0, 2);
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendVTG() {
  String sentence = "GPVTG,";
  sentence += String(heading, 2) + ",T,,M,";
  float speedKnots = speed * 0.539957;
  sentence += String(speedKnots, 3) + ",N,";
  sentence += String(speed, 3) + ",K,";
  sentence += "A";
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendRMC() {
  String sentence = "GPRMC,";
  sentence += getTimeUTC() + ",";
  sentence += "A,";
  sentence += formatLatitude(latitude) + ",";
  sentence += formatLongitude(longitude) + ",";
  float speedKnots = speed * 0.539957;
  sentence += String(speedKnots, 3) + ",";
  sentence += String(heading, 2) + ",";
  sentence += "181225,";
  sentence += ",,A";
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendGSV() {
  int totalSats = min(satellites, 24);
  int numMessages = (totalSats + 3) / 4;
  for (int msgNum = 1; msgNum <= numMessages; msgNum++) {
    String sentence = "GPGSV,";
    sentence += String(numMessages) + ",";
    sentence += String(msgNum) + ",";
    sentence += String(totalSats) + ",";
    int startSat = (msgNum - 1) * 4;
    int endSat = min(startSat + 4, totalSats);
    for (int i = startSat; i < endSat; i++) {
      int prn = i + 1;
      int elevation = 30 + (i * 7) % 60;
      int azimuth = (i * 30) % 360;
      int snr = 35 + (i * 3) % 20;
      char satInfo[20];
      snprintf(satInfo, sizeof(satInfo), "%02d,%02d,%03d,%02d", prn, elevation, azimuth, snr);
      sentence += String(satInfo);
      if (i < endSat - 1 || endSat < totalSats) sentence += ",";
    }
    int remaining = 4 - (endSat - startSat);
    for (int i = 0; i < remaining; i++) {
      sentence += ",,,";
      if (i < remaining - 1) sentence += ",";
    }
    String checksum = calculateChecksum("$" + sentence);
    Serial.println("$" + sentence + "*" + checksum);
  }
}

void sendZDA() {
  String sentence = "GPZDA,";
  sentence += getTimeUTC() + ",";
  sentence += "18,12,2025,00,00";
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendHPR() {
  String sentence = "GPHPR,";
  sentence += getTimeUTC() + ",";
  char headingStr[10];
  snprintf(headingStr, sizeof(headingStr), "%.2f", heading);
  sentence += String(headingStr) + ",";
  char pitchStr[10];
  snprintf(pitchStr, sizeof(pitchStr), "%.2f", pitch);
  sentence += String(pitchStr) + ",";
  char rollStr[10];
  snprintf(rollStr, sizeof(rollStr), "%.2f", roll);
  sentence += String(rollStr) + ",";
  sentence += "A";
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void sendKSXT() {
  String sentence = "KSXT,";
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  int hours = (seconds / 3600) % 24;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  int centisecs = (currentMillis % 1000) / 10;
  char timeBuf[25];
  snprintf(timeBuf, sizeof(timeBuf), "20260108%02d%02d%02d.%02d", hours, minutes, secs, centisecs);
  sentence += String(timeBuf) + ",";
  char lonBuf[20];
  snprintf(lonBuf, sizeof(lonBuf), "%.8f", longitude);
  sentence += String(lonBuf) + ",";
  char latBuf[20];
  snprintf(latBuf, sizeof(latBuf), "%.8f", latitude);
  sentence += String(latBuf) + ",";
  char altBuf[20];
  snprintf(altBuf, sizeof(altBuf), "%.4f", altitude);
  sentence += String(altBuf) + ",";
  char hdgBuf[10];
  snprintf(hdgBuf, sizeof(hdgBuf), "%.2f", heading);
  sentence += String(hdgBuf) + ",";
  char pitchBuf[10];
  snprintf(pitchBuf, sizeof(pitchBuf), "%.2f", pitch);
  sentence += String(pitchBuf) + ",";
  sentence += String(hdgBuf) + ",";
  char spdBuf[10];
  snprintf(spdBuf, sizeof(spdBuf), "%.3f", speed);
  sentence += String(spdBuf) + ",";
  char rollBuf[10];
  snprintf(rollBuf, sizeof(rollBuf), "%.2f", roll);
  sentence += String(rollBuf) + ",";
  int ksxtQual = (gpsQuality == 4) ? 3 : (gpsQuality == 5) ? 2 : gpsQuality;
  sentence += String(ksxtQual) + ",";
  sentence += String(ksxtQual) + ",";
  sentence += String(satellites) + ",";
  sentence += String(satellites) + ",,,,";
  float rad = radians(heading);
  float vE = speed * sin(rad);
  float vN = speed * cos(rad);
  char velBuf[40];
  snprintf(velBuf, sizeof(velBuf), "%.3f,%.3f,0.000", vE, vN);
  sentence += String(velBuf) + ",,";
  String checksum = calculateChecksum("$" + sentence);
  Serial.println("$" + sentence + "*" + checksum);
}

void simulateMovement() {
  if (!simulateMotion || speed <= 0.001)
    return;
  float speedMs = speed / 3.6;
  float distance = speedMs * (updateRate / 1000.0);
  double latIncrement = (distance * cos(radians(heading))) / 111111.0;
  double lonIncrement = (distance * sin(radians(heading))) / (111111.0 * cos(radians(latitude)));
  latitude += latIncrement;
  longitude += lonIncrement;
}

void setup() {
  delay(500);
  Serial.begin(115200);
  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  loadSettings();
  yield();

  if (serialBaud != 115200) {
    Serial.end();
    delay(10);
    Serial.begin(serialBaud);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(100);
    ESP.wdtFeed();
  }

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/update", HTTP_POST, handleUpdate);
  server.begin();

  ArduinoOTA.setHostname("gps-fake-esp8266");
  ArduinoOTA.onStart([]() { server.stop(); });
  ArduinoOTA.begin();
}

void loop() {
  ESP.wdtFeed();

  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
  }

  ArduinoOTA.handle();
  server.handleClient();

  if (Serial.available()) {
    while (Serial.available()) {
      Serial.read();
    }
    digitalWrite(LED_BUILTIN, HIGH);
    ledOnTime = millis();
  }

  if (millis() - ledOnTime > LED_BLINK_DURATION) {
    digitalWrite(LED_BUILTIN, LOW);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= updateRate) {
    lastUpdate = currentMillis;
    simulateMovement();
    if (enableGGA) sendGGA();
    if (enableGSA) sendGSA();
    if (enableGSV) sendGSV();
    if (enableRMC) sendRMC();
    if (enableVTG) sendVTG();
    if (enableZDA) sendZDA();
    if (enableHPR) sendHPR();
    if (enableKSXT) sendKSXT();
  }
}
