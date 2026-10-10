#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ════════════════════════════════════════════
const char* WIFI_SSID  = "ESP_Test1729";
const char* WIFI_PASS  = "1234567890";
const char* BOT_TOKEN  = "YOU_Token_bot";
const char* CHAT_ID    = "YOU_TG_ID";
// ════════════════════════════════════════════

const unsigned long PHOTO_INTERVAL = 5000;   // как часто слать фото (мс)

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// ──────── Микрофон MAX9814 ────────
#define MIC_PIN         1     // OUT микрофона -> GPIO1 (ADC1_CH0 на ESP32-S3)
#define LED_PIN         2     // опционально, статус записи (можно убрать)

#define SAMPLE_RATE     8000
#define RECORD_SECONDS  3
#define BUFFER_SIZE     (SAMPLE_RATE * RECORD_SECONDS)

int16_t audioBuffer[BUFFER_SIZE];
int bufferIndex = 0;

const char* TG_HOST = "api.telegram.org";
WiFiClientSecure client;

unsigned long lastPhoto = 0;
int photoNum = 0;
int voiceNum = 0;

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 1000000UL / SAMPLE_RATE;

enum State { LISTENING, RECORDING };
State currentState = LISTENING;
unsigned long recordingStart = 0;

int noiseLevel = 0;
int threshold  = 0;

// ── обработка звука ──
const int FADE_SAMPLES  = 160;   // ~20мс при 8кГц, убирает щелчок в начале/конце записи
const int16_t TARGET_PEAK = 30000; // целевая пиковая громкость после нормализации (из 32767 макс)
const float MAX_NORMALIZE_GAIN = 60.0f; // предел усиления, чтобы полная тишина не превращалась в шум
int runningAvg = 0;              // для антиспайк-фильтра (подавляет короткие наводки/помехи)

// ──────────────── КАМЕРА ────────────────
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM; cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM; cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_QVGA;
  cfg.jpeg_quality = 15;
  cfg.fb_count     = 1;
  cfg.fb_location  = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) { Serial.printf("❌ Камера: 0x%x\n", err); return false; }
  Serial.println("✅ Камера OK");
  return true;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(500);
  Serial.println("\n🚀 ESP32-S3-CAM запускается...");

  pinMode(LED_PIN, OUTPUT);

  if (!initCamera()) { delay(3000); ESP.restart(); }

  // ── WiFi точно как в ESPRadar v3.0 ──
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] " + String(WIFI_SSID));
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n⚠️ Не подключился, статус: " + String(WiFi.status()));
  }

  // Отключаем WiFi power-save (modem sleep): периодическое "прослушивание" beacon'ов
  // наводит помехи на ADC каждые ~100мс и даёт ритмичный стук в записи звука.
  WiFi.setSleep(false);

  WiFi.softAP("ESP32CAM", "12345678", 6, 0, 4);
  Serial.println("[AP] ESP32CAM поднят");

  // ── ADC для микрофона ──
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  // ── Калибровка фонового шума ──
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(MIC_PIN);
    delay(5);
  }
  noiseLevel = sum / 200;
  threshold  = noiseLevel + 150; // чувствительнее — ловит более тихие/далёкие звуки
  Serial.printf("🎚️ Шум: %d, порог: %d\n", noiseLevel, threshold);

  delay(1000);
  sendPhoto();
  lastPhoto = millis();
}

// ──────────────── ФОТО ────────────────
void sendPhoto() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi нет, статус: " + String(WiFi.status()));
    return;
  }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("❌ Кадр не получен"); return; }
  Serial.printf("📸 Фото #%d, %d байт\n", ++photoNum, fb->len);

  client.setInsecure();
  if (!client.connect(TG_HOST, 443)) {
    Serial.println("❌ Нет соединения с Telegram");
    esp_camera_fb_return(fb);
    return;
  }

  String boundary = "Boundary7MA4YWxkTrZu0gW";
  String url      = "/bot" + String(BOT_TOKEN) + "/sendPhoto";
  String head =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
    String(CHAT_ID) + "\r\n"
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
    "📷 Фото #" + String(photoNum) + "\r\n"
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"photo\"; filename=\"cam.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";
  int total   = head.length() + fb->len + tail.length();

  client.println("POST " + url + " HTTP/1.1");
  client.println("Host: " + String(TG_HOST));
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.println("Content-Length: " + String(total));
  client.println("Connection: close");
  client.println();
  client.print(head);
  size_t sentJpg = 0;
  { // надёжная отправка JPEG
    size_t len = fb->len;
    while (sentJpg < len) {
      size_t n = client.write(fb->buf + sentJpg, len - sentJpg);
      if (n > 0) sentJpg += n; else { if (!client.connected()) break; delay(1); }
    }
  }
  client.print(tail);

  esp_camera_fb_return(fb);

  unsigned long timeout = millis() + 8000;
  while (!client.available()) {
    if (millis() > timeout) { Serial.println("❌ Таймаут"); client.stop(); return; }
    delay(50);
  }
  String resp = client.readStringUntil('\n');
  client.stop();
  if (resp.indexOf("200") > 0) Serial.println("✅ Фото отправлено!");
  else Serial.println("❌ Ошибка фото: " + resp);
}

// Надёжная отправка буфера: client.write() может вернуть меньше байт,
// чем просили (особенно на больших буферах через TLS) — досылаем остаток.
size_t writeAll(WiFiClientSecure &c, const uint8_t* data, size_t len) {
  size_t sent = 0;
  unsigned long lastProgress = millis();
  while (sent < len) {
    size_t n = c.write(data + sent, len - sent);
    if (n > 0) {
      sent += n;
      lastProgress = millis();
    } else {
      if (!c.connected()) { Serial.println("❌ Соединение оборвалось во время отправки"); break; }
      if (millis() - lastProgress > 10000) { Serial.println("❌ Зависла отправка (0 байт за 10с)"); break; }
      delay(1);
    }
  }
  return sent;
}

// ──────────────── ГОЛОС ────────────────
void buildWavHeader(uint8_t* hdr, uint32_t dataLen) {
  uint32_t fileSize = dataLen + 36;
  uint32_t byteRate  = SAMPLE_RATE * 2;

  memcpy(hdr, "RIFF", 4);
  hdr[4]=fileSize&0xFF; hdr[5]=(fileSize>>8)&0xFF; hdr[6]=(fileSize>>16)&0xFF; hdr[7]=(fileSize>>24)&0xFF;
  memcpy(hdr+8, "WAVE", 4);
  memcpy(hdr+12, "fmt ", 4);
  hdr[16]=16; hdr[17]=0; hdr[18]=0; hdr[19]=0;
  hdr[20]=1; hdr[21]=0;                 // PCM
  hdr[22]=1; hdr[23]=0;                 // mono
  hdr[24]=SAMPLE_RATE&0xFF; hdr[25]=(SAMPLE_RATE>>8)&0xFF;
  hdr[26]=(SAMPLE_RATE>>16)&0xFF; hdr[27]=(SAMPLE_RATE>>24)&0xFF;
  hdr[28]=byteRate&0xFF; hdr[29]=(byteRate>>8)&0xFF;
  hdr[30]=(byteRate>>16)&0xFF; hdr[31]=(byteRate>>24)&0xFF;
  hdr[32]=2; hdr[33]=0;                 // block align
  hdr[34]=16; hdr[35]=0;                // bits per sample
  memcpy(hdr+36, "data", 4);
  hdr[40]=dataLen&0xFF; hdr[41]=(dataLen>>8)&0xFF;
  hdr[42]=(dataLen>>16)&0xFF; hdr[43]=(dataLen>>24)&0xFF;
}

void sendVoice() {
  Serial.println("🎙️ Отправка голосового...");

  uint32_t dataLen = bufferIndex * sizeof(int16_t);
  uint8_t wavHeader[44];
  buildWavHeader(wavHeader, dataLen);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi нет для отправки голосового");
    return;
  }
  client.setInsecure();
  if (!client.connect(TG_HOST, 443)) {
    Serial.println("❌ Нет соединения с Telegram (голос)");
    return;
  }

  voiceNum++;
  String boundary = "Boundary7MA4YWxkTrZu0gX";
  String url      = "/bot" + String(BOT_TOKEN) + "/sendVoice";
  String head =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n" +
    String(CHAT_ID) + "\r\n"
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
    "🎙️ Голосовое #" + String(voiceNum) + "\r\n"
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"voice\"; filename=\"voice.wav\"\r\n"
    "Content-Type: audio/wav\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";
  uint32_t total = head.length() + 44 + dataLen + tail.length();

  client.println("POST " + url + " HTTP/1.1");
  client.println("Host: " + String(TG_HOST));
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.println("Content-Length: " + String(total));
  client.println("Connection: close");
  client.println();
  writeAll(client, (const uint8_t*)head.c_str(), head.length());
  writeAll(client, wavHeader, 44);
  size_t sentAudio = writeAll(client, (uint8_t*)audioBuffer, dataLen);
  writeAll(client, (const uint8_t*)tail.c_str(), tail.length());
  Serial.printf("📤 Отправлено байт аудио: %u / %u\n", (unsigned)sentAudio, (unsigned)dataLen);

  unsigned long timeout = millis() + 20000;
  while (!client.available()) {
    if (millis() > timeout) { Serial.println("❌ Таймаут (голос)"); client.stop(); return; }
    delay(50);
  }
  String fullResp = "";
  unsigned long readDeadline = millis() + 5000;
  while (client.connected() && millis() < readDeadline) {
    while (client.available()) {
      fullResp += (char)client.read();
      readDeadline = millis() + 2000; // продлеваем, пока идут данные
    }
  }
  client.stop();
  Serial.println("── Ответ Telegram (голос) ──");
  Serial.println(fullResp);
  Serial.println("─────────────────────────────");
  if (fullResp.indexOf("HTTP/1.1 200") >= 0 && fullResp.indexOf("\"ok\":true") > 0) {
    Serial.println("✅ Голосовое отправлено!");
  } else {
    Serial.println("❌ Ошибка голоса (см. ответ выше)");
  }
}

// ──────────────── ОСНОВНОЙ ЦИКЛ ────────────────
void loop() {
  static unsigned long lastRecon = 0;
  if (WiFi.status() != WL_CONNECTED && millis() - lastRecon > 15000) {
    lastRecon = millis();
    Serial.println("🔄 Переподключаюсь...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    WiFi.setSleep(false);
  }

  // Фото шлём только когда не идёт запись звука (чтобы не сбивать тайминг сэмплов)
  if (currentState != RECORDING && millis() - lastPhoto >= PHOTO_INTERVAL) {
    lastPhoto = millis();
    sendPhoto();
  }

  if (currentState == LISTENING) {
    int level = analogRead(MIC_PIN);

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
      lastDebug = millis();
      Serial.printf("🔊 уровень: %d | порог: %d | шум(база): %d\n", level, threshold, noiseLevel);
    }

    // медленно подстраиваем базовый уровень шума, только когда тихо
    // (чтобы порог не "уплывал" от температуры/времени, но не полз вверх от громкого звука)
    if (abs(level - noiseLevel) < 80) {
      noiseLevel = (int)(noiseLevel * 0.999f + level * 0.001f);
      threshold = noiseLevel + 150;
    }

    if (level > threshold) {
      Serial.println("🎙️ Обнаружен звук, начинаю запись...");
      bufferIndex = 0;
      recordingStart = millis();
      lastSampleTime = micros();
      runningAvg = level; // старт антиспайк-фильтра с текущего уровня
      currentState = RECORDING;
      digitalWrite(LED_PIN, HIGH);
    }
  }
  else if (currentState == RECORDING) {
    if (micros() - lastSampleTime >= sampleInterval) {
      if (bufferIndex < BUFFER_SIZE) {
        // oversampling: 2 быстрых замера на 1 сэмпл — меньше шума от самого ADC
        int raw = (analogRead(MIC_PIN) + analogRead(MIC_PIN)) / 2;

        // антиспайк: если скачок слишком резкий для реального звука (наводка/помеха),
        // приглушаем его до разумного предела вместо того, чтобы пропустить как есть
        const int SPIKE_LIMIT = 300;
        if (raw > runningAvg + SPIKE_LIMIT) raw = runningAvg + SPIKE_LIMIT;
        if (raw < runningAvg - SPIKE_LIMIT) raw = runningAvg - SPIKE_LIMIT;
        runningAvg = (runningAvg * 7 + raw) / 8; // медленно следует за сигналом

        // центрирование относительно базового уровня (без фиксированного усиления —
        // громкость подтянем один раз после записи через нормализацию)
        int32_t centered = (int32_t)(raw - noiseLevel) * 2;
        if (centered > 32767) centered = 32767;   // защита от переполнения int16_t
        if (centered < -32768) centered = -32768;
        audioBuffer[bufferIndex++] = (int16_t)centered;
      }
      lastSampleTime += sampleInterval;
    }

    if (millis() - recordingStart >= RECORD_SECONDS * 1000) {
      digitalWrite(LED_PIN, LOW);
      Serial.printf("Сэмплов записано: %d\n", bufferIndex);

      // ── нормализация громкости ──
      // находим пиковую амплитуду в записи и растягиваем весь клип до TARGET_PEAK,
      // чтобы тихие/далёкие звуки стали такими же громкими и чёткими, как близкие
      int16_t peak = 0;
      for (int i = 0; i < bufferIndex; i++) {
        int16_t a = abs(audioBuffer[i]);
        if (a > peak) peak = a;
      }
      if (peak > 20) { // не усиливаем полную тишину в шум
        float gain = (float)TARGET_PEAK / (float)peak;
        if (gain > MAX_NORMALIZE_GAIN) gain = MAX_NORMALIZE_GAIN;
        for (int i = 0; i < bufferIndex; i++) {
          int32_t v = (int32_t)(audioBuffer[i] * gain);
          if (v > 32767) v = 32767;
          if (v < -32768) v = -32768;
          audioBuffer[i] = (int16_t)v;
        }
        Serial.printf("🔊 Нормализация: пик был %d, усиление x%.1f\n", peak, gain);
      }

      // fade-in / fade-out — убирает щелчок на границах записи
      for (int i = 0; i < FADE_SAMPLES && i < bufferIndex; i++) {
        float g = (float)i / FADE_SAMPLES;
        audioBuffer[i] = (int16_t)(audioBuffer[i] * g);
        int idx = bufferIndex - 1 - i;
        audioBuffer[idx] = (int16_t)(audioBuffer[idx] * g);
      }

      if (bufferIndex > 1000) {
        sendVoice();
      }
      currentState = LISTENING;
    }
  }
}
