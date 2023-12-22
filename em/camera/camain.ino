#include <M5Core2.h>


SemaphoreHandle_t xMutex = NULL;

void sendData(void *args) {
  Serial.println("Thread1 Start");
  BaseType_t xStatus;
  const TickType_t xTicksToWait = 1000UL;
  xSemaphoreGive(xMutex);

  uint32_t num = 1;
  for(;;) {
    xStatus = xSemaphoreTake(xMutex, xTicksToWait);
    if (xStatus == pdTRUE) {
      Serial2.printf("Core2 Send:%d\n", num);
      M5.Lcd.fillRect(0, 50, 240, 20, TFT_BLACK);
      M5.Lcd.setCursor(0, 50);
      M5.Lcd.printf("Core2 Send:%d\n", num);
      Serial.printf("Core2 Send:%d\n", num);
      num = num + 1;
    }
    xSemaphoreGive(xMutex);
    vTaskDelay(500);
  }
}

void recvData(void *args) {
  Serial.println("Thread2 Start");
  BaseType_t xStatus;
  const TickType_t xTicksToWait = 1000UL;
  xSemaphoreGive(xMutex);
  for(;;) {
    xStatus = xSemaphoreTake(xMutex, xTicksToWait);
    if (xStatus == pdTRUE) {
      while (Serial2.available()) {
        String recv_str = Serial2.readStringUntil('\n');
        Serial.println(recv_str);
        M5.Lcd.setCursor(0, 120);
        M5.Lcd.printf("Recv:\n");
        M5.Lcd.fillRect(0, 135, 240, 20, TFT_BLACK);
        M5.Lcd.setCursor(0, 135);
        M5.Lcd.print(recv_str);

      }
    }
    xSemaphoreGive(xMutex);
    vTaskDelay(33);
  }
}


void setup() {
  M5.begin(true, true, true, false);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
  Serial2.begin(57600, SERIAL_8N1, 32, 33); 
  delay(1000);
  Serial.println("-- HELLO (baud rate = 57600)");
  M5.Lcd.println("UART Monitor");
  delay(2000);

  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("UART Monitor");

  xMutex = xSemaphoreCreateMutex();
  if (xMutex != NULL) {
    xTaskCreateUniversal( sendData,"task1", 2048, NULL, 5, NULL, 1);
    xTaskCreateUniversal( recvData,"task2", 8192, NULL, 6, NULL, tskNO_AFFINITY);
  }
  
}

void loop() {
  vTaskDelay(1);
}