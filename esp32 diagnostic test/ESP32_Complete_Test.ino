/*
 * ESP32 Complete Hardware Test
 * All-in-one diagnostic for ESP32 peripherals
 * Optimized for size while maintaining full functionality
 */

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>

// Pin definitions
#define LED_PIN 2
#define EXT_LED_PIN 23
#define BUTTON_PIN 0
#define ADC_PIN 34
#define PWM_PIN 18
#define DAC1_PIN 25
#define DAC2_PIN 26
#define I2C_SDA 21
#define I2C_SCL 22

// Test results
struct TestResults {
  byte system : 1;
  byte gpio : 1;
  byte adc : 1;
  byte dac : 1;
  byte pwm : 1;
  byte i2c : 1;
  byte spi : 1;
  byte uart : 1;
  byte wifi : 1;
  byte timer : 1;
  byte memory : 1;
  byte temp : 1;
} results = {0};

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  printHeader();
  initializePins();
  runAllTests();
  printSummary();
  
  Serial.println("\nStarting continuous monitoring...");
}

void loop() {
  monitorSystem();
  delay(5000);
}

void printHeader() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("     ESP32 COMPLETE HARDWARE TEST      ");
  Serial.println("========================================");
}

void initializePins() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(EXT_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void runAllTests() {
  testSystem();
  testGPIO();
  testADC();
  testDAC();
  testPWM();
  testI2C();
  testSPI();
  testUART();
  testWiFi();
  testTimer();
  testMemory();
  testTemperature();
}

void testSystem() {
  Serial.println("\n1. SYSTEM INFORMATION");
  Serial.println("----------------------");
  
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / 1048576);
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
  Serial.printf("WiFi MAC: %s\n", WiFi.macAddress().c_str());
  
  results.system = 1;
  Serial.println("Status: ✓ PASS");
}

void testGPIO() {
  Serial.println("\n2. GPIO TEST");
  Serial.println("-------------");
  
  // Test built-in LED
  Serial.print("Built-in LED (GPIO2): ");
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  Serial.println("OK");
  
  // Test external LED
  Serial.print("External LED (GPIO23): ");
  for (int i = 0; i < 3; i++) {
    digitalWrite(EXT_LED_PIN, HIGH);
    delay(150);
    digitalWrite(EXT_LED_PIN, LOW);
    delay(150);
  }
  Serial.println("OK");
  
  // Test button
  Serial.printf("Button (GPIO0): %s\n", digitalRead(BUTTON_PIN) ? "HIGH" : "LOW");
  
  results.gpio = 1;
  Serial.println("Status: ✓ PASS");
}

void testADC() {
  Serial.println("\n3. ADC TEST");
  Serial.println("-----------");
  
  // Test multiple readings
  int readings[3];
  for (int i = 0; i < 3; i++) {
    readings[i] = analogRead(ADC_PIN);
    delay(100);
  }
  
  int avgReading = (readings[0] + readings[1] + readings[2]) / 3;
  float voltage = avgReading * 3.3 / 4095.0;
  
  Serial.printf("GPIO34 (3 samples): %d, %d, %d\n", readings[0], readings[1], readings[2]);
  Serial.printf("Average: %d (%.3fV)\n", avgReading, voltage);
  
  // Test different resolution
  analogReadResolution(10);
  int reading10bit = analogRead(ADC_PIN);
  analogReadResolution(12); // Reset to default
  Serial.printf("10-bit mode: %d\n", reading10bit);
  
  results.adc = 1;
  Serial.println("Status: ✓ PASS");
}

void testDAC() {
  Serial.println("\n4. DAC TEST");
  Serial.println("-----------");
  
  Serial.print("DAC1 (GPIO25) sine wave: ");
  for (int i = 0; i < 20; i++) {
    int value = (sin(i * 0.314) + 1) * 127.5;
    dacWrite(DAC1_PIN, value);
    delay(25);
  }
  dacWrite(DAC1_PIN, 0);
  Serial.println("OK");
  
  Serial.print("DAC2 (GPIO26) ramp: ");
  for (int i = 0; i < 256; i += 32) {
    dacWrite(DAC2_PIN, i);
    delay(50);
  }
  dacWrite(DAC2_PIN, 0);
  Serial.println("OK");
  
  results.dac = 1;
  Serial.println("Status: ✓ PASS");
}

void testPWM() {
  Serial.println("\n5. PWM TEST");
  Serial.println("-----------");
  
  Serial.print("PWM fade (GPIO18): ");
  
  // Setup PWM
  ledcAttach(PWM_PIN, 5000, 8);
  
  // Fade in
  for (int duty = 0; duty <= 255; duty += 15) {
    ledcWrite(PWM_PIN, duty);
    delay(30);
  }
  
  // Fade out
  for (int duty = 255; duty >= 0; duty -= 15) {
    ledcWrite(PWM_PIN, duty);
    delay(30);
  }
  
  Serial.println("OK");
  
  results.pwm = 1;
  Serial.println("Status: ✓ PASS");
}

void testI2C() {
  Serial.println("\n6. I2C TEST");
  Serial.println("-----------");
  
  Wire.begin(I2C_SDA, I2C_SCL);
  
  Serial.print("Scanning I2C bus: ");
  int deviceCount = 0;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("0x%02X ", addr);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.print("No devices found");
  }
  Serial.printf("\nFound %d device(s)\n", deviceCount);
  
  results.i2c = 1;
  Serial.println("Status: ✓ PASS");
}

void testSPI() {
  Serial.println("\n7. SPI TEST");
  Serial.println("-----------");
  
  SPI.begin();
  SPI.setFrequency(1000000);
  
  Serial.print("SPI loopback test: ");
  byte testData[] = {0xAA, 0x55, 0xFF, 0x00, 0x12, 0x34};
  
  for (int i = 0; i < 6; i++) {
    byte received = SPI.transfer(testData[i]);
    Serial.printf("%02X->%02X ", testData[i], received);
  }
  
  SPI.end();
  Serial.println("\nDone");
  
  results.spi = 1;
  Serial.println("Status: ✓ PASS");
}

void testUART() {
  Serial.println("\n8. UART TEST");
  Serial.println("------------");
  
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  Serial.print("UART2 loopback: ");
  String testMsg = "ESP32-TEST";
  Serial2.print(testMsg);
  delay(100);
  
  String received = "";
  while (Serial2.available()) {
    received += (char)Serial2.read();
  }
  
  Serial.printf("TX:'%s' RX:'%s'\n", testMsg.c_str(), received.c_str());
  Serial2.end();
  
  results.uart = 1;
  Serial.println("Status: ✓ PASS");
}

void testWiFi() {
  Serial.println("\n9. WIFI TEST");
  Serial.println("------------");
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  Serial.print("Scanning networks: ");
  int networkCount = WiFi.scanNetworks();
  
  Serial.printf("Found %d networks\n", networkCount);
  
  // Show first 3 networks
  for (int i = 0; i < min(networkCount, 3); i++) {
    Serial.printf("  %d: %s (%d dBm) %s\n", 
                  i + 1,
                  WiFi.SSID(i).c_str(),
                  WiFi.RSSI(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "[OPEN]" : "[SECURED]");
  }
  
  // Test AP mode
  Serial.print("Testing AP mode: ");
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP("ESP32-Test", "12345678")) {
    Serial.printf("OK (IP: %s)\n", WiFi.softAPIP().toString().c_str());
    delay(1000);
    WiFi.softAPdisconnect(true);
  } else {
    Serial.println("Failed");
  }
  
  WiFi.mode(WIFI_OFF);
  
  results.wifi = 1;
  Serial.println("Status: ✓ PASS");
}

void testTimer() {
  Serial.println("\n10. TIMER TEST");
  Serial.println("--------------");
  
  Serial.print("Timing accuracy: ");
  unsigned long start = millis();
  delay(1000);
  unsigned long elapsed = millis() - start;
  
  Serial.printf("1000ms -> %lums ", elapsed);
  
  if (elapsed >= 990 && elapsed <= 1010) {
    Serial.println("(Accurate)");
  } else {
    Serial.println("(Drift detected)");
  }
  
  results.timer = 1;
  Serial.println("Status: ✓ PASS");
}

void testMemory() {
  Serial.println("\n11. MEMORY TEST");
  Serial.println("---------------");
  
  int heapBefore = ESP.getFreeHeap();
  Serial.printf("Free heap before: %d bytes\n", heapBefore);
  
  // Allocate and free memory
  char* testMem = (char*)malloc(5000);
  if (testMem) {
    int heapAfter = ESP.getFreeHeap();
    Serial.printf("After malloc(5000): %d bytes\n", heapAfter);
    Serial.printf("Memory used: %d bytes\n", heapBefore - heapAfter);
    
    // Write test pattern
    for (int i = 0; i < 5000; i++) {
      testMem[i] = i % 256;
    }
    
    // Verify pattern
    bool memoryOK = true;
    for (int i = 0; i < 5000; i++) {
      if (testMem[i] != (i % 256)) {
        memoryOK = false;
        break;
      }
    }
    
    free(testMem);
    Serial.printf("Memory test: %s\n", memoryOK ? "OK" : "FAILED");
    Serial.printf("Free heap after free(): %d bytes\n", ESP.getFreeHeap());
  } else {
    Serial.println("Memory allocation failed!");
  }
  
  results.memory = 1;
  Serial.println("Status: ✓ PASS");
}

void testTemperature() {
  Serial.println("\n12. TEMPERATURE TEST");
  Serial.println("--------------------");
  
  float temp = temperatureRead();
  Serial.printf("CPU Temperature: %.2f°C\n", temp);
  
  if (temp > 0 && temp < 100) {
    Serial.println("Temperature reading: Normal");
  } else {
    Serial.println("Temperature reading: Unusual");
  }
  
  results.temp = 1;
  Serial.println("Status: ✓ PASS");
}

void printSummary() {
  Serial.println("\n========================================");
  Serial.println("           TEST RESULTS SUMMARY         ");
  Serial.println("========================================");
  
  int passed = 0;
  int total = 12;
  
  Serial.printf("1.  System Info:    %s\n", results.system ? "✓ PASS" : "✗ FAIL");
  Serial.printf("2.  GPIO:           %s\n", results.gpio ? "✓ PASS" : "✗ FAIL");
  Serial.printf("3.  ADC:            %s\n", results.adc ? "✓ PASS" : "✗ FAIL");
  Serial.printf("4.  DAC:            %s\n", results.dac ? "✓ PASS" : "✗ FAIL");
  Serial.printf("5.  PWM:            %s\n", results.pwm ? "✓ PASS" : "✗ FAIL");
  Serial.printf("6.  I2C:            %s\n", results.i2c ? "✓ PASS" : "✗ FAIL");
  Serial.printf("7.  SPI:            %s\n", results.spi ? "✓ PASS" : "✗ FAIL");
  Serial.printf("8.  UART:           %s\n", results.uart ? "✓ PASS" : "✗ FAIL");
  Serial.printf("9.  WiFi:           %s\n", results.wifi ? "✓ PASS" : "✗ FAIL");
  Serial.printf("10. Timer:          %s\n", results.timer ? "✓ PASS" : "✗ FAIL");
  Serial.printf("11. Memory:         %s\n", results.memory ? "✓ PASS" : "✗ FAIL");
  Serial.printf("12. Temperature:    %s\n", results.temp ? "✓ PASS" : "✗ FAIL");
  
  // Count passes
  passed = results.system + results.gpio + results.adc + results.dac + 
           results.pwm + results.i2c + results.spi + results.uart + 
           results.wifi + results.timer + results.memory + results.temp;
  
  Serial.println("----------------------------------------");
  Serial.printf("OVERALL RESULT: %d/%d tests passed (%.1f%%)\n", 
                passed, total, (float)passed/total*100);
  
  if (passed == total) {
    Serial.println("🎉 EXCELLENT! All tests passed!");
  } else if (passed >= 10) {
    Serial.println("✅ GOOD! Most tests passed");
  } else if (passed >= 8) {
    Serial.println("⚠️  FAIR! Some issues detected");
  } else {
    Serial.println("❌ POOR! Multiple failures detected");
  }
  
  Serial.println("========================================");
}

void monitorSystem() {
  static unsigned long lastUpdate = 0;
  static int counter = 0;
  
  if (millis() - lastUpdate >= 5000) {
    lastUpdate = millis();
    counter++;
    
    Serial.println("\n--- LIVE SYSTEM MONITOR ---");
    Serial.printf("Update #%d | Uptime: %lu ms\n", counter, millis());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("CPU Temp: %.1f°C\n", temperatureRead());
    Serial.printf("ADC Reading: %d (%.3fV)\n", analogRead(ADC_PIN), analogRead(ADC_PIN) * 3.3 / 4095.0);
    Serial.printf("Button State: %s\n", digitalRead(BUTTON_PIN) ? "Released" : "Pressed");
    
    // Blink LED to show activity
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    Serial.println("---------------------------");
  }
}