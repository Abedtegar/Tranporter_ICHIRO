# Tranporter_ICHIRO
# ESP32 Transporter Control System with WiFi Web Server

Robot transporter dengan sistem kontrol ESP32 yang dilengkapi dengan sensor dan web interface.

## Deskripsi
Sistem kontrol ESP32 yang dilengkapi dengan:
- Sensor GY-25 (gyroscope) untuk pembacaan Roll, Pitch, Yaw
- Sensor TCS34725 (color sensor) untuk deteksi warna dan lux
- 2 motor DC dengan encoder untuk kontrol pergerakan
- WiFi Web Server untuk monitoring dan kontrol jarak jauh

## Fitur
1. **Mode Otomatis**: Motor beroperasi dalam siklus otomatis dengan berbagai pola putaran
2. **Mode Manual**: Kontrol motor secara manual melalui web interface
3. **Monitoring Real-time**: Semua data sensor ditampilkan secara real-time
4. **Emergency Stop**: Tombol darurat untuk menghentikan semua motor
5. **Web Interface**: Interface web yang responsif untuk monitoring dan kontrol

## Pin Configuration

### Motor Control
- EN1 (Motor 1 PWM): Pin 23
- EN2 (Motor 2 PWM): Pin 19
- IN1 (Motor 1 Direction A): Pin 18
- IN2 (Motor 1 Direction B): Pin 5
- IN3 (Motor 2 Direction A): Pin 4
- IN4 (Motor 2 Direction B): Pin 2

### Encoder
- Encoder 1 A: Pin 14
- Encoder 1 B: Pin 27
- Encoder 2 A: Pin 26
- Encoder 2 B: Pin 25

### Sensors
- GY-25 RX: Pin 16
- GY-25 TX: Pin 17
- TCS34725: I2C (SDA/SCL default pins)

## Setup

### 1. Konfigurasi WiFi
Edit file `main.cpp` dan ubah kredensial WiFi:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```

### 2. Upload Program
1. Pastikan semua library sudah terinstall (akan otomatis terinstall saat build)
2. Hubungkan ESP32 ke komputer
3. Upload program menggunakan PlatformIO

### 3. Monitor Serial
Buka Serial Monitor (115200 baud) untuk melihat:
- Status koneksi WiFi
- IP address yang diberikan
- Data sensor real-time

## Penggunaan Web Interface

### Akses Web Interface
1. Pastikan ESP32 dan perangkat Anda terhubung ke WiFi yang sama
2. Buka browser dan akses IP address yang ditampilkan di Serial Monitor
3. Contoh: `http://192.168.1.100`

### Fitur Web Interface

#### 1. System Status
- Menampilkan status koneksi
- Toggle antara mode Auto/Manual
- Tombol Emergency Stop

#### 2. Sensor Data (Real-time)
- **Gyroscope**: Roll, Pitch, Yaw (dalam derajat)
- **Color Sensor**: Nilai R, G, B, dan Lux
- **Encoder**: Posisi dan arah putaran motor
- **Motor State**: Status motor saat ini

#### 3. Manual Motor Control
- Slider untuk mengatur kecepatan Motor 1 (-100% to +100%)
- Slider untuk mengatur kecepatan Motor 2 (-100% to +100%)
- Nilai positif = putaran searah jarum jam
- Nilai negatif = putaran berlawanan jarum jam

## API Endpoints

### GET Endpoints
- `/` - Halaman utama web interface
- `/data` - JSON data semua sensor dan status motor

### POST Endpoints
- `/toggleMode` - Toggle antara mode Auto/Manual
- `/setMotor` - Set kecepatan motor (mode manual)
- `/emergencyStop` - Stop semua motor

### Contoh API Usage
```javascript
// Toggle mode
fetch('/toggleMode', {method: 'POST'})

// Set motor speed
fetch('/setMotor', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({motor: 1, speed: 50})
})

// Emergency stop
fetch('/emergencyStop', {method: 'POST'})
```

## Mode Operasi

### Mode Otomatis
Motor beroperasi dalam siklus berikut:
1. Motor 1 CW (0-100%)
2. Motor 1 CW (100-0%)
3. Motor 1 CCW (0-100%)
4. Motor 1 CCW (100-0%)
5. Motor 2 CW (0-100%)
6. Motor 2 CW (100-0%)
7. Motor 2 CCW (0-100%)
8. Motor 2 CCW (100-0%)
9. Kembali ke langkah 1

### Mode Manual
- Kontrol independen untuk setiap motor
- Range kecepatan: -100% hingga +100%
- Update real-time melalui web interface

## Troubleshooting

### WiFi Tidak Terhubung
1. Periksa SSID dan password WiFi
2. Pastikan WiFi dalam jangkauan
3. Periksa Serial Monitor untuk error message

### Web Interface Tidak Dapat Diakses
1. Pastikan ESP32 dan perangkat dalam jaringan yang sama
2. Periksa IP address di Serial Monitor
3. Coba restart ESP32

### Motor Tidak Bergerak
1. Periksa koneksi motor driver
2. Pastikan power supply mencukupi
3. Gunakan Emergency Stop lalu coba lagi

### Sensor Tidak Berfungsi
1. **GY-25**: Periksa koneksi Serial (pin 16, 17)
2. **TCS34725**: Periksa koneksi I2C
3. Monitor Serial untuk error message sensor

## Safety Features
- Emergency Stop: Menghentikan semua motor segera
- Auto-timeout: Koneksi web akan timeout jika tidak ada aktivitas
- Error handling: System akan tetap berjalan meski ada sensor yang error

## Dependencies
- ESP32 Arduino Core
- Adafruit TCS34725 Library
- ArduinoJson Library
- WiFi Library (built-in)
- WebServer Library (built-in)
