# Temperature & Humidity Controller – STM32

## Main Functions
- Read temperature & humidity by **DHT11** (every 2 seconds).  
- Display parameters on **LCD I2C 16x2**.  
- Adjust temperature and humidity threshold by 4 buttons (**OK – UP – DOWN – RIGHT**).  
- Control **heating lamp** and **humidifier** based on setting value.  
- Control **stepper motor** to turn eggs periodically.

---

## Pin Connection (STM32F1)

### 1. LCD 16x2 I2C
- LCD VCC  → 5V  
- LCD GND  → GND  
- LCD SCL  → **PB6** (I2C1_SCL)  
- LCD SDA  → **PB7** (I2C1_SDA)  
- I2C address: `0x27 << 1` 

### 2. DHT11 Sensor
- DHT11 VCC  → 5V  
- DHT11 GND  → GND  
- DHT11 DATA → `PA2` 
- Timer: **TIM2**

### 3. Outputs (Relay / Driver)
- Heating lamp relay IN   → `PB5`  
- Humidifier relay IN     → `PB4`  
- Stepper driver STEP pin → `PB8`   

### 4. Buttons (OK – UP – DOWN – RIGHT)
Using pull-up:

- OK button    → `PB0`  
- UP button    → `PB1`  
- DOWN button  → `PB10` 
- RIGHT button → `PB11`

### 5. Module RTC DS1302
Using Half-Duplex Master, but due to module error it cannot be added

- DS1302 VCC → 5V
- DS1302 GND → GND
- DS1302 CLK → PA5 (SPI1_SCK)
- DS1302 DAT → PA7 (SPI1_MOSI)
- DS1302 RST → PA4
