#include <BleKeyboard.h>

#include "stdlib.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "esp_log.h"
#include "math.h"
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif



// define button pin
const uint32_t  BUTTON_GPIO_X = 5;
const uint32_t  BUTTON_GPIO_Y = 16;
const uint32_t  BUTTON_GPIO_A = 17;
const uint32_t  BUTTON_GPIO_B = 4;
const uint32_t  BUTTON_GPIO_L = 32;
const uint32_t  BUTTON_GPIO_ZL = 14;
const uint32_t  BUTTON_GPIO_R = 23;
const uint32_t  BUTTON_GPIO_ZR = 19;
const uint32_t  BUTTON_GPIO_DU = 33;
const uint32_t  BUTTON_GPIO_DD = 26;
const uint32_t  BUTTON_GPIO_DL = 25;
const uint32_t  BUTTON_GPIO_DR = 27;

// define mapping pin 
char map_x = 'a';
char map_y = 'c';
char map_a = 'x';
char map_b = 'z';
char map_l = 'T';
char map_r = 'i';
char map_zl = 's';
char map_zr = 'd';



// define adc pin
const uint32_t  LJOY_GPIO_X = 13;
const uint32_t  LJOY_GPIO_Y = 12;
const uint32_t  RJOY_GPIO_X = 2;
const uint32_t  RJOY_GPIO_Y = 15;

// define MPU pin
const uint32_t  PIN_SDA = 21;
const uint32_t  PIN_SCL = 22;


// Setting define
#define MPU_OPEN  1
#define BLE_OPEN  1
#define BTN_OPEN  1
#define JOY_OPEN  1


#define FADE_TIME 3000000000
#define CNT_NUM   30
#define GPIO_AVALIABLE 64


static SemaphoreHandle_t sleep_sem;

TaskHandle_t senderHandle;
TaskHandle_t btnHandle;
QueueHandle_t xCastQueue;

volatile unsigned long *btn_count;
unsigned long tStart, tEnd;
int32_t joy_val[4];

BleKeyboard bleKeyboard;
Adafruit_MPU6050 mpu;


struct ring_buf {
  float *buf;
  int8_t len;
  int8_t rear;
  int8_t front;
};


// 12 stupid ISR tasks
static void IRAM_ATTR gpio_isr_X()
{
    uint32_t gpio_num = BUTTON_GPIO_X;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_Y()
{
    uint32_t gpio_num = BUTTON_GPIO_Y;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_A()
{
    uint32_t gpio_num = BUTTON_GPIO_A;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_B()
{
    uint32_t gpio_num = BUTTON_GPIO_B;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_L()
{
    uint32_t gpio_num = BUTTON_GPIO_L;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_ZL()
{
    uint32_t gpio_num = BUTTON_GPIO_ZL;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_R()
{
    uint32_t gpio_num = BUTTON_GPIO_R;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_ZR()
{
    uint32_t gpio_num = BUTTON_GPIO_ZR;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_DU()
{
    uint32_t gpio_num = BUTTON_GPIO_DU;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_DD()
{
    uint32_t gpio_num = BUTTON_GPIO_DD;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_DL()
{
    uint32_t gpio_num = BUTTON_GPIO_DL;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}

static void IRAM_ATTR gpio_isr_DR()
{
    uint32_t gpio_num = BUTTON_GPIO_DR;
    static BaseType_t xHigherPriorityTaskWoken;
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);

    if( xHigherPriorityTaskWoken )
    {
        portYIELD_FROM_ISR ();
    }
}


static int32_t norm_joy(int32_t raw) {
    if(raw > 1600 && raw < 2000) {
        return 0;
    }else if(raw < 100) {
        return -1;
    }else if(raw > 3500) {
        return 1;
    }
}

void gpio_config() {
  pinMode(BUTTON_GPIO_X, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_Y, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_A, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_B, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_L, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_ZL, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_R, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_ZR, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_DU, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_DD, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_DL, INPUT_PULLUP);
  pinMode(BUTTON_GPIO_DR, INPUT_PULLUP);
  pinMode(LJOY_GPIO_X, INPUT_PULLUP);
  pinMode(LJOY_GPIO_Y, INPUT_PULLUP); 
  pinMode(RJOY_GPIO_X, INPUT_PULLUP);
  pinMode(RJOY_GPIO_Y, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_X), gpio_isr_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_Y), gpio_isr_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_A), gpio_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_B), gpio_isr_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_L), gpio_isr_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_ZL), gpio_isr_ZL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_R), gpio_isr_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_ZR), gpio_isr_ZR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_DU), gpio_isr_DU, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_DD), gpio_isr_DD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_DL), gpio_isr_DL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO_DR), gpio_isr_DR, CHANGE);
  
}

void mpu_init() {
    if (!mpu.begin()) {
      Serial.println("[MPU] : Failed to find MPU6050 chip");
        while (1) {
          delay(10);
        }
    }
    Serial.println("[MPU] : Device Found!");
    

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
      }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
  }

  Serial.println("");
  vTaskDelay(100 / portTICK_RATE_MS);
}


/* Select correspond button event to send*/
static void button_event(uint32_t gpio_num) {
    
    gpio_num_t port = (gpio_num_t)gpio_num;
    char btn_addr = 0x0;
    bool dpad = false;
    Serial.print("gpio");
    Serial.println(port);

    
    switch (port)
    {
    case BUTTON_GPIO_X:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_x);
        } else {
          bleKeyboard.release(map_x);
        }
        break;
    case BUTTON_GPIO_Y:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_y);
        } else {
          bleKeyboard.release(map_y);
        }
        break;
    case BUTTON_GPIO_A:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_a);
        } else {
          bleKeyboard.release(map_a);
        }
        break;
    case BUTTON_GPIO_B:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_b);
        } else {
          bleKeyboard.release(map_b);
        }
        break;
    case BUTTON_GPIO_L:
        // TAB
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(KEY_TAB);
        } else {
          bleKeyboard.release(KEY_TAB);
        }
        break;
    case BUTTON_GPIO_R:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_r);
        } else {
          bleKeyboard.release(map_r);
        }
        
        break;
    case BUTTON_GPIO_ZL:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_zl);
        } else {
          bleKeyboard.release(map_zl);
        }
        break;
    case BUTTON_GPIO_ZR:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(map_zr);
        } else {
          bleKeyboard.release(map_zr);
        }
        break;
    case BUTTON_GPIO_DU:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(KEY_UP_ARROW);
          bleKeyboard.press('a');
          delay(10);
          bleKeyboard.release(KEY_UP_ARROW);
          bleKeyboard.release('a');
        }

        break;
    case BUTTON_GPIO_DD:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(KEY_DOWN_ARROW);
          bleKeyboard.press('a');
          delay(10);
          bleKeyboard.release(KEY_DOWN_ARROW);
          bleKeyboard.release('a');
        }
        break;
    case BUTTON_GPIO_DL:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(KEY_LEFT_ARROW);
          bleKeyboard.press('a');
          delay(10);
          bleKeyboard.release(KEY_LEFT_ARROW);
          bleKeyboard.release('a');
        }
        break;
    case BUTTON_GPIO_DR:
        if (digitalRead(port) == LOW) {
          bleKeyboard.press(KEY_RIGHT_ARROW);
          bleKeyboard.press('a');
          delay(10);
          bleKeyboard.release(KEY_RIGHT_ARROW);
          bleKeyboard.release('a');
        }
        break;
    default:
        Serial.println("[Button] button address didn't exist");
        break;
    }


    
    // reset timer
    tStart = millis();
}


// Task handler field

void vEventSender(void *pvParameters) {

  while(1) {
      if(!bleKeyboard.isConnected()) {
        Serial.println("[Event Sender] : wait for connecting...");
        vTaskDelay(2000 / portTICK_RATE_MS);
      }
      


      if(JOY_OPEN) {
        
        joy_val[0]= analogRead(LJOY_GPIO_X);
        joy_val[1]= analogRead(LJOY_GPIO_Y);
        joy_val[2]= analogRead(RJOY_GPIO_X);
        joy_val[3]= analogRead(RJOY_GPIO_Y);

        Serial.print("[Joy] raw X ");
        Serial.print(joy_val[0]);
        Serial.print("  raw Y ");
        Serial.println
        (joy_val[1]);
        
        if (BLE_OPEN) {
          if (norm_joy(joy_val[0]) == 0) {
            bleKeyboard.release(KEY_UP_ARROW);
            bleKeyboard.release(KEY_DOWN_ARROW);
          }else if(norm_joy(joy_val[0]) == 1) {
            bleKeyboard.press(KEY_UP_ARROW);
            bleKeyboard.release(KEY_DOWN_ARROW);
          }else if(norm_joy(joy_val[0]) == -1) {
            bleKeyboard.release(KEY_UP_ARROW);
            bleKeyboard.press(KEY_DOWN_ARROW);
          }
          if (norm_joy(joy_val[1]) == 0) {
            bleKeyboard.release(KEY_LEFT_ARROW);
            bleKeyboard.release(KEY_RIGHT_ARROW);
          }else if(norm_joy(joy_val[1]) == 1) {
            bleKeyboard.press(KEY_LEFT_ARROW);
            bleKeyboard.release(KEY_RIGHT_ARROW);
          }else if(norm_joy(joy_val[1]) == -1) {
            bleKeyboard.release(KEY_LEFT_ARROW);
            bleKeyboard.press(KEY_RIGHT_ARROW);
          }
        }
      }
      vTaskDelay(50 / portTICK_RATE_MS);
  }
  
}

void vButton(void *pvParameters) {
  uint32_t gpio_num;
  BaseType_t receive_success;
  unsigned long tNow;


    Serial.println("[Button] : Start button task. ");

    while(1) {
             
        tNow = millis();

        // Clean up all the queue element
        while(xQueuePeek(xCastQueue, &gpio_num, (TickType_t)0) == pdTRUE) {
            // Do not block if receive no data
            receive_success = xQueueReceive(xCastQueue, &gpio_num, ( TickType_t ) 0 );
            if(receive_success == pdTRUE) {
                if (btn_count[gpio_num] == 0) {
                    // First receive event, record current time
                    Serial.print("[Button] receive ");
                    Serial.println(gpio_num);
                    
                    btn_count[gpio_num] = millis();
                }
                else {
                    // Still in count down process, just ignore
                    continue;
                }
            }
            else {
                // Queue empty
                Serial.println("[Button] : This statement cannot happen");
                break;
            }
        }
        
        // Wait time up to trigger button event
        int rest_cnt = 0;
        for(int i = 0; i < GPIO_AVALIABLE; i++) {
            if((btn_count[i] != 0) && ((tNow - btn_count[i]) >= CNT_NUM)) {
                // Judge event happend and press/release button
                btn_count[i] = 0;
                button_event(i);
            }
            else if(btn_count[i] == 0) {
                rest_cnt++;
            }
        }
        if(rest_cnt == GPIO_AVALIABLE) {
            xSemaphoreTake(sleep_sem, portMAX_DELAY);
        }
    }
}


void vTaskMPU6050(void *pvParameters) {
  
  sensors_event_t a, g, temp;
  float angle_x, angle_y;
 
  while(1) {
    mpu.getEvent(&a, &g, &temp);
    angle_x = g.gyro.x;
    angle_y = g.gyro.y;
     Serial.print("[MPU]angle x: ");
     Serial.print(angle_x);
     Serial.print(", angle y: ");
     Serial.println(angle_y);

    bool send_msg = false;
    if (angle_x > 4) {
      send_msg = true;
      Serial.println("LEFT special");
      if (BLE_OPEN) {
        bleKeyboard.press(KEY_LEFT_ARROW);
        bleKeyboard.press('a');
        delay(10);
        bleKeyboard.release(KEY_LEFT_ARROW);
        bleKeyboard.release('a');
      }
    }
    else if (angle_x < -4) {
      send_msg = true;
      Serial.println("RIGHT special");
      if (BLE_OPEN) {
        bleKeyboard.press(KEY_RIGHT_ARROW);
        bleKeyboard.press('a');
        delay(10);
        bleKeyboard.release(KEY_RIGHT_ARROW);
        bleKeyboard.release('a');
      }
    }


    if (angle_y > 4) {
      send_msg = true;

      Serial.println("UP special");
      if (BLE_OPEN) {
        bleKeyboard.press(KEY_UP_ARROW);
        bleKeyboard.press('a');
        delay(10);
        bleKeyboard.release(KEY_UP_ARROW);
        bleKeyboard.release('a');
      }
    }
    else if (angle_y < -4) {
      send_msg = true;
      Serial.println("DOWN special");
      if (BLE_OPEN) {
        bleKeyboard.press(KEY_DOWN_ARROW);
        bleKeyboard.press('a');
        delay(10);
        bleKeyboard.release(KEY_DOWN_ARROW);
        bleKeyboard.release('a');
      }
    }


    if (send_msg) {
      vTaskDelay(400 / portTICK_RATE_MS);
    }
    
    vTaskDelay(35 / portTICK_RATE_MS);
  }
}


void setup() {
  
  BaseType_t task_init;
  Serial.begin(115200);
  Serial.println("[main] Gamepad init... ");
  mpu_init();
  gpio_config();
  
  bleKeyboard.begin();

  // Debounce buffer setup
  btn_count = (unsigned long *)malloc(GPIO_AVALIABLE * sizeof(unsigned long));
  for(int i = 0; i < GPIO_AVALIABLE; i++) {
    btn_count[i] = 0;
  }

  // 
 
  sleep_sem = xSemaphoreCreateBinary();
  xCastQueue = xQueueCreate(16, sizeof(struct b_event *));


  
  // create task
  task_init = xTaskCreate(vEventSender, "EventSender", 8198, NULL, 2, &senderHandle);
  configASSERT(task_init);
  
  if (MPU_OPEN) {
    task_init = xTaskCreate(vTaskMPU6050, "MPUtask", 8198, NULL, 3, NULL);
    configASSERT(task_init);
  }

  if (BTN_OPEN) {
    task_init = xTaskCreate(vButton, "Interrupt test", 4096, NULL, 4, &btnHandle);
    configASSERT(task_init);
  }

  
  
  Serial.println("[main]System Ready!");
}



void loop() {
}
