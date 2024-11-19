#include <Arduino.h>
#include <BleKeyboard.h>

BleKeyboard Ble_Keyboard("My_Media_control", "Espressif", 100); //编辑信息（蓝牙名称，设备制造商，电池电量）

#define A_pin GPIO_NUM_5 //A port connection pins
#define B_pin GPIO_NUM_4 //B port connection pins
#define key_pin GPIO_NUM_8 //Button port pins
#define led_pin GPIO_NUM_2

int lastButtonState = HIGH; // 上一次的按键状态
int currentButtonState = HIGH; // 当前的按键状态
unsigned long lastDebounceTime = 0; // 上一次按键去抖动的时间
unsigned long lastPressTime = 0; // 上一次按键按下的时间
bool waitingForDoubleClick = false; // 标记是否正在等待双击
const unsigned long debounceDelay = 50; // 去抖动延迟
const unsigned long doubleClickDelay = 250; // 双击的最大间隔时间

int A_pin_State; 
int A_pin_BeforeState;
bool bottomState =0; 
bool timeState = 0;
bool state[3];

uint32_t currenttime;
uint32_t periodtime = 0;

void task_bleKeyboard_init(){
    
    Ble_Keyboard.begin();
}

void task_knob_init(){
    pinMode (A_pin,INPUT); 
    pinMode (B_pin,INPUT); 
    pinMode (key_pin,INPUT);
  
    A_pin_BeforeState = digitalRead(A_pin); 
}

void task_knob(){
        state[0] = 0;//左拧状态
        state[1] = 0;//右拧状态
        state[2] = 1;//按键状态

        A_pin_State = digitalRead(A_pin);
        if (A_pin_State != A_pin_BeforeState){
            if (digitalRead(B_pin) != A_pin_BeforeState){
                
                digitalWrite(led_pin,LOW);
                state[1] = 1; //向右拧
                delay(10);
                digitalWrite(led_pin,HIGH);
            } 
            else {
                digitalWrite(led_pin,LOW);
                state[0] = 1; //向左拧
                delay(10);
                digitalWrite(led_pin,HIGH);
            }
        }
        A_pin_BeforeState = A_pin_State;
            if (digitalRead(key_pin) == LOW){
                digitalWrite(led_pin,LOW);
                state[2] = 0;
                delay(10);
                digitalWrite(led_pin,HIGH);
            }
        
    digitalWrite(led_pin,HIGH);

    // uint32_t currenttime = millis();
    // if (currenttime - periodtime >10){
    // Serial.print(state[0]);
    // Serial.print("\t");
    // Serial.print(state[1]);
    // Serial.print("\t");
    // Serial.println(state[2]);
    // periodtime = millis();
    // }
}

void task_control(){
    if(Ble_Keyboard.isConnected()) { 

    if (state[2] == 1){
    if (state[0] == 1)
        Ble_Keyboard.write(KEY_MEDIA_VOLUME_DOWN);

    if (state[1] ==1 ){
        Ble_Keyboard.write(KEY_MEDIA_VOLUME_UP);
    }
    }

    if (state[2] == 0){
    if (state[0] == 1)
        Ble_Keyboard.write(KEY_MEDIA_PREVIOUS_TRACK);

    if (state[1] ==1){
        Ble_Keyboard.write(KEY_MEDIA_NEXT_TRACK);
    }
    }

   currentButtonState = digitalRead(key_pin);

  // 检查按键是否从释放变为按下
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    // 延时去抖动
    delay(debounceDelay);
    // 再次读取按键状态，确认按键确实被按下
    if (digitalRead(key_pin) == LOW) {
      // 记录按键按下的时间
      lastPressTime = millis();
      // 开始等待双击
      waitingForDoubleClick = true;
      // 更新上一次去抖动的时间
      lastDebounceTime = millis();
    }
  }

  // 检查是否需要执行双击操作
  if (waitingForDoubleClick && (millis() - lastPressTime < doubleClickDelay)) {
    // 检查是否再次按下按键
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      // 延时去抖动
      delay(debounceDelay);
      // 再次读取按键状态，确认按键确实被按下
      if (digitalRead(key_pin) == LOW) {
        // 双击检测
        Ble_Keyboard.write(KEY_MEDIA_PLAY_PAUSE);
        // 重置等待双击的标志
        waitingForDoubleClick = false;
      }
    }
  } else if (waitingForDoubleClick && (millis() - lastPressTime >= doubleClickDelay)) {
    // 单击检测
        Ble_Keyboard.write(KEY_MEDIA_MUTE);
    // 重置等待双击的标志
    waitingForDoubleClick = false;
  }

  // 更新上一次的按键状态
  lastButtonState = currentButtonState;

  // 短暂延时，防止CPU占用过高
  delay(10);
}

}

void setup(){

    pinMode(led_pin,OUTPUT);
    digitalWrite(led_pin,LOW);
    Serial.begin(115200);

    task_bleKeyboard_init();
    task_knob_init();
    digitalWrite(led_pin,HIGH);
}

void loop(){
    task_control();
    task_knob();
}