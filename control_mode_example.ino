#include <Arduino.h>
#include <math.h>
#include <M5Stack.h>
#include "cybergear_driver.hh"



#define USE_ESP32_CAN  // If you want to use ESP32_CAN or PWRCAN, please uncomment this line
#define USE_PWRCAN     // If you want to use PWRCAN, please uncomment this line
#ifdef USE_ESP32_CAN
#include "cybergear_can_interface_esp32.hh"
#else
#include "cybergear_can_interface_mcp.hh"
#endif

#define INC_POSITION  120.0
#define INC_VELOCITY  0.4
#define INC_TORQUE    0.04

/**
 * @brief Draw display
 *
 * @param mode current, speed, position, motion mode
 * @param is_mode_change mode change flag
 */
void draw_display(uint8_t mode, bool is_mode_change = false);

/**
 * @brief Get the color and mode
 *
 * @param mode target mode
 * @param color mode color
 * @param mode_str mode string
 */
void get_color_and_mode_str(uint8_t mode, uint16_t & color, String & mode_str);

// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
uint8_t MOT_CAN_ID1 = 0x7F;
uint8_t MOT_CAN_ID2 = 0x7E;


// init cybergeardriver
CybergearDriver driver1 = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID1);
CybergearDriver driver2 = CybergearDriver(MASTER_CAN_ID, MOT_CAN_ID2);
MotorStatus motor_status1;
MotorStatus motor_status2;
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp interface;
#endif

// init sprite for display
TFT_eSprite sprite = TFT_eSprite(&sprite);

uint8_t mode = MODE_POSITION;   //!< current mode
float target_pos1 = 0.0;         //!< motor target position
float target_pos2 = 0.0;         //!< motor target position
float target_vel = 0.0;         //!< motor target velocity
float target_torque = 0.0;      //!< motor target torque
float dir = 1.0f;               //!< direction for motion mode
float default_kp = 50.0f;       //!< default kp for motion mode
float default_kd = 1.0f;        //!< default kd for motion mode
float init_speed = 5.0f;       //!< initial speed
float slow_speed = 0.8f;        //!< slow speed
bool state_change_flag1 = false; //!< state change flag
bool state_change_flag2 = false; //!< state change flag

float free_rotation_sensitivity = 0.1f; //!< トルク感度（外力の小さな変化で回転しやすくする）
float max_free_rotation_speed = 0.8f;   //!< 自由回転モード時の最大速度

float M = 0.5f;   // 質量
float B = 0.01f;   // 粘性
float K = 0.01f;   // 剛性
float v = 0.0;
float G = 0.1f; //重力補償







void setup()
{
  M5.begin();

  // init sprite
  sprite.setColorDepth(8);
  sprite.setTextSize(3);
  sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());

  // init cybergear driver
#ifdef USE_PWRCAN
  interface.init(16, 17);
#else
  interface.init();
#endif

  driver1.init(&interface);
  driver1.init_motor(mode);
  driver1.set_limit_speed(init_speed);
  driver1.enable_motor();

  driver2.init(&interface);
  driver2.init_motor(mode);
  driver2.set_limit_speed(init_speed);
  driver2.enable_motor();

  // display current status
  draw_display(mode, true);
}

void draw_display(uint8_t mode, bool is_mode_change)
{
  uint16_t bg_color;
  String mode_str;
  get_color_and_mode_str(mode, bg_color, mode_str);

  if (is_mode_change) M5.Lcd.fillScreen(bg_color);
  sprite.fillScreen(bg_color);
  sprite.setCursor(0, 0);
  sprite.setTextColor(TFT_WHITE, bg_color);

  sprite.setTextSize(4);
  sprite.println(mode_str);

  sprite.setTextSize(2);
  sprite.println("");
  sprite.println("=== Target ===");
  sprite.print("Position:");
  sprite.print(target_pos2);
  sprite.println(" rad");
  sprite.print("Velocity:");
  sprite.print(target_vel);
  sprite.println(" rad/s");
  sprite.print("Current : ");
  sprite.print(target_torque);
  sprite.println(" A");
  sprite.println("");

  sprite.println("=== Current ===");
  sprite.print("Position:");
  sprite.print(motor_status2.position);
  sprite.println(" rad");
  sprite.print("Velocity:");
  sprite.print(motor_status2.velocity);
  sprite.println(" rad/s");
  sprite.print("Effort : ");
  sprite.print(motor_status2.effort);
  sprite.println(" Nm");

  sprite.pushSprite(0, 0);
}

void get_color_and_mode_str(uint8_t mode, uint16_t & color, String & mode_str)
{
  switch (mode)
  {
    case MODE_POSITION:
      color = RED;
      mode_str = String("Position");
      break;
    case MODE_SPEED:
      color = GREEN;
      mode_str = String("Speed");
      break;
    case MODE_CURRENT:
      color = BLUE;
      mode_str = String("Current");
      break;
    case MODE_MOTION:
      color = BLACK;
      mode_str = String("Motion");
      break;
  }
}

void loop()
{
  // update m5 satatus
  M5.update();

  // check mode change
  if(M5.BtnB.wasPressed()) {
    mode = (mode + 1) % MODE_CURRENT + 1;
    state_change_flag1 = true;
    state_change_flag2 = true;
    driver1.init_motor(mode);
    driver1.enable_motor();
    driver2.init_motor(mode);
    driver2.enable_motor();
    target_pos1 = motor_status1.position;
    target_pos2 = motor_status2.position;
    target_vel = 0.0;
    target_torque = 0.0;
    draw_display(mode, true);

  }
 else if (M5.BtnC.wasPressed()) {
    if (mode == MODE_POSITION) {
      target_pos1 += INC_POSITION / 180.0f * M_PI;

    } else if (mode == MODE_SPEED) {
      target_vel += INC_VELOCITY;

    } else if (mode == MODE_CURRENT) {
      target_torque += INC_TORQUE;
    }
    draw_display(mode);

  } else if (M5.BtnA.wasPressed()) {
    if (mode == MODE_POSITION) {
      target_pos1 -= INC_POSITION / 180.0f * M_PI;

    } else if (mode == MODE_SPEED) {
      target_vel -= INC_VELOCITY;

    } else if (mode == MODE_CURRENT) {
      target_torque -= INC_TORQUE;
    }
    draw_display(mode);
  }

  if (driver1.get_run_mode() == MODE_POSITION) {

    // set limit speed when state changed
    if (state_change_flag1) {
      driver1.set_limit_speed(slow_speed);
      state_change_flag1 = false;
    }
    if (std::fabs(motor_status1.position - target_pos1) < 10.0 / 180.0 * M_PI) {
      driver1.set_limit_speed(slow_speed);
    }//driver1.set_position_ref;

    // set limit speed when state changed
    if (state_change_flag2) {
      driver2.set_limit_speed(slow_speed);
      state_change_flag2 = false;
    }
    if (std::fabs(motor_status2.position - target_pos2) < 10.0 / 180.0 * M_PI) {
      driver2.set_limit_speed(slow_speed);
    }//driver.set_position_ref;

   

    if (std::fabs(motor_status2.effort) > free_rotation_sensitivity) {
      float acceleration = (motor_status2.effort - B * motor_status2.velocity - K * motor_status2.position - G) / M;
      v = acceleration / 25;  // Δt = 0.01s（サンプリング周期）
      target_pos2 -= v / 1;
      
    }
    // if (motor_status2.effort > free_rotation_sensitivity) {
    //   float acceleration = (motor_status2.effort - B * motor_status2.velocity - K * motor_status2.position ) / M;
    //   v = acceleration / 25;  // Δt = 0.01s（サンプリング周期）
    //   target_pos2 -= v / 1;
      
    // }
    //sprite.println("アドミッタンス");
    driver2.set_position_ref(target_pos2);
    driver1.set_position_ref(target_pos1);
    
  }
  else if (driver1.get_run_mode() == MODE_SPEED) {
    float external_force = motor_status1.effort;
    if (std::fabs(external_force) > free_rotation_sensitivity) {
      // 外力が感度閾値を超えた場合、速度を設定
      target_vel = -max_free_rotation_speed * (external_force > 0 ? 1.0f : -1.0f);
    } else {
        // 外力が小さい場合は停止
        target_vel = 0.0f;
        }
    driver1.set_speed_ref(target_vel);
  }
  else if (driver1.get_run_mode() == MODE_CURRENT) {
    driver1.set_current_ref(target_torque);
  }
  
  else {
    target_pos1 += dir * 10.0 / 180.0 * M_PI;
    if (target_pos1 > P_MAX) { dir = -1.0; target_pos1 = P_MAX; }
    else if (target_pos1 < P_MIN) { dir = 1.0; target_pos1 = P_MIN; }

    target_pos2 += dir * 10.0 / 180.0 * M_PI;
    if (target_pos2 > P_MAX) { dir = -1.0; target_pos2 = P_MAX; }
    else if (target_pos2 < P_MIN) { dir = 1.0; target_pos2 = P_MIN; }

    driver1.motor_control(target_pos1, dir * target_vel, dir * target_torque, default_kd, default_kd);
    driver2.motor_control(target_pos2, dir * target_vel, dir * target_torque, default_kd, default_kd);
  }

  // update and get motor data
  if ( driver1.process_packet()) {
    motor_status1 = driver1.get_motor_status();
    draw_display(mode);
  }
  if ( driver2.process_packet()) {
    motor_status2 = driver2.get_motor_status();
    draw_display(mode);
  }

  delay(200);
}
