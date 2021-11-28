#pragma once

#include "wled.h"
#include <PinButton.h>

#if 0
template <uint8_t SIZE, uint8_t DEBOUNCE_MS = 50, uint16_t LONGPRESS_MS = 600, uint16_t HOLD_MS = 300>
class ButtonUl {
  public:
    enum state_t : uint8_t {
      NONE = 0,
      SINGLE = 1,
      DOUBLE = 2,
      HOLD = 3
    };

  private:
    typedef struct {
      uint8_t pin;
      unsigned long start;
      state_t state;
      uint8_t active;
    } data_t;

    data_t data[SIZE];

  public:
    ButtonUl()
    {
      for (uint8_t i = 0; i < SIZE; i++)
      {
        data[i].state = NONE;
      }
    }

    void setPin(uint8_t button, uint8_t pin, uint8_t mode, uint8_t active = LOW) {
      if (button < SIZE) {
        data[button].pin = pin;
        data[button].start = 0;
        data[button].state = NONE;
        data[button].active = active;
        pinMode(pin, mode);
      }
    }

    void handle()
    {
      unsigned long now = millis();
      for (uint8_t i = 0; i < SIZE; i++)
      {
        data_t* da = &data[i];
        boolean active = digitalRead(da->pin) == da->active;

        if ((da->start == 0) && (active))
        {
          // Not yet pressed before, get start time
          da->start = now;
        } else {
          // Pressed before, get difference to start
          unsigned long diff = now - da->start;

          if (active) {
            // Still pressed, check if longpress
            if (diff > LONGPRESS_MS) {
              da->state = HOLD;
            } 
          } else {
            // Released, check time
            if (diff > LONGPRESS_MS) {
              // Was longpress action, just clear record
              da->state = NONE;
            }
            
          }

        }

      }
    }

};
#endif

class UsermodRotaryEncoderTree : public Usermod {
  private:

    typedef struct {
      int8_t encA;
      int8_t encB;
      int8_t bE;
      int8_t bL;
      int8_t bR;
    } usermod_config_t;


    usermod_config_t config;
    boolean validConfig = false;
    boolean validConfigInit = false;

    boolean debug = false;

    PinButton* btE = NULL;
    PinButton* btL = NULL;
    PinButton* btR = NULL;

    void _changeHue(uint8_t* ptr, int8_t value)
    {
      CRGB col_rgb(ptr[0],ptr[1],ptr[2]);
      CHSV col_hsv = rgb2hsv_approximate(col_rgb);
      int16_t newH = (int16_t)col_hsv.h;    // convert uint to int so we can do cheap overflow correction

      // Add offset
      newH += value;

      // correct the overflow in both directions if neccessary to get into H range again
      if (newH > 255) newH -= 255;
      if (newH < 0) newH += 255;

      // Set H
      col_hsv.h = (uint8_t)newH;

      col_rgb = CRGB(col_hsv);
      ptr[0] = col_rgb.r;
      ptr[1] = col_rgb.g;
      ptr[2] = col_rgb.b;
    }

    void changeHue(int8_t value) {
      if (debug) Serial.print("HueChg ");
      if (debug) Serial.println(value);

      _changeHue(&col[0], value);
      _changeHue(&colSec[0], value);

      colorUpdated(CALL_MODE_BUTTON);
      updateInterfaces(CALL_MODE_BUTTON);
    }

  public:

    static uint8_t encoderPin;
    static volatile int8_t encoderChange;

    static void IRAM_ATTR pci() {
      static unsigned long lastTime = 0;
      unsigned long currentTime = millis();
      if (currentTime >= (lastTime+2))
      {
        lastTime = currentTime;

        uint8_t encB = digitalRead(encoderPin);
        if (encB == HIGH) {
          // Increase H
          encoderChange += 10;
        } else {
          // Decrease H
          encoderChange -= 10;
        }
      }
    }

    void setup() {
      if (debug) Serial.println("RET: setup()");
      if (validConfig)
      {
        if (debug) Serial.println("RET: valid config init");

        btE = new PinButton(config.bE);
        btR = new PinButton(config.bR);
        btL = new PinButton(config.bL);

        pinMode(config.encA, INPUT_PULLUP);
        pinMode(config.encB, INPUT_PULLUP);

        UsermodRotaryEncoderTree::encoderPin = config.encB;
        UsermodRotaryEncoderTree::encoderChange = 0;
        attachInterrupt(digitalPinToInterrupt(config.encA), pci, RISING);

        validConfigInit = true;
      }
      if (debug) Serial.println("RET: setup() finished");
    }


    void connected() {
    }


    void loop() {
#if 1
      if (encoderChange != 0)
      {
        noInterrupts();
        int8_t cur = encoderChange;
        encoderChange = 0;
        interrupts();

        Serial.println(cur);
        changeHue(cur);
      }

#else

      static uint8_t lastEncA = 0;
      static unsigned long lastTime = 0;
      unsigned long currentTime = millis();

      // First handle rotary encoder as fast as possible
      if (validConfigInit) {
        if (currentTime >= (lastTime+2))
        {
          lastTime = millis();

          uint8_t encA = digitalRead(config.encA);
          uint8_t encB = digitalRead(config.encB);

          if ((!encA) && (lastEncA)) {
            if (encB == HIGH) {
              // Increase H
              changeHue(10);
            } else {
              // Decrease H
              changeHue(-10);
            }
          }
          lastEncA = encA;
      
        }
      }
#endif


      if (validConfigInit)
      {
        btE->update();
        btR->update();
        btL->update();

        if (btE->isSingleClick()) {
            if (debug) Serial.println("bE clicked");
            // load preset 1
            applyPreset(1);
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btE->isDoubleClick()) {
            Serial.println("btE->double");
            // random effect
            effectCurrent = random(MODE_COUNT);
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btE->isLongClick()) {
            if (debug) Serial.println("bE holding");
            // toggle on off
            toggleOnOff();
            colorUpdated(CALL_MODE_BUTTON);
        }

        if (btR->isDoubleClick()) {
            if (debug) Serial.println("bR double");
            // faster
            if (effectSpeed < 245) {
              effectSpeed += 10;
            } else if (effectSpeed < 255) {
              effectSpeed += 1;
            }
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btL->isDoubleClick()) {
            if (debug) Serial.println("bL double");
            // slower
            if (effectSpeed > 10) {
              effectSpeed -= 10;
            } else if (effectSpeed > 0) {
              effectSpeed -= 1;
            }
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btR->isSingleClick()) {
            if (debug) Serial.println("bR clicked");
            // next preset
            if (currentPreset < 10) {
              if (applyPreset(currentPreset+1) == false) {
                // Fallback to preset 1
                applyPreset(1);
              }
            } else {
              applyPreset(1);
            }
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btL->isSingleClick()) {
            if (debug) Serial.println("bL clicked");
            // previous preset
            if (currentPreset > 1) {
              if (applyPreset(currentPreset-1) == false) {
                // Fallback to preset 1
                applyPreset(1);
              }
            } else {
              applyPreset(10);
            }
            colorUpdated(CALL_MODE_FX_CHANGED);
        }

        if (btL->isLongClick()) {
          if (debug) Serial.println("btL->long");
          // load preset 2
          applyPreset(2);
          colorUpdated(CALL_MODE_FX_CHANGED);
        }
    
        if (btR->isLongClick()) {
          if (debug) Serial.println("btR->long");
          // load preset 3
          applyPreset(3);
          colorUpdated(CALL_MODE_FX_CHANGED);
        }


      }


    }


    void addToJsonState(JsonObject& root)
    {
    }


    void readFromJsonState(JsonObject& root)
    {
    }


    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject("usermodRotaryEncoderTree");
      top["encA"] = config.encA;
      top["encB"] = config.encB;
      top["bE"] = config.bE;
      top["bL"] = config.bL;
      top["bR"] = config.bR;
    }


    bool readFromConfig(JsonObject& root)
    {
      JsonObject top = root["usermodRotaryEncoderTree"];

      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top["encA"],  config.encA, -1);
      configComplete &= getJsonValue(top["encB"],  config.encB, -1);
      configComplete &= getJsonValue(top["bE"],  config.bE, -1);
      configComplete &= getJsonValue(top["bL"],  config.bL, -1);
      configComplete &= getJsonValue(top["bR"],  config.bR, -1);

      validConfig = configComplete;

      if (debug) Serial.println("RET: readFromConfig()");

      return configComplete;
    }

   
    uint16_t getId()
    {
      return USERMOD_ID_ROTARY_ENCODER_TREE;
    }

};

uint8_t UsermodRotaryEncoderTree::encoderPin;
volatile int8_t UsermodRotaryEncoderTree::encoderChange;
