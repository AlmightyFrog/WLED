#pragma once

// Known issues:
//
// - Hue calculation is not precise, but good enough
// - Debug printout of e.g. effectCurrent, effectSpeed and effectCurrent might lag one value behind
//

#include "wled.h"
#include <PinButton.h>

// Per default do not print anything
#define USER_DEBUG_PRINT(...)
#define USER_DEBUG_PRINTF(...)
#define USER_DEBUG_PRINTLN(...)

#ifdef USERMOD_ROTARY_ENCODER_TREE_DEBUG
#if USERMOD_ROTARY_ENCODER_TREE_DEBUG == 1
#undef USER_DEBUG_PRINT
#undef USER_DEBUG_PRINTF
#undef USER_DEBUG_PRINTLN
#define USER_DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define USER_DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define USER_DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#endif
#endif

class SimpleRotaryEncoder
{
private:
  const int pinA;
  const int pinB;
  bool Enc_A_prev;
  int direction;
  const bool debug;

public:
  SimpleRotaryEncoder(int pinA, int pinB, bool debug = false)
      : pinA(pinA), pinB(pinB), direction(0), debug(debug)
  {
    // Initialise pins and state
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    Enc_A_prev = digitalRead(pinA);
  }

  int update()
  {
    bool Enc_A = digitalRead(pinA); // Read encoder pin A
    bool Enc_B = digitalRead(pinB); // Read encoder pin B

    direction = 0; // Reset direction

    // Detect rising edge
    if ((Enc_A) && (!Enc_A_prev))
    {
      if (Enc_B == LOW) // changes to LOW so that then encoder registers a change at the very end of a pulse
      {                 // B is high so clockwise
        direction = 1;  // Clockwise
      }
      else if (Enc_B == HIGH)
      {                 // B is low so counter-clockwise
        direction = -1; // Counter-clockwise
      }

      if (debug)
      {
        USER_DEBUG_PRINTF("Pin (%d, %d) = (%d, %d); Dir = %d\r\n", pinA, pinB, Enc_A, Enc_B, direction);
      }
    }

    Enc_A_prev = Enc_A; // Store value of A for next time

    return direction;
  }
};

class RotaryEncoderTree : public Usermod
{
private:
  SimpleRotaryEncoder *simpleEncoder;

  PinButton *btE = NULL;
  PinButton *btL = NULL;
  PinButton *btR = NULL;

  uint8_t _changeHue(uint8_t *ptr, int8_t value)
  {
    CRGB col_rgb(ptr[0], ptr[1], ptr[2]);
    CHSV col_hsv = rgb2hsv_approximate(col_rgb);

    // Calculate new hue value
    int16_t newH = (int16_t)col_hsv.h; // convert uint to int so we can do cheap overflow correction
    newH += value;                     // add offset
    newH = (newH + 256) % 256;         // make sure it's in the range 0-255

    // Update color
    col_hsv.h = (uint8_t)newH;
    col_rgb = CRGB(col_hsv);
    ptr[0] = col_rgb.r;
    ptr[1] = col_rgb.g;
    ptr[2] = col_rgb.b;

    return col_hsv.h;
  }

  void changeHue(int8_t value)
  {
    uint8_t hue = _changeHue(&col[0], value);
    uint8_t hueSec = _changeHue(&colSec[0], value);
    USER_DEBUG_PRINTF("Hue = %d, HueSec = %d, change = %d\r\n", hue, hueSec, value);

    colorUpdated(CALL_MODE_BUTTON);
    updateInterfaces(CALL_MODE_BUTTON);
  }

public:
  void setup()
  {
    btE = new PinButton(USERMOD_ROTARY_ENCODER_TREE_ENCODER_BUTTON);
    btR = new PinButton(USERMOD_ROTARY_ENCODER_TREE_BUTTON_RIGHT);
    btL = new PinButton(USERMOD_ROTARY_ENCODER_TREE_BUTTON_LEFT);
    simpleEncoder = new SimpleRotaryEncoder(USERMOD_ROTARY_ENCODER_TREE_ENCODER_A, USERMOD_ROTARY_ENCODER_TREE_ENCODER_B);
  }

  void loop()
  {
    // Handle rotary encoder
    int dir = simpleEncoder->update(); // Get the direction
    if (dir == 1)
    {
      changeHue(10);
    }
    else if (dir == -1)
    {
      changeHue(-10);
    }

    // Handle buttons
    btE->update();
    btR->update();
    btL->update();

    // Button Encoder, single click -> load preset 1
    if (btE->isSingleClick())
    {
      applyPreset(1, CALL_MODE_DIRECT_CHANGE);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("bE->click effectCurrent = %d\r\n", effectCurrent);
    }

    // Button Encoder, double click -> random effect
    if (btE->isDoubleClick())
    {
      effectCurrent = random(MODE_COUNT);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("btE->double effectCurrent = %d\r\n", effectCurrent);
    }

    // Button Encoder, long click -> toggle on/off
    if (btE->isLongClick())
    {
      USER_DEBUG_PRINTLN("bE->long");
      toggleOnOff();
      colorUpdated(CALL_MODE_BUTTON);
    }

    // Button Right, double click -> faster
    if (btR->isDoubleClick())
    {
      effectSpeed = min((int)effectSpeed + 10, 255);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("bR->double effectSpeed = %d\r\n", effectSpeed);
    }

    // Button Left, double click -> slower
    if (btL->isDoubleClick())
    {
      effectSpeed = max((int)effectSpeed - 10, 0);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("bL->double effectSpeed = %d\r\n", effectSpeed);
    }

    // Button Right, single click -> next preset (1-10)
    if (btR->isSingleClick())
    {
      uint8_t targetPreset;
      if (currentPreset == 10)
      {
        targetPreset = 1;
      }
      else
      {
        targetPreset = currentPreset + 1;
      }

      if (applyPreset(targetPreset, CALL_MODE_DIRECT_CHANGE) == false)
      {
        applyPreset(10, CALL_MODE_DIRECT_CHANGE);
      }

      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("bR->click preset = %d\r\n", currentPreset);
    }

    // Button Left, single click -> previous preset (1-10)
    if (btL->isSingleClick())
    {
      uint8_t targetPreset;
      if (currentPreset == 1)
      {
        targetPreset = 10;
      }
      else
      {
        targetPreset = currentPreset - 1;
      }

      if (applyPreset(targetPreset, CALL_MODE_DIRECT_CHANGE) == false)
      {
        applyPreset(1, CALL_MODE_DIRECT_CHANGE);
      }

      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("bL->click preset = %d\r\n", currentPreset);
    }

    // Button Left, long click -> load preset 2
    if (btL->isLongClick())
    {
      applyPreset(2, CALL_MODE_DIRECT_CHANGE);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("btL->long currentPreset = %d\r\n", currentPreset);
    }

    // Button Right, long click -> load preset 3
    if (btR->isLongClick())
    {
      applyPreset(3, CALL_MODE_DIRECT_CHANGE);
      colorUpdated(CALL_MODE_FX_CHANGED);
      USER_DEBUG_PRINTF("btR->long currentPreset = %d\r\n", currentPreset);
    }
  }

  uint16_t getId()
  {
    return USERMOD_ID_ROTARY_ENCODER_TREE;
  }
};
