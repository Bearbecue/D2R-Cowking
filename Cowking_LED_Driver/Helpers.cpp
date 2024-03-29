//----------------------------------------------------------------------------

#include <Arduino.h>
#include "Helpers.h"
#include <FastLED.h>  // Used for CRGB helpers

//----------------------------------------------------------------------------

void  LoopTimer::tick()
{
  const unsigned long  curUS = micros();
  if (m_PrevUS < 0)
    m_PrevUS = curUS;

  // Assume we'll never do more than an entire wrap in a single tick
  // this would happen if the time between two ticks gets longer than 1 hour 11 minutes
  // Safe to assume it won't...
  m_DtUS = curUS - m_PrevUS;
  if (curUS > m_PrevUS) // micros timer wrapper around.
    m_DtUS -= 1;  // handle wrapping around: (((unsigned long)-1) - m_PrevUS) + curUS)

  m_PrevUS = curUS;

  m_ElapsedTime += m_DtUS * 1.0e-6;
}

//----------------------------------------------------------------------------

CRGB  ShiftHS(const CRGB &color, int32_t hue_shift, int32_t sat_shift)
{
  if (hue_shift == 0 && sat_shift == 0) // Nothing to shift
    return color;

  // Convert from RGB to HSV
  int32_t       h = 0;
  int32_t       s = 0;
  const int32_t v = max(max(color.r, color.g), color.b);
  const int32_t delta = v - min(min(color.r, color.g), color.b);
  if (delta > 0)
  {
    const float invDelta = 1.0f / delta;
    float   hvalue;
    if (v == color.r)
      hvalue = (color.g - color.b) * invDelta + 0.0f;
    else if (v == color.g)
      hvalue = (color.b - color.r) * invDelta + 2.0f;
    else
      hvalue = (color.r - color.g) * invDelta + 4.0f;
    if (hvalue < 0.0f)
      hvalue += 6.0f;

    h = clamp(int32_t(hvalue * 255.0f / 6.0f), 0, 255);
    s = clamp(int32_t(delta * 255.0f / v), 0, 255);
  }

  // Shift the hue & sat
  h = (h + hue_shift) & 0xFF;         // wrap hue
  s = clamp(s + sat_shift, 0, 0xFF);  // clamp sat

  // Convert back from HSV to RGB
  const float delta2 = (s * v) / 255.0f;
  const float h6 = h * 6.0f / 255.0f;
  const float r0n = 2.0f - fabsf(h6 - 3.0f);
  const float g0n = fabsf(h6 - 2.0f) - 1.0f;
  const float b0n = fabsf(h6 - 4.0f) - 1.0f;
  return CRGB(v - int32_t(delta2 * clamp(r0n, 0.0f, 1.0f)),
              v - int32_t(delta2 * clamp(g0n, 0.0f, 1.0f)),
              v - int32_t(delta2 * clamp(b0n, 0.0f, 1.0f)));
}

//----------------------------------------------------------------------------
// Returns a -1, 1 noise

float  Noise(float t)
{
  return sinf(t *  1.000f) * 0.400f +
         sinf(t *  3.200f) * 0.325f +
         sinf(t *  7.200f) * 0.200f +
         sinf(t * 17.800f) * 0.075f;
}

//----------------------------------------------------------------------------

float  NoisePeriod()
{
  // Time weights = 1.0, 3.2, 7.2, 17.8
  // Common period = 12816
  // 12816 * (2.pi) = 80525.303f
  return 80525.303f;  // Noise repeats itself every ~22h and 22 minutes if 't' is in seconds
}

//----------------------------------------------------------------------------
