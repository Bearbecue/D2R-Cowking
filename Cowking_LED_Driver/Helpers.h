//----------------------------------------------------------------------------
//  Various helper functions

template<typename _T, typename _TA, typename _TB>
static inline _T clamp(_T v, _TA vMin, _TB vMax) { return v < vMin ? vMin : v > vMax ? vMax : v; }

template<typename _T>
static inline _T sign(_T v) { return v < 0 ? -1 : 1; }

//----------------------------------------------------------------------------

#define STR_STARTS_WITH(__str, __literal) (!strncmp(__str, __literal, sizeof(__literal)-1))

//----------------------------------------------------------------------------

class LoopTimer
{
public:
  void          tick();

  unsigned long dtMS() const { return m_DtUS * 1000; }
  unsigned long dtUS() const { return m_DtUS; }
  float         dt() const { return m_DtUS * 1.0e-6f; }
  double        elapsedTime() const { return m_ElapsedTime; }

private:
  unsigned long m_DtUS = 0;
  unsigned long m_PrevUS = -1;
  double        m_ElapsedTime = 0;
};

//----------------------------------------------------------------------------

extern float  Noise(float t);
extern float  NoisePeriod();
static float  smoothstep(float t) { return t * t * (3.0f - 2.0f * t); }

struct CRGB;
extern CRGB   ShiftHS(const CRGB &color, int32_t hue_shift, int32_t sat_shift);

//----------------------------------------------------------------------------
