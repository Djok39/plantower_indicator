// MQ* gas sensor driver. Suitable for MQ7, MQ9 and other MQ*
// Authors: Djok_39
// License: MIT
#include "mgos.h"
#include "mgos_adc.h"
#include "driver/adc.h"
#include "math.h"

#define EVENT_GRP_SM MGOS_EVENT_BASE('S', 'M', '_')
#define SM_METHANE_AVAILABLE (EVENT_GRP_SM + 1)
#define SM_CO_AVAILABLE (EVENT_GRP_SM + 2)
#define SM_MQ_DISABLED (EVENT_GRP_SM + 6)
#define MQ_SAMPLES 64
#define HW_TICK_USEC 500
#define MEASURE_TICK_MS (20)
#define LOW_DUTY_TICKS (90000000 / HW_TICK_USEC)
#define HIGH_DUTY_TICKS (60000000 / HW_TICK_USEC)
#define K (MEASURE_TICK_MS*1000 / HW_TICK_USEC)
#define MEASURE_TICKS_WINDOW (K*MQ_SAMPLES + K*10) // samples*1_tick + 100msec
#define LOW_BEGIN_MEASURE_TICK (LOW_DUTY_TICKS - MEASURE_TICKS_WINDOW)
#define HIGH_BEGIN_MEASURE_TICK (HIGH_DUTY_TICKS - MEASURE_TICKS_WINDOW)
#define ADC2VALUE (1000.0/4095.0) // i want to see range from 0...1000 on display, but instead 1000.0 i want to see something like "OVR" as overshoot indication

struct MQ_STATE{
  bool      initialised;
  double    methane;
  double    co;
  bool      methane_valid;  // deprecated
  bool      co_valid;       // deprecated
};

struct MQ_STATE mq;

static int measureTimerId = 0;
static int hwTimer = 0;
static int timerTick = 0;
static bool doMeasure = false;
static bool dualMode = true;  // Measure CO and Methane as described in MQ9 datasheet
static int samplesTop = 0;
static uint16_t samples[MQ_SAMPLES];
static double previousValue = -1000.0; // mark as ivalid
static char* previousName = "";
static int power = 0;
static int sense = 0;

struct PWM_Mode{
  uint16_t mult;
  uint16_t div;
  char*   name;
};

int currentJob = 0;
int measuringJob = -1;
struct PWM_Mode workCycle[] = {
  {100,100,"methane"}, {14, 50, "co"}
  // {100,100,"gas50"}, {14,50,"gas14"} 
};

#define TOTAL_JOBS (sizeof(workCycle)/sizeof(struct PWM_Mode))
bool mq_toggle(void);

inline bool isLowPower(){
  return workCycle[currentJob].mult != workCycle[currentJob].div;
}
inline int getNextJob(){
  int nextJob = currentJob + 1;
  return (nextJob < TOTAL_JOBS) ? nextJob : 0;
}
// return values:
// empty string - no valid value
// else - name of measured gas: gas50, gas15, gas14, etc
char* mq_read_designation(void){
  return previousName;
}
// pre-measured value
double mq_read_value(void){
  return previousValue;
}

static void MQ9jobLog(void *arg){
  LOG(LL_INFO, ("MQ9 set job to %i: %s (%i/%i)", currentJob, workCycle[currentJob].name, workCycle[currentJob].mult, workCycle[currentJob].div));
}

bool mq_set_job(int value){
  currentJob = value;
  // mgos_invoke_cb(MQ9jobLog, NULL, true);
  return true;
}

static void measure_timer_cb(void *arg) {
  assert(measuringJob>=0);
  bool lowPowerMode = workCycle[measuringJob].mult != workCycle[measuringJob].div;
  assert(doMeasure);
  assert(measureTimerId);
  int  mq9sum;
  bool inWindow = (lowPowerMode && timerTick >= LOW_BEGIN_MEASURE_TICK && timerTick < LOW_DUTY_TICKS) || 
    (!lowPowerMode && timerTick >= HIGH_BEGIN_MEASURE_TICK && timerTick < HIGH_DUTY_TICKS);

  if (inWindow){
    assert(samplesTop < MQ_SAMPLES);
    samples[samplesTop++] = mgos_adc_read(sense);
  }
  if (samplesTop >= MQ_SAMPLES || !inWindow){
    assert(measureTimerId);
    mgos_clear_timer(measureTimerId);
    if (!inWindow){
      LOG(LL_ERROR, ("not fit measuring window %i %i", timerTick, samplesTop));
    }
    doMeasure = false;
    measureTimerId = 0;
    mq9sum = 0;
    int correctSamples = 0;
    for(int i=0; i < samplesTop; i++ ){
      mq9sum += samples[i];
      correctSamples++;
    }
    mq.co_valid = false;
    mq.methane_valid = false;
    if (mq9sum == 0 && !lowPowerMode){
      LOG(LL_ERROR, ("MQ sensor probably disconnected. stopping."));
      correctSamples = 0;
      mq_toggle();
    };
    if (correctSamples > 16){
      double gas = (double)mq9sum / (double)correctSamples * ADC2VALUE * 100.0;
      gas = round(gas) / 100.0;
      if (lowPowerMode){
        mq.co = gas;
        mq.co_valid = true;
        LOG(LL_INFO, ("Measured MQ LO%i level: %f, samples: %i", measuringJob >> 1, mq.co, correctSamples));
      }else{
        mq.methane = gas;
        mq.methane_valid = true;
        LOG(LL_INFO, ("Measured MQ HI%i level: %f, samples: %i", measuringJob >> 1, mq.methane, correctSamples));
      }
      previousValue = gas;
      previousName = workCycle[measuringJob].name;
      mgos_event_trigger(lowPowerMode ? SM_CO_AVAILABLE : SM_METHANE_AVAILABLE, (void*)NULL);
    }else{
      previousValue = -1.0; // mark as invalid
      previousName = "";
    }
  }
  (void) arg;
}

static void begin_measure_cb(void *arg){
  assert(doMeasure);
  assert(measureTimerId == 0);
  // clear buffer
  samplesTop = 0;
  for(int i=0; i < MQ_SAMPLES; i++ )
    samples[i] = 0;
  // LOG(LL_INFO, ("Begin MQ9 measurement, lowPower=%i", lowPowerMode));
  measureTimerId = mgos_set_timer(MEASURE_TICK_MS, MGOS_TIMER_RUN_NOW | MGOS_TIMER_REPEAT, measure_timer_cb, NULL);
}

static void pwm_hw_timer_cb(void *arg) {
  timerTick++;
  bool lowPowerMode = isLowPower();
  int rest = lowPowerMode ? timerTick % workCycle[currentJob].div : 0; 
  if (lowPowerMode){
    /*if (doMeasure){
      // do nothing
    }else */
    if (rest == workCycle[currentJob].mult){
      mgos_gpio_write(power, 0); // disable each 3th tick (30ms) - we will get 1.5V from 5V 
    }else if(rest==0){
      mgos_gpio_write(power, 1); // enable each 10th tick (100ms)
    }
    if (timerTick >= LOW_DUTY_TICKS){
      // end work cycle
      if (dualMode){
        mq_set_job(getNextJob());
      }
      timerTick = 0;
      rest = 0;
    }else if (timerTick == LOW_BEGIN_MEASURE_TICK){
      doMeasure = true;
      measuringJob = currentJob;
      // mgos_gpio_write(power, 0); // no heat while measuring
      mgos_invoke_cb(begin_measure_cb, (void *)0, true);
    }
  }else{// full power mode
    if (!doMeasure && !mgos_gpio_read_out(power)){
      mgos_gpio_write(power, 1);
    };
    if (timerTick >= HIGH_DUTY_TICKS){
      // end work cycle
      if (dualMode){
        mq_set_job(getNextJob());
      }
      timerTick = 0;
      rest = 0;
    }else if (timerTick == HIGH_BEGIN_MEASURE_TICK){
      doMeasure = true;
      measuringJob = currentJob;
      // mgos_gpio_write(power, 0); // no heat while measuring
      mgos_invoke_cb(begin_measure_cb, (void *)0, true);
    }
  }
  (void) arg;
}

bool mq_init(int powerPin, int sensePin){
  if (mq.initialised)
    return true;

  // run it once
  if (!sense && !power){
    power = powerPin;
    sense = sensePin;
    mgos_adc_enable(sense);
    esp32_set_channel_attenuation(sense, ADC_ATTEN_DB_0);
    // load low power duty cycle from config
    if (TOTAL_JOBS >= 2 && strcmp(workCycle[1].name, "co")==0){
      workCycle[1].mult = mgos_sys_config_get_mq_mult();
      workCycle[1].div = mgos_sys_config_get_mq_div();
    }
  };
  
  mq.initialised = true;
  mq.methane_valid = false;
  mq.co_valid = false;

  timerTick = 0;
  samplesTop = 0;

  mgos_gpio_set_mode(power, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_write(power, 0);  // off
  mq_set_job(0);

  hwTimer = mgos_set_hw_timer(HW_TICK_USEC, MGOS_TIMER_RUN_NOW | MGOS_TIMER_REPEAT, pwm_hw_timer_cb, NULL /* arg */);
  LOG(LL_INFO, ("MQ timer starts, window=%i, K=%i, jobs=%i", MEASURE_TICKS_WINDOW, K, TOTAL_JOBS));
  return true;
}

// Toggle enabled/disabled state.
bool mq_toggle(void){
  if (mq.initialised && hwTimer){
    mgos_clear_timer(hwTimer);
    hwTimer = 0;
    LOG(LL_INFO, ("MQ timer stopped"));
    mq.initialised = false;
    mq.methane_valid = false;
    mq.co_valid = false;
    mgos_gpio_write(power, 0);  // off
    mgos_event_trigger(SM_MQ_DISABLED, (void*)NULL);
    return true;
  }else if (power && sense){
    return mq_init(power, sense);
  };
  return false;
};