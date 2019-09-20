#include <stdio.h>
#include "mgos.h"
#include "math.h"
#include "ssd1306.h"
#include "main.h"
#include "esp_wifi.h"

typedef struct {
  uint16_t env_pm10, env_pm25, env_pm100;
} PMS_DATA;

typedef struct {
    uint16_t frameLength;
    uint16_t standartPM10, standartPM25, standartPM100;
    uint8_t env_pm10HI, env_pm10LO;
    uint8_t env_pm25HI, env_pm25LO;
    uint8_t env_pm100HI, env_pm100LO;
    uint8_t particles03umHI, particles03umLO;
    uint8_t particles05umHI, particles05umLO;
    uint8_t particles10umHI, particles10umLO;
    uint8_t particles25umHI, particles25umLO;
    uint8_t particles50umHI, particles50umLO;
    uint8_t particles100umHI, particles100umLO;
    uint16_t unused;
    uint8_t checksum_hi;
    uint8_t checksum_lo;
} plantowerData_t;
#define PMS_PACKET_SIZE (sizeof(plantowerData_t)+2)

#define PMS_MA_BUFFER_SIZE 3857 // enough for 1 hour averaging
#define EVENT_GRP_SM MGOS_EVENT_BASE('S', 'M', '_')
#define PMS_EVENT (EVENT_GRP_SM + 5)
#define SM_METHANE_AVAILABLE (EVENT_GRP_SM + 1)
#define SM_CO_AVAILABLE (EVENT_GRP_SM + 2)

#define UART_NO2 2
static char szState[16] = "";
static bool initialised = false;
static struct mgos_ssd1306 * oled = NULL;
static double fLastUpdate = 0.0;
uint8_t y_offset = 0;
uint8_t x_offset = 0;
static int top = 0;
static PMS_DATA ma_buffer[PMS_MA_BUFFER_SIZE];
static double co_value = 0.0;
static double methane_value = 0.0;
uint8_t response[PMS_PACKET_SIZE];

// points - number of last points to average
double pm1_0(int points){
  if (points > PMS_MA_BUFFER_SIZE){
    LOG(LL_ERROR, ("too many ma points"));
    return -1.0;
  };

  double sum = 0.0;
  int count=0;
  for(int i=top; count < points; count++, i-- ){
    if (i < 0){
      i = PMS_MA_BUFFER_SIZE-1;
    }
    int val = ma_buffer[i].env_pm10;
    if (val == 0xFFFF)
      break;
    else
      sum += val;
  };
  return count ? (sum / (double)count) : -1.0;
};

double pm2_5(int points){
  if (points > PMS_MA_BUFFER_SIZE){
    LOG(LL_ERROR, ("too many ma points"));
    return -1.0;
  };

  double sum = 0.0;
  int count=0;
  for(int i=top; count < points; count++, i-- ){
    if (i < 0){
      i = PMS_MA_BUFFER_SIZE-1;
    }
    int val = ma_buffer[i].env_pm25;
    if (val == 0xFFFF)
      break;
    else
      sum += val;
  };
  return count ? (sum / (double)count) : -1.0;
};

double pm10(int points){
  if (points > PMS_MA_BUFFER_SIZE){
    LOG(LL_ERROR, ("too many ma points"));
    return -1.0;
  };

  double sum = 0.0;
  int count=0;
  for(int i=top; count < points; count++, i-- ){
    if (i < 0){
      i = PMS_MA_BUFFER_SIZE-1;
    }
    int val = ma_buffer[i].env_pm100;
    if (val == 0xFFFF)
      break;
    else
      sum += val;
  };
  return count ? (sum / (double)count) : -1.0;
};

static void uart_dispatcher(int uart_no, void *arg) {
  size_t rx_av;
  size_t cntUart;
  assert(uart_no == UART_NO2);
  assert(initialised);
  while ((rx_av = mgos_uart_read_avail(uart_no)) >= PMS_PACKET_SIZE)
  {
    // read first byte
    cntUart=mgos_uart_read(uart_no, &response, 1);
    // check header
    if (cntUart != 1 || response[0] != 0x42){
      // LOG(LL_WARN, ("PMS  UART%d, avail %d", UART_NO2, mgos_uart_read_avail(uart_no)));
      continue;
    }
    // read next bytes
    cntUart = mgos_uart_read(uart_no, &response[1], sizeof(plantowerData_t)+1);
    if (cntUart != (sizeof(plantowerData_t)+1) || response[1] != 0x4D){
      LOG(LL_ERROR, ("PMS**03: bad packet"));
      continue;
    }
    plantowerData_t* p = (plantowerData_t*)&response[2];
    int receiveSum=0;
    for(int i=0; i < sizeof(plantowerData_t); i++){
      receiveSum += response[i];
    }
    uint16_t packetChecksum =  ( p->checksum_hi << 8) + p->checksum_lo;
    if(receiveSum != packetChecksum)  //check the serial data 
    {
      LOG(LL_ERROR, ("PMS**03: bad checksum: %04X != %04X", receiveSum, packetChecksum));
      continue;
    }
    uint16_t pm10 = p->env_pm10LO + (p->env_pm10HI << 8);
    uint16_t pm25 = p->env_pm25LO + (p->env_pm25HI << 8);
    uint16_t pm100 = p->env_pm100LO + (p->env_pm100HI << 8);
    // LOG(LL_INFO, ("PMS**03: pm2.5: %i %.2f %.2f %.2f %.2f %.2f %.2f", pm25, pm2_5(5), pm2_5(15), pm2_5(30), pm2_5(60), pm2_5(150), pm2_5(300)));

    // let some warmup time
    if (mgos_uptime() >= 30.0){
      top++;
      if (top >= PMS_MA_BUFFER_SIZE){
        top=0;
      };
      ma_buffer[top].env_pm10 = pm10;
      ma_buffer[top].env_pm25 = pm25;
      ma_buffer[top].env_pm100 = pm100;
    };
    mgos_event_trigger(PMS_EVENT, (void*)NULL);
  }
  (void) arg;
}

static void refresh_dispay();
static void delayed_refresh_timer_cb(void *arg) {
  refresh_dispay();
  (void) arg;
};

static void screensaver_timer_cb(void *arg) {
  x_offset += 1;
  if (x_offset > 6){
    x_offset = 0;
    y_offset += 1;
    if (y_offset > 2){
      y_offset = 0;
    };
  };
};

static void refresh_dispay()
{
  assert(oled);
  double fNowUptime = mgos_uptime();
  // Restrict screen update to every 33ms
  if ((fNowUptime - fLastUpdate) < 0.03333333333333333333333333333333){
    mgos_set_timer(10 /* msec */, false /* repeat */, delayed_refresh_timer_cb, NULL /* arg */);
    return;
  };

  fLastUpdate = fNowUptime;
  mgos_ssd1306_clear(oled);
  mgos_ssd1306_select_font(oled, 1);
  char str[60] = {'\0'};  // TODO: calcuate more precise needed buffer size
  // uint8_t width = 128;
  uint8_t step = 104 / 4;

  if (initialised){
    plantowerData_t* p = (plantowerData_t*)&response[2];
    uint16_t _pm10 = p->env_pm10LO + (p->env_pm10HI << 8);
    uint16_t _pm25 = p->env_pm25LO + (p->env_pm25HI << 8);
    uint16_t _pm100 = p->env_pm100LO + (p->env_pm100HI << 8);

    uint16_t _03um = p->particles03umLO + (p->particles03umHI << 8);
    uint16_t _05um = p->particles05umLO + (p->particles05umHI << 8);
    uint16_t _10um = p->particles10umLO + (p->particles10umHI << 8);
    uint16_t _25um = p->particles25umLO + (p->particles25umHI << 8);
    uint16_t _50um = p->particles50umLO + (p->particles50umHI << 8);
    uint16_t _100um = p->particles100umLO + (p->particles100umHI << 8);

    mgos_ssd1306_draw_string (oled, x_offset, y_offset + 3, szState);
    mgos_ssd1306_draw_string (oled, x_offset + step, y_offset + 3, "PM1");
    mgos_ssd1306_draw_string (oled, x_offset + step*2, y_offset + 3, "2.5");
    mgos_ssd1306_draw_string (oled, x_offset + step*3, y_offset + 3, "10");
    // mgos_ssd1306_draw_string (oled, x_offset + step*4, y_offset + 3, "?");
    if (mq.initialised){
      mgos_ssd1306_draw_string (oled, x_offset + step*4, y_offset + 3, "CO");
    }

    // Show instant PM value.
    sprintf(str, "%i", _pm10); mgos_ssd1306_draw_string (oled, x_offset + step, y_offset + 15, str);
    sprintf(str, "%i", _pm25); mgos_ssd1306_draw_string (oled, x_offset + step*2, y_offset + 15, str);
    sprintf(str, "%i", _pm100); mgos_ssd1306_draw_string (oled, x_offset + step*3, y_offset + 15, str);
    if (co_value == 1000.0){
      mgos_ssd1306_draw_string (oled, x_offset + step*4, y_offset + 15, "OVR");
    }else if (co_value != 0.0){
      sprintf(str, "%.0f", co_value); mgos_ssd1306_draw_string (oled, x_offset + step*4, y_offset + 15, str);
    }
    
    // Show mean average for 5 minutes.
    mgos_ssd1306_draw_string (oled, x_offset, y_offset + 24, "~");
    sprintf(str, "%.1f", pm1_0(321)); mgos_ssd1306_draw_string (oled, x_offset + step, y_offset + 24, str);
    sprintf(str, "%.1f", pm2_5(321)); mgos_ssd1306_draw_string (oled, x_offset + step*2, y_offset + 24, str);
    sprintf(str, "%.1f", pm10(321)); mgos_ssd1306_draw_string (oled, x_offset + step*3, y_offset + 24, str);

    // other particles
    mgos_ssd1306_select_font(oled, 0);
    mgos_ssd1306_draw_string (oled, x_offset + 27, y_offset + 35, "particles, um");
    // mgos_ssd1306_draw_hline (oled, x_offset, y_offset + 42, 128, 1);

    // mgos_ssd1306_draw_string (oled, x_offset, y_offset + 45, "03  05  10  25  50 100");
    mgos_ssd1306_draw_string (oled, x_offset, y_offset + 45, "0.3");
    mgos_ssd1306_draw_string (oled, x_offset + 28, y_offset + 45, "0.5");
    mgos_ssd1306_draw_string (oled, x_offset + 56, y_offset + 45, "1");
    mgos_ssd1306_draw_string (oled, x_offset + 78 - 5, y_offset + 45, "2.5");
    mgos_ssd1306_draw_string (oled, x_offset + 100, y_offset + 45, "5");
    mgos_ssd1306_draw_string (oled, x_offset + 110, y_offset + 45, "10");
    mgos_ssd1306_select_font(oled, 1);  // restore default font

    sprintf(str, "%i", _03um);
    mgos_ssd1306_draw_string (oled, x_offset, y_offset + 53, str);

    sprintf(str, "%i", _05um);
    mgos_ssd1306_draw_string (oled, x_offset + 28, y_offset + 53, str);

    sprintf(str, "%i", _10um);
    mgos_ssd1306_draw_string (oled, x_offset + 56, y_offset + 53, str);

    sprintf(str, "%i", _25um);
    mgos_ssd1306_draw_string (oled, x_offset + 76, y_offset + 53, str);

    sprintf(str, "%i", _50um);
    mgos_ssd1306_draw_string (oled, x_offset + 96, y_offset + 53, str);

    sprintf(str, "%i", _100um);
    mgos_ssd1306_draw_string (oled, x_offset + 116, y_offset + 53, str);
  }
  // show changes
  mgos_ssd1306_refresh(oled, false);
};

static void group_events_cb(int ev, void *evd, void *arg) {
  if (ev == PMS_EVENT){
    refresh_dispay();
  }else if (ev == SM_CO_AVAILABLE){
    co_value = mq_read_value();
    refresh_dispay();
  }else if (ev == SM_METHANE_AVAILABLE){
    methane_value = mq_read_value();
    refresh_dispay();
  }
}

enum mgos_app_init_result mgos_app_init(void) {
  //--- setup OLED display
  oled = mgos_ssd1306_get_global();
  if (oled){
    mgos_ssd1306_fill_circle (oled, 63, 31, 32, 1); // TODO: show logo
    mgos_ssd1306_refresh(oled, false);
    mgos_event_add_group_handler(EVENT_GRP_SM, group_events_cb, NULL);

    mgos_set_timer(1000 /* ms */, MGOS_TIMER_REPEAT /* repeat */, screensaver_timer_cb, NULL /* arg */);
  };
  return MGOS_APP_INIT_SUCCESS;
};

bool pms_init(int uartRx, int uartTx){
  if (initialised)
    return true;

  memset(ma_buffer, 0xFF, sizeof(PMS_DATA) * PMS_MA_BUFFER_SIZE);

  //--- setup PMS7003 communication via UART
  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(UART_NO2, &ucfg);
  ucfg.dev.rx_gpio = uartRx;
  ucfg.dev.tx_gpio = uartTx;
  ucfg.baud_rate = 9600;
  ucfg.num_data_bits = 8;
  ucfg.parity = MGOS_UART_PARITY_NONE;
  ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
  if (!mgos_uart_configure(UART_NO2, &ucfg)) {
    return false;
  }

  mgos_uart_set_dispatcher(UART_NO2, uart_dispatcher, NULL /* arg */);
  mgos_uart_set_rx_enabled(UART_NO2, true);
  initialised = true;
  return true;
};

bool set_state(char* newState){
  strncpy(szState, newState, 16);
  refresh_dispay();
  return true;
};

char szMacAddress[18] = {'\0'};
char* get_mac_address(void){
  if (szMacAddress[0] == 0){
    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
    sprintf(szMacAddress, "%02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  return szMacAddress;
}