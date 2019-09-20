load('api_config.js');
load('api_net.js');
load('api_dash.js');
load('api_events.js');
load('api_gpio.js');
load('api_mqtt.js');
load('api_timer.js');
load('api_sys.js');
load('api_mq.js');

let nodeId = Cfg.get('node.name');
let pmMaPoints = 1286; // number of points for mean average. 643 is ~10 minuts for pms7003
let isOnline = false;
let methane_value = null;
let co_value = null;
let narodUrl = 'tcp://narodmon.ru:8283';
let getMac = ffi('char* get_mac_address(void)');
let base = Event.baseNumber("SM_");
Event.regBase(base, "Smart node") || die("Failed to register base event number");

let PMS_EVENT = base + 5;

// GPIO assignment
let pin = {
  boot: 0,
  led: 2,
  rx: 23,
  tx: 14,
  mqPower: 33,
  mqSense: 35
};

let PMS = {
  init: ffi('bool pms_init(int,int)'),
  pm1_0: ffi('double pm1_0(int)'),
  pm2_5: ffi('double pm2_5(int)'),
  pm10: ffi('double pm10(int)'),
};

let setState = ffi('bool set_state(char*)');

GPIO.set_mode(pin.led, GPIO.MODE_OUTPUT);
GPIO.write(pin.led, 0);

print('init pms7003:', PMS.init(pin.rx, pin.tx));
print('init MQ7:', MQ.init(pin.mqPower, pin.mqSense));

Event.on(base + MQ.EVENT_CO, function() {
  co_value = MQ.value();

  GPIO.toggle(pin.led);
  MQTT.pub('co/' + nodeId, JSON.stringify(MQ.value()), 0);
  GPIO.toggle(pin.led);
}, null);

Event.on(base + MQ.EVENT_METHANE, function() {
  methane_value = MQ.value();
}, null);

Event.on(base + MQ.EVENT_MQ_OFF, function() {
  co_value = null;
  methane_value = null;
}, null);

Timer.set(10000, Timer.REPEAT, function() {
  let pm10 = PMS.pm10(pmMaPoints);
  if (pm10 >= 0.0){
    GPIO.toggle(pin.led);
    MQTT.pub('pm100/' + nodeId, JSON.stringify(pm10), 0);
    MQTT.pub('pm25/' + nodeId, JSON.stringify(PMS.pm2_5(pmMaPoints)), 0);
    MQTT.pub('pm10/' + nodeId, JSON.stringify(PMS.pm1_0(pmMaPoints)), 0);
    GPIO.toggle(pin.led);
  };
}, null);

let narodMsg = null;
let onNarodmonTimer = function(){
  if (!isOnline){
    print('device offline');
    return;
  };
  let pmStr = '';
  let pm10 = PMS.pm10(pmMaPoints);
  if (pm10 >= 0.0){
    let pm2_5 = PMS.pm2_5(pmMaPoints);
    let pm1_0 = PMS.pm1_0(pmMaPoints);
    pmStr = '#PM1_0#' + JSON.stringify(pm1_0) + '#PM1\n#PM2_5#' + JSON.stringify(pm2_5) + '#PM2.5\n#PM10_0#' + JSON.stringify(pm10) + '#PM10\n';
  };
  let coStr = co_value ? '#LO#' + JSON.stringify(co_value) + '#ЛОС (MQ7)\n' : '';
  let methaneStr = methane_value ? '#HI#' + JSON.stringify(methane_value) + '#Горячий режим (MQ7)\n' : '';

  if (pm10 < 0.0 && !co_value && !methane_value){
    print('Narodmon: no data to send...');
    return;
  }
  narodMsg = '#' + getMac() + '#' + nodeId + '\n' + pmStr + methaneStr + coStr + '##';
  GPIO.toggle(pin.led);
  Net.connect({
    addr: narodUrl,
    onconnect: function(conn) {
      Net.send(conn, narodMsg);
    },
    ondata: function(conn, data) {
      if (data === 'OK\n'){
        print('sendto', narodUrl, 'ok!');
        Net.close(conn);
      }else{
        print(narodUrl, 'error:', data);
      }
      Net.discard(conn, data.length);  // Discard received data
    },
    onerror: function(conn) {
      print(narodUrl, 'connection error.');
    }
  });
  GPIO.toggle(pin.led);
};
// Запустить через 2,5 минуты (период "разогрева") таймер для отправки данных на народный мониторинг
if (narodUrl){
  Timer.set(150000, 0, function() {
    print('Narodmon timer started, device mac is', getMac());
    Timer.set(300000, Timer.REPEAT + Timer.RUN_NOW, onNarodmonTimer, null);
  }, null);
};

// Turn ON or OFF MQ7 sensor
GPIO.set_button_handler(pin.boot, GPIO.PULL_NONE, GPIO.INT_EDGE_NEG, 50, function() {
  MQ.toggle();
}, null);

Event.on(Net.STATUS_GOT_IP, function() {
  isOnline = true;
  setState('^');
  GPIO.toggle(pin.led);
  MQTT.pub('pm100/' + nodeId, 'null', 0);
  MQTT.pub('pm25/' + nodeId, 'null', 0);
  MQTT.pub('pm10/' + nodeId, 'null', 0);
  GPIO.toggle(pin.led);
}, null);

Event.on(Net.STATUS_CONNECTING, function() {
  isOnline = false;
  setState('...');
}, null);

Event.on(Net.STATUS_DISCONNECTED, function() {
  isOnline = false;
  setState('_');
}, null);
