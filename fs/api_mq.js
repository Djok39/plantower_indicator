load('api_events.js');

let MQ = {
  init: ffi('bool mq_init(int,int)'),
  toggle: ffi('bool mq_toggle(void)'),
  value: ffi('double mq_read_value(void)'),
  designation: ffi('char* mq_read_designation(void)')
};

MQ.EVENT_METHANE = 1;
MQ.EVENT_CO = 2;
MQ.EVENT_MQ_OFF = 6;