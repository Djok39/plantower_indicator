struct MQ_STATE{
  bool      initialised;
  double    methane;
  double    co;
  bool      methane_valid;  // deprecated
  bool      co_valid;       // deprecated
};

struct MQ_STATE mq;

char* mq_read_designation(void);
double mq_read_value(void);
