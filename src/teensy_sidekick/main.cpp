#include "Arduino.h"
#include "phoenix_can_shield.h"

// CAN Node settings
static constexpr uint32_t nodeID = 103;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "org.phoenix.sidekick";

// application settings
static constexpr float framerate = 100;

// Power
int power_update_rate = 5;
MonotonicTime last_power_update = MonotonicTime::fromMSec(0);
#define CELL4_PIN     A0
#define OWN_CURR_PIN  A1
#define SER_CURR_PIN  A4
#define JT_CURR_PIN   A5
#define NUC_CURR_PIN  A6
#define SER_V_PIN     A10
#define JT_V_PIN      A11
#define NUC_V_PIN     A14
#define SER_CURR_FACTOR 0
#define JT_CURR_FACTOR  0
#define NUC_CURR_FACTOR 0
#define SER_V_FACTOR    0.02578125/4 // 14k + 2k
#define JT_V_FACTOR     0.02578125/4 // 14k + 2k
#define NUC_V_FACTOR    0.02578125/4 // 14k + 2k
#define Cell4_FACTOR    0.0211263/4 // 10k + 1k8 
#define OWN_CURR_FACTOR 0.00161133/4 // 0R01 + 200V/V

#include "Publisher.h"
#include "Subscriber.h"


void setup() {
  Serial.begin(115200);

  // setup power
  analogReadRes(12);
  analogReadAveraging(8);

  // init LEDs
  initLeds();

  // create a node
  systemClock = &initSystemClock();
  canDriver = &initCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // init publisher
  initPublisher(node);

  // init subscriber
  initSubscriber(node);

  // set up filters
  configureCanAcceptanceFilters(*node);

  // start up node
  node->setModeOperational();

}

void loop() {

  // wait in cycle
  cycleWait(framerate);

  // do some CAN stuff
  cycleNode(node);

  // cycle publisher
  cyclePublisher();

  // toggle heartbeat
  toggleHeartBeat();
}
