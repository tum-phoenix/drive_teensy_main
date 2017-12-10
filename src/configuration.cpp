#include <UAVCAN.hpp>
#include <uavcan/protocol/param_server.hpp>

using namespace uavcan;


static const unsigned NodeMemoryPoolSize = 8192;
static constexpr uint32_t bitrate = 1000000;
static constexpr uint32_t nodeID = 100;

Node<NodeMemoryPoolSize> *node;

ISystemClock& getSystemClock()
{
    return uavcan_nxpk20::SystemClock::instance();
}

ICanDriver& getCanDriver()
{
    static bool initialized = false;
    if (!initialized)
    {
        initialized = true;
        int res = uavcan_nxpk20::CanDriver::instance().init(bitrate);
        if (res < 0)
        {
            Serial.println("Error on CanDriver initialization");
        }
    }
    return uavcan_nxpk20::CanDriver::instance();
}

void background_processing()
{
  ; // do nothing
}

/*
* This would be our configuration storage.
*/
static struct Params
{
   unsigned foo = 42;
   float bar = 0.123456F;
   double baz = 1e-5;
   std::string booz = "Hello world!";
} configuration;

/*
 * Now, we need to define some glue logic between the server (below) and our configuration storage (above).
 * This is done via the interface uavcan::IParamManager.
 */
class : public uavcan::IParamManager
{
    void getParamNameByIndex(Index index, Name& out_name) const override
    {

        if (index == 0) { out_name = "foo"; }
        if (index == 1) { out_name = "bar"; }
        if (index == 2) { out_name = "baz"; }
        if (index == 3) { out_name = "booz"; }
    }

    void assignParamValue(const Name& name, const Value& value) override
    {
        if (name == "foo")
        {
            /*
             * Parameter "foo" is an integer, so we accept only integer values here.
             */
            if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
            {
                configuration.foo = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
                Serial.print("Changed foo to: ");
                Serial.println(configuration.foo);
            }
        }
        else if (name == "bar")
        {
            /*
             * Parameter "bar" is a floating point, so we accept only float values here.
             */
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.bar = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed bar to: ");
                Serial.println(configuration.bar);
            }
        }
        else if (name == "baz")
        {
            /*
             * Ditto
             */
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.baz = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed baz to: ");
                Serial.println(configuration.baz);
            }
        }
        else if (name == "booz")
        {
            /*
             * Parameter "booz" is a string, so we take only strings.
             */
            if (value.is(uavcan::protocol::param::Value::Tag::string_value))
            {
                configuration.booz = value.as<uavcan::protocol::param::Value::Tag::string_value>()->c_str();
                Serial.print("Changed booz to: ");
                Serial.println(configuration.booz.c_str());
            }
        }
        else
        {
            Serial.println("Can't assign parameter!");
        }
    }

    void readParamValue(const Name& name, Value& out_value) const override
    {
        if (name == "foo")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.foo;
        }
        else if (name == "bar")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.bar;
        }
        else if (name == "baz")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.baz;
        }
        else if (name == "booz")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::string_value>() = configuration.booz.c_str();
        }
        else
        {
            Serial.println("Can't read parameter: ");
        }
    }

    int saveAllParams() override
    {
        Serial.println("Save - this implementation does not require any action");
        return 0;     // Zero means that everything is fine.
    }

    int eraseAllParams() override
    {
        Serial.println("Erase - all params reset to default values");
        configuration = Params();
        return 0;
    }

    /**
     * Note that this method is optional. It can be left unimplemented.
     */
    void readParamDefaultMaxMin(const Name& name, Value& out_def,
                                NumericValue& out_max, NumericValue& out_min) const override
    {
        if (name == "foo")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Params().foo;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 9000;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        }
        else if (name == "bar")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().bar;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "baz")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().baz;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "booz")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::string_value>() = Params().booz.c_str();
            Serial.println("Limits for 'booz' are not defined");
        }
        else
        {
            Serial.println("Can't read the limits for parameter: ");
        }
    }
} param_manager;


/*
 * Now, this node can be reconfigured via UAVCAN. Awesome.
 * Many embedded applications require a restart before the new configuration settings can
 * be applied, so it is highly recommended to also support the remote restart service.
 */
class : public uavcan::IRestartRequestHandler
{
    bool handleRestartRequest(uavcan::NodeID request_source) override
    {
        Serial.println("Got a remote restart request!");
        /*
         * We won't really restart, so return 'false'.
         * Returning 'true' would mean that we're going to restart for real.
         * Note that it is recognized that some nodes may not be able to respond to the
         * restart request (e.g. if they restart immediately from the service callback).
         */
        return false;
    }
} restart_request_handler;


uavcan::ParamServer* server;

void setup()
{
  delay(3000);
  Serial.begin(9600);
  node = new Node<NodeMemoryPoolSize>(getCanDriver(), getSystemClock());
  Serial.println("Application node created");

  node->setNodeID(nodeID);
  node->setName("org.phoenix.configuration");


  // Start Node and then the subscriber
  if(node->start() < 0){
    Serial.println("Unable to start node!");
  }

  node->setModeOperational();

  node->setRestartRequestHandler(&restart_request_handler);

  server = new uavcan::ParamServer(*node);
  const int server_start_res = server->start(&param_manager);
  if (server_start_res < 0)
  {
     Serial.println("Failed to start ParamServer!");
  }else{
    Serial.println("Started Parameterserver successfully!");
  }



}

int ct = 0;

void loop()
{
  // Wait for frames 100ms, then do other stuff
  Serial.print("Application spinning. Heartbeat counter: ");
  Serial.println(ct);
  ct++;
  const int res = node->spin(uavcan::MonotonicDuration::fromMSec(1000));
  if (res < 0)
  {
      Serial.println("Error while spinning...");
  }
  // do other stuff
  background_processing();
}
