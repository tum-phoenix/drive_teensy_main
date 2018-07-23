#ifndef	PARAMETER_HPP
#define	PARAMETER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/param_server.hpp>
#include <EEPROM.h>

using namespace uavcan;

static constexpr uint8_t param_start_addr = 0;


// parameter storage with default values
static struct Params
{
   unsigned maxSpeed = 5;
   float maxMotorAmps = 10;
} configuration;

// save parameter struct in non-volatile EEPROM
void writeParamsToEEPROM()
{
    EEPROM.put(param_start_addr, configuration);
}

// read parameter from non-volatile EEPROM
void readParamsFromEEPROM()
{
    EEPROM.get(param_start_addr, configuration);
}


/*
 * Now, we need to define some glue logic between the server (below) and our configuration storage (above).
 * This is done via the interface uavcan::IParamManager.
 */
class : public uavcan::IParamManager
{
    void getParamNameByIndex(Index index, Name& out_name) const override
    {

        if (index == 0) { out_name = "maxSpeed[m/s]"; }
        if (index == 1) { out_name = "maxMotorCurrent[A]"; }
    }

    void assignParamValue(const Name& name, const Value& value) override
    {

        if (name == "maxSpeed[m/s]")
        {
            /*
             * Parameter "foo" is an integer, so we accept only integer values here.
             */
            if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
            {
                configuration.maxSpeed = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
                Serial.print("Changed maxSpeed[m/s] to: ");
                Serial.println(configuration.maxSpeed);
            }
        }
        else if (name == "maxMotorCurrent[A]")
        {
            /*
             * Parameter "bar" is a floating point, so we accept only float values here.
             */
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.maxMotorAmps = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed maxMotorCurrent[A] to: ");
                Serial.println(configuration.maxMotorAmps);
            }
        }
        else
        {
            Serial.println("Can't assign parameter!");
        }
    }

    void readParamValue(const Name& name, Value& out_value) const override
    {
        if (name == "maxSpeed[m/s]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.maxSpeed;
        }
        else if (name == "maxMotorCurrent[A]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.maxMotorAmps;
        }
        else
        {
            Serial.println("Can't read parameter: ");
        }
    }

    int saveAllParams() override
    {
        Serial.println("Save - all Params");
        writeParamsToEEPROM();
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


        if (name == "maxSpeed[m/s]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Params().maxSpeed;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 15;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 1;
        }
        else if (name == "maxMotorCurrent[A]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().maxMotorAmps;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 20;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
        }
        else
        {
            Serial.println("Can't read the limits for parameter: ");
        }
    }
} param_manager;

uavcan::ParamServer* server;



void initParameter(Node<NodeMemoryPoolSize> *node)
{
  readParamsFromEEPROM();
  server = new uavcan::ParamServer(*node);
  const int server_start_res = server->start(&param_manager);
  if (server_start_res < 0)
  {
    Serial.println("Failed to start ParamServer!");
  }else{
    Serial.println("Started Parameterserver successfully!");
  }
}

#endif
