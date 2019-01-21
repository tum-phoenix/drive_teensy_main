#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/param_server.hpp>
#include <EEPROM.h>

using namespace uavcan;

static constexpr uint8_t param_start_addr = 0;

// parameter storage with default values
static struct Params
{
    float steeringOff_RL = 0;
    float steeringOff_RR = 0;
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
    void getParamNameByIndex(Index index, Name &out_name) const override
    {
        if (index == 0)
        {
            out_name = "Servo Offset RL [deg]";
        }
        if (index == 1)
        {
            out_name = "Servo Offset RR [deg]";
        }
    }

    void assignParamValue(const Name &name, const Value &value) override
    {

        if (name == "Servo Offset RL [deg]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.steeringOff_RL = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Servo Offset RL [deg] to: ");
                Serial.println(configuration.steeringOff_RL);
            }
        }
        else if (name == "Servo Offset RR [deg]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.steeringOff_RR = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Servo Offset RR [deg] to: ");
                Serial.println(configuration.steeringOff_RR);
            }
        }
        else
        {
            Serial.println("Can't assign parameter!");
        }
    }

    void readParamValue(const Name &name, Value &out_value) const override
    {
        if (name == "Servo Offset RL [deg]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.steeringOff_RL;
        }
        else if (name == "Servo Offset RR [deg]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.steeringOff_RR;
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
        return 0; // Zero means that everything is fine.
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
    void readParamDefaultMaxMin(const Name &name, Value &out_def,
                                NumericValue &out_max, NumericValue &out_min) const override
    {

        if (name == "Servo Offset RR [deg]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().steeringOff_RR;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 15;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -15;
        }
        else if (name == "Servo Offset RL [deg]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().steeringOff_RL;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 15;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -15;
        }
        else
        {
            Serial.println("Can't read the limits for parameter: ");
        }
    }
} param_manager;

uavcan::ParamServer *server;

void initParameter(Node<NodeMemoryPoolSize> *node)
{
    readParamsFromEEPROM();
    server = new uavcan::ParamServer(*node);
    const int server_start_res = server->start(&param_manager);
    if (server_start_res < 0)
    {
        Serial.println("Failed to start ParamServer!");
    }
    else
    {
        Serial.println("Started Parameterserver successfully!");
    }
}

#endif
