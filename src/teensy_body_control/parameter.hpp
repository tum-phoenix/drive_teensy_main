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
   float maxSpeedRC = 2;
   float maxSpeedAuton = 5;
   float maxMotorAmps = 16;
   float speedKp = 20;
   float speedKi = 0.0;
   float speedKd = 0.0;
   float tvFactor = 0.3;
   float acFactor = 0.3;
   float steeringOff_FL = 0;
   float steeringOff_FR = 0;
   float park_steer0 = 32;
   float park_s_x0 = 0.25;
   float park_x0 = 0.1;
   float park_s_x1 = 0.25;

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

        if (index == 0) { out_name = "maxSpeedRC[m/s]"; }
        if (index == 1) { out_name = "maxSpeedAuton[m/s]"; }
        if (index == 2) { out_name = "maxMotorCurrent[A]"; }
        if (index == 3) { out_name = "speedKp[-]"; }
        if (index == 4) { out_name = "speedKi[-]"; }
        if (index == 5) { out_name = "speedKd[-]"; }
        if (index == 6) { out_name = "tvFactor[-]"; }
        if (index == 7) { out_name = "acFactor[-]"; }
        if (index == 8) { out_name = "Servo Offset FL [deg]"; }
        if (index == 9) { out_name = "Servo Offset FR [deg]"; }
        if (index == 10){ out_name = "Park Steer Angle"; }
        if (index == 11){ out_name = "Park Steer Dist"; }
        if (index == 12){ out_name = "Park Straight Dist"; }
        if (index == 13){ out_name = "Park Steer 2 Dist"; }
    }

    void assignParamValue(const Name& name, const Value& value) override
    {

        if (name == "maxSpeedRC[m/s]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.maxSpeedRC = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed maxSpeedRC[m/s] to: ");
                Serial.println(configuration.maxSpeedRC);
            }
        }
        if (name == "maxSpeedAuton[m/s]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.maxSpeedAuton = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed maxSpeedAuton[m/s] to: ");
                Serial.println(configuration.maxSpeedAuton);
            }
        }
        else if (name == "maxMotorCurrent[A]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.maxMotorAmps = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed maxMotorCurrent[A] to: ");
                Serial.println(configuration.maxMotorAmps);
            }
        }
        else if (name == "speedKp[-]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.speedKp = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed speedKp[-] to: ");
                Serial.println(configuration.speedKp);
            }
        }
        else if (name == "speedKi[-]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.speedKi = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed speedKi[-] to: ");
                Serial.println(configuration.speedKi);
            }
        }
        else if (name == "speedKd[-]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.speedKd = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed speedKd[-] to: ");
                Serial.println(configuration.speedKd);
            }
        }
        else if (name == "tvFactor[-]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.tvFactor = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed tvFactor[-] to: ");
                Serial.println(configuration.tvFactor);
            }
        }
        else if (name == "acFactor[-]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.acFactor = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Changed acFactor[-] to: ");
                Serial.println(configuration.acFactor);
            }
        }
        else if (name == "Servo Offset FL [deg]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.steeringOff_FL = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Servo Offset FL [deg] to: ");
                Serial.println(configuration.steeringOff_FL);
            }
        }
        else if (name == "Servo Offset FR [deg]")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.steeringOff_FR = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Servo Offset FR [deg] to: ");
                Serial.println(configuration.steeringOff_FR);
            }
        }
        else if (name == "Park Steer Angle")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.park_steer0 = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Park Steer Angle to: ");
                Serial.println(configuration.park_steer0);
            }
        }
        else if (name == "Park Steer Dist")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.park_s_x0 = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Park Steer Dist to: ");
                Serial.println(configuration.park_s_x0);
            }
        }
        else if (name == "Park Straight Dist")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.park_x0 = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Park Straight Dist to: ");
                Serial.println(configuration.park_x0);
            }
        }
        else if (name == "Park Steer 2 Dist")
        {
            if (value.is(uavcan::protocol::param::Value::Tag::real_value))
            {
                configuration.park_s_x1 = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
                Serial.print("Park Steer Dist 2 to: ");
                Serial.println(configuration.park_s_x1);
            }
        }
        else 
        {
            Serial.println("Can't assign parameter!");
        }
    }

    void readParamValue(const Name& name, Value& out_value) const override
    {
        if (name == "maxSpeedRC[m/s]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.maxSpeedRC;
        }
        else if (name == "maxSpeedAuton[m/s]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.maxSpeedAuton;
        }
        else if (name == "maxMotorCurrent[A]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.maxMotorAmps;
        }
        else if (name == "speedKp[-]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.speedKp;
        }
        else if (name == "speedKi[-]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.speedKi;
        }
        else if (name == "speedKd[-]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.speedKd;
        }
        else if (name == "tvFactor[-]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.tvFactor;
        }
        else if (name == "acFactor[-]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.acFactor;
        }
        else if (name == "Servo Offset FL [deg]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.steeringOff_FL;
        }
        else if (name == "Servo Offset FR [deg]")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.steeringOff_FR;
        }
        else if (name == "Park Steer 2 Dist")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.park_s_x1;
        }
        else if (name == "Park Steer Dist")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.park_s_x0;
        }
        else if (name == "Park Straight Dist")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.park_x0;
        }
        else if (name == "Park Steer Angle")
        {
            out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.park_steer0;
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


        if (name == "maxSpeedRC[m/s]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().maxSpeedRC;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 10;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = .1;
        }
        if (name == "maxSpeedAuton[m/s]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().maxSpeedAuton;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 10;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = .1;
        }
        else if (name == "maxMotorCurrent[A]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().maxMotorAmps;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 20;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
        }
        else if (name == "speedKp[-]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().speedKp;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 500;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "speedKi[-]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().speedKi;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 10;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "speedKd[-]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().speedKd;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 10;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "tvFactor[-]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().tvFactor;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "acFactor[-]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().acFactor;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 0;
        }
        else if (name == "Servo Offset FR [deg]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().steeringOff_FR;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 15;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -15;
        }
        else if (name == "Servo Offset FL [deg]")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().steeringOff_FL;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 15;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -15;
        }
        else if (name == "Park Steer Angle")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().park_steer0;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 40;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -40;
        }
        else if (name == "Park Steer Dist")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().park_s_x0;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1.;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -1.;
        }
        else if (name == "Park Straight Dist")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().park_s_x0;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1.;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -1.;
        }
        else if (name == "Park Steer 2 Dist")
        {
            out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Params().park_s_x1;
            out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 1.;
            out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -1.;
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
