/*

Module:  MCCI_Modbus_Serial_Host.cpp

Function:
    Implementation code for Serial-over-Modbus driver.

Copyright notice and License:
    See LICENSE file accompanying this project.

Author:
    Terry Moore, MCCI Corporation   September 2019

*/

#include <MCCI_Modbus_Serial_Host.h>

#include <Catena.h>
extern McciCatena::Catena gCatena;

using namespace McciCatena;

bool ModbusSerialHost::bindHost(cCatenaModbusRtuHost &bus)
    {
    if (Modbus::isDeviceAddress(bus.getID()))
        {
        return false;
        }
    else if (this->m_fRunning)
        {
        return false;
        }
    else
        {
        this->m_pBus = &bus;
        return true;
        }
    }

bool ModbusSerialHost::bindDevice(std::uint8_t devAddr)
    {
    if (this->m_fRunning)
        return false;
    else if (Modbus::isDeviceAddress(devAddr))
        {
        this->m_devAddress = devAddr;
        return true;
        }
    else
        return false;
    }


/*

Name:   ModbusSerialHost::begin()

Function:
    Prepare to use the ModbusSerialHost() object.

Definition:
    void ModbusSerialHost::begin(
            unsigned long baudrate,
            std::uint16_t config
            );

    void ModbusSerialHost::begin();

    void ModbusSerialHost::begin(
            unsigned long baudrate
            );

Descriptions:
    The baudrate and configuration are saved or re-used,
    and then the FSM for driving the serial port is started.

Returns:
    No explicit result.

*/

void ModbusSerialHost::begin()
    {
    this->begin(this->m_baudrate, this->m_serialConfig);
    }

void ModbusSerialHost::begin(unsigned long baudrate)
    {
    this->begin(baudrate, this->m_serialConfig);
    }

void ModbusSerialHost::begin(unsigned long baudrate, std::uint16_t config)
    {
    if (! this->isBound() || this->m_fRunning)
        {
        // do nothing
        return;
        }

    if (! this->m_fRegistered)
        {
        this->m_fRegistered = true;
        gCatena.registerObject(this);
        }

    // save the params
    this->m_serialConfig = config;
    this->m_serialConfig = baudrate;

    // prepare to start
    this->m_fExit = false;
    this->m_fRunning = true;
    this->m_fsm.init(*this, &ModbusSerialHost::fsmDispatch);
    }

void ModbusSerialHost::end()
    {
    if (this->m_fRunning)
        {
        this->m_fExit = true;
        this->m_fsm.eval();
        }
    }

int ModbusSerialHost::available()
    {
    if (this->m_fRunning)
        {
        return this->m_rxqueue.available();
        }
    return 0;
    }

int ModbusSerialHost::availableForWrite()
    {
    if (this->m_fRunning)
        {
        return this->m_rxqueue.free();
        }
    return 0;
    }

int ModbusSerialHost::peek()
    {
    if (this->m_fRunning)
        {
        return this->m_rxqueue.peek();
        }
    return -1;
    }

int ModbusSerialHost::read()
    {
    if (this->m_fRunning)
        {
        return this->m_rxqueue.read();
        }
    return -1;
    }

void ModbusSerialHost::pushThingsForward()
    {
    this->m_pBus->poll();
    this->poll();
    yield();
    }

// wait for all output to be delivered to the UART
void ModbusSerialHost::flush()
    {
    while (this->m_fRunning && ! this->m_txqueue.isEmpty())
        {
        this->pushThingsForward();
        }
    }

bool ModbusSerialHost::write1(std::uint8_t data)
    {
    bool const fEvent = this->m_txqueue.isEmpty();

    if (this->m_txqueue.write(data))
        {
        this->m_fsm.eval();
        return true;
        }
    else
        return false;
    }

size_t ModbusSerialHost::write(std::uint8_t data)
    {
    while ((!! *this) && ! this->m_txqueue.write(data))
        {
        // buffer is full. wait for room.
        this->pushThingsForward();
        }
    return 1;
    }

ModbusSerialHost::operator bool()
    {
    if (! this->m_fRunning)
        return false;
    
    return this->m_fConnected && this->m_statusBits.isConnected();
    }

void ModbusSerialHost::poll()
    {
    if (! this->m_fRunning)
        return;

    this->m_fsm.eval();
    }
