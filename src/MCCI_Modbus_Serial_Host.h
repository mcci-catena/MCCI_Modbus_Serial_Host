/*

Module:  MCCI_Modbus_Serial_Host.h

Function:
    Defines a serial object using a Modbus device implementing the MCCI
    Serial-over-Modbus protocol.

Copyright notice and License:
    See LICENSE file accompanying this project.

Author:
    Terry Moore, MCCI Corporation   September 2019

*/

#pragma once

#ifndef _MCCI_Modbus_Serial_Host_h_
# define _MCCI_Modbus_Serial_Host_h_

// get std::uint32_t, etc.
#include <cstdint>

// we need the Catena Arduino Platform for the FSM, so might as well use other thigns
#include <Catena_FSM.h>
#include <Catena_ModbusRtuHost.h>
#include <Catena_PollableInterface.h>

// we need HardwareSerial.h to get the SERIAL config consts in scope.
#include <HardwareSerial.h>

// we need the protocol info
#include <MCCI_Modbus_Serial_Protocol.h>

// we need to reference a ModbusRtu host.
#include <ModbusRtuV2.h>

// we inherit from Stream.h.
#include <Stream.h>

// Versioning information.

/// @brief generate a 32-bit version
#define MCCI_MODBUS_SERIAL_HOST_VERSION_CALC(major, minor, patch, local)	\
    (((major) << 24u) | ((minor) << 16u) | ((patch) << 8u) | (local))

/// @brief the version of this library - increment on every addition/deletion
#define	MCCI_MODBUS_SERIAL_HOST_VERSION	MCCI_MODBUS_SERIAL_HOST_VERSION_CALC(0, 1, 0, 0)        /* v0.1.0.0 */

/// @brief get major code
#define	MCCI_MODBUS_SERIAL_HOST_VERSION_GET_MAJOR(v)	\
    (((v) >> 24u) & 0xFFu)

/// @brief get minor code
#define	MCCI_MODBUS_SERIAL_HOST_VERSION_GET_MINOR(v)	\
    (((v) >> 16u) & 0xFFu)

/// @brief get patch number (from semantic version)
#define	MCCI_MODBUS_SERIAL_HOST_VERSION_GET_PATCH(v)	\
    (((v) >> 8u) & 0xFFu)

/// @brief get local version -- used in between official releases.
#define	MCCI_MODBUS_SERIAL_HOST_VERSION_GET_LOCAL(v)	\
    ((v) & 0xFFu)

namespace McciCatena {

// we derive from Stream rather than from HardwareSerial because
// there are portabilty issues with HardwareSerial.
class ModbusSerialHost : public Stream, cPollableObject
    {
public:
    static constexpr unsigned long kBaudRateDefault = 9600;
    static constexpr unsigned kNumRegisters = 64;
    static constexpr unsigned kQueueSize = 256;

public:
    /// @brief Default constructor needs later bind to set host/dev
    ModbusSerialHost()
        : m_pBus(nullptr)
        , m_devAddress(0)
        {
        }

    /// @brief Constructor given modbus host reference and dev addr
    ModbusSerialHost(cCatenaModbusRtuHost &arg_bus, std::uint8_t arg_devId)
        : m_pBus(&arg_bus)
        , m_devAddress(arg_devId)
        {
        }

    // neither copyable nor movable.
    ModbusSerialHost(const ModbusSerialHost&) = delete;
    ModbusSerialHost& operator=(const ModbusSerialHost&) = delete;
    ModbusSerialHost(const ModbusSerialHost&&) = delete;
    ModbusSerialHost& operator=(const ModbusSerialHost&&) = delete;

    // our reference is the Adafruit HardwareSerial.h

    /// @brief initialize with defaults.
    void begin();

    /// @brief initialize setting baudrate.
    void begin(unsigned long baudrate);

    /// @brief intiailize setting baudrate and configuration
    void begin(unsigned long baudrate, std::uint16_t config);

    /// @brief stop operations
    void end();

    /// @brief return count of available bytes in the input queue
    int available();

    /// @brief return number of availalbe bytes in the output queue
    int availableForWrite();

    /// @brief return first character in input queue, without removing from queue; -1 for none.
    int peek();

    /// @brief return first character from input queueu, removing it; -1 for none.
    int read();

    /// @brief discard the output queue
    void flush();

    /// @brief write a single byte, blocking if need. Return number of bytes written (1). 
    size_t write(uint8_t data);

    using Print::write;     // we need Print::write(str), Print::write(buf, n)

    /// @brief for compatibility with USB serial, return true when device is up
    operator bool();

    /// @brief set Modbus host
    bool bindHost(cCatenaModbusRtuHost &bus);

    /// @brief set device address
    bool bindDevice(std::uint8_t devAddr);

    bool isBound() const
        {
        return this->m_pBus != nullptr && cCatenaModbusRtuHost::isDeviceAddress(this->m_devAddress);
        }

    /// @brief poll for operational status
    void poll();

//--------------------
//  protected methods
//--------------------

protected:
    enum class State : std::uint8_t
        {
        stNoChange = 0, // this name must be present: indicates "no change of state"

        stInitial,      // this name must be present: it's the starting state.
        stConfigure,    // configuring device
        stAwaitDevice,  // wait for a device to be connected
        stIdle,         // parked; not doing anything.
        stRead,         // active; sleeping between polls.
        stWrite,        // active; writing data.

        stFinal,        // this name must be present, it's the terminal state.
        };

    static constexpr char *getStateName(State s)
        {
        switch (s)
            {
        case State::stNoChange:     return "NoChange";
        case State::stInitial:      return "Initial";
        case State::stConfigure:    return "Configure";
        case State::stAwaitDevice:  return "AwaitDevice";
        case State::stIdle:         return "Idle";
        case State::stRead:         return "Read";
        case State::stWrite:        return "Write";
        case State::stFinal:        return "Final";
        default:                    return "<<unknown>>";
            }
        }

//--------------------
// private ring-buffer class
//--------------------
private:
    template <std::uint16_t Nbuffer>
    class RingBuffer
        {
    public:
        void init()
            {
            this->m_head = this->m_tail = this->m_buffer;
            }

        std::uint8_t *nextCell(std::uint8_t *pCell)
            {
            auto pResult = pCell + 1;
            if (pResult == this->m_buffer + Nbuffer)
                pResult = this->m_buffer;
            return pResult;
            }

        bool isFull()
            {
            return this->nextCell(this->m_head) == this->m_tail; 
            }

        bool isEmpty() const
            {
            return this->m_head == this->m_tail;
            }

        std::uint16_t available()
            {
            std::uint16_t result;

            auto pTail = this->m_tail;
            auto pHead = this->m_head;
            if (pTail > pHead)
                pHead += Nbuffer;
            return pHead - pTail;
            }

        std::uint16_t free()
            {
            return Nbuffer - this->available() - 1;
            }
        
        int peek()
            {
            if (this->m_tail != this->m_head)
                return *this->m_tail;
            return -1;
            }

        int read()
            {
            auto pTail = this->m_tail;
            if (pTail != this->m_head)
                {
                int result = *pTail;
                this->m_tail = this->nextCell(pTail);
                return result;
                }
            return -1;
            }

        bool write(std::uint8_t c)
            {
            auto pHead = this->m_head;
            auto pNewHead = this->nextCell(pHead);
            if (pNewHead == this->m_tail)
                return false;

            *pHead = c;
            return true;
            }

    private:
        std::uint8_t *m_head;
        std::uint8_t *m_tail;
        std::uint8_t m_buffer[Nbuffer];
        };

//--------------------
//  private methods
//--------------------
private:
    // the dispatch function used for the FSM.
    State fsmDispatch(State currentState, bool fEntry);

    // call this when busy-waiting.
    void pushThingsForward();

    // internal write/event handler
    bool write1(std::uint8_t data);

//---------------------
//  private members
//---------------------
private:
    // the FSM
    cFSM <ModbusSerialHost, State>  m_fsm;

    // the datagram
    cModbusDatagram                 m_datagram;

    // the Modbus bus to be used.
    cCatenaModbusRtuHost            *m_pBus;

    // the baudrate (or the last baudrate selected).
    unsigned long                   m_baudrate = kBaudRateDefault;

    // the configuration
    std::uint16_t                   m_serialConfig = SERIAL_8N1;

    // the device address.
    std::uint8_t                    m_devAddress;

    // set true once registered with gCatena.
    bool                            m_fRegistered : 1;

    // set true when running.
    bool                            m_fRunning: 1;

    // set true to ask FSM to exit
    bool                            m_fExit: 1;

    // set true when a device is connected.
    bool                            m_fConnected: 1;

    // Most recent status
    ModbusSerialProtocol::StatusBits m_statusBits;

    // the register read/write bufffer
    std::uint16_t                   m_regs[kNumRegisters];

    // the input buffer
    RingBuffer<kQueueSize>          m_rxqueue;
    // the output buffer
    RingBuffer<kQueueSize>          m_txqueue;
    };

} // namespace McciCatena


#endif // _MCCI_Modbus_Serial_Host_h_
