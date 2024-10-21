"""File provides encoding and decoding of packets sent to and from the firmware."""

import ctypes
from enum import Enum
import os
import json
from ctypes.util import find_library
from typing import Dict
from axcend_focus_device_config import device_config

PACKET_MAX_LENGTH = 16  # bytes

PROTO_PREFIX = b"proto1 "

# Used with the pump set command
class PumpSet():
    """Responsible for defining the pump set commands."""
    TARGET_PRESSURE_COMMAND = 4
    FLOW_RATE_COMMAND = 6 # For the flow rate
    DIRECTION_COMMAND = 8 # For the direction, 1 is dispense, 0 is aspirate
    DIRECTION_DISPENSE = 0
    DIRECTION_FILL = 1
    CONTROL_MODE_COMMAND = 10 # For the control mode, 0 is pressure, 1 is disposition
    CONTROL_MODE_FLOW = 0
    CONTROL_MODE_PRESSURE = 1

class PumpCommand():
    """Responsible for defining the pump commands."""
    PUMP_IDLE = 0
    PUMP_FILL = 1
    PUMP_DISPENSE = 2
    PUMP_ASPIRATE = 3
    PUMP_STOP = 4

class AcknowledgementChannelID(Enum):
    """Enum is responsible for defining the acknowledgement channel IDs."""
    COMMAND = 0
    METHOD = 1

class AbortPacketReasons:
    """Enum is responsible for defining the abort packet reasons."""
    User = 0x0000
    ResetRun = 0x0001
    OverPressure = 0x0100
    SolventExchangeError = 0x0300
    ASyringeUnderLimit = 0xa000
    ASyringeOverLimit = 0xa100
    ASyringeAirLimit = 0xa200
    BSyringeUnderLimit = 0xb000
    BSyringeOverLimit = 0xb100
    BSyringeAirLimit = 0xb200
    HeartBeat = 0x0011
    NoDVPackets = 0x0111
    TemperatureHigh = 0xc000
    PositionNotLive = 0xc0ab
    PressureNotLive = 0xc2ab

class TestPhase:
    PhaseIdle = 0
    PhaseRefill = 1
    PhasePressurize = 2
    PhaseEquilibrate = 3
    PhaseRun = 4
    PhaseFinalize = 5
    PhaseService = 6
    PhaseAbort = 7

class DataAcquisitionState(Enum):
    """Enum is responsible for defining the data acquisition state."""

    DISABLED = 0
    CARTRIDGE_ONLY = 1
    VALVE_PUMPS_ONLY = 2
    CARTRIDGE_AND_VALVE_PUMPS = 3


class Field(ctypes.Structure):
    _fields_ = [
        ("packet_ID", ctypes.c_uint8),
        ("type_ID", ctypes.c_uint8),
        ("unitId", ctypes.c_uint8),
        ("data", ctypes.c_uint8 * 12),
        ("end", ctypes.c_uint8),
    ]


class Packet(ctypes.Union):
    _fields_ = [("raw", ctypes.c_uint8 * PACKET_MAX_LENGTH), ("fields", Field)]


class _PhaseInfoBits(ctypes.Structure):
    _fields_ = [
        ("phase", ctypes.c_uint8, 3),
        ("valveB", ctypes.c_uint8, 1),
        ("valveA", ctypes.c_uint8, 2),
        ("moving", ctypes.c_uint8, 1),
        ("direction", ctypes.c_uint8, 1),
    ]


class PhaseInfo(ctypes.Union):
    _fields_ = [("raw", ctypes.c_uint8), ("bits", _PhaseInfoBits)]


class _PressureBits(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("packet_ID", ctypes.c_uint8),
        ("type_ID", ctypes.c_uint8),
        ("phase", PhaseInfo),
        ("streamId", ctypes.c_uint16),
        ("pressureA", ctypes.c_uint16),
        ("pressureB", ctypes.c_uint16),
        ("positionA", ctypes.c_uint16),
        ("positionB", ctypes.c_uint16),
        ("flowRate", ctypes.c_uint16),
        ("end", ctypes.c_uint8),
    ]


class PressureInfo(ctypes.Union):
    _fields_ = [("raw", ctypes.c_uint8 * PACKET_MAX_LENGTH), ("fields", _PressureBits)]


class _UV_data(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("channel_ID", ctypes.c_uint8),
        ("time_stamp", ctypes.c_uint32),
        ("value", ctypes.c_uint32),
    ]


class UVData(ctypes.Union):
    _fields_ = [("raw", ctypes.c_uint8 * 9), ("fields", _UV_data)]


class SystemParametersFields(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),  # Offset 0
        ("hardwareVersion", ctypes.c_uint8),  # Offset 1
        ("valvePumpCtlType", ctypes.c_uint8),  # Offset 2
        ("backPlateBoardType", ctypes.c_uint8),  # Offset 3
        ("boardSerialNumber", ctypes.c_uint32),  # Offset 4
        ("productionDate", ctypes.c_uint32),  # Offset 8
        ("productionLocation", ctypes.c_uint16),  # Offset 12
        ("maxPressureRating", ctypes.c_uint16),  # Offset 14
        ("pressureSensorType", ctypes.c_uint8),  # Offset 16
        ("valveSolType", ctypes.c_uint8),  # Offset 17
        ("valveInjType", ctypes.c_uint8),  # Offset 18
        ("valvePrgType", ctypes.c_uint8),  # Offset 19
        ("pumpSyringeAEmptyValue", ctypes.c_uint16),  # Offset 20
        ("pumpSyringeAFullValue", ctypes.c_uint16),  # Offset 22
        ("pumpSyringeBEmptyValue", ctypes.c_uint16),  # Offset 24
        ("pumpSyringeBFullValue", ctypes.c_uint16),  # Offset 26
        ("pumpLifetimeUsage", ctypes.c_uint32),  # Offset 28
        ("injectionVolume", ctypes.c_uint16),  # Offset 32
        ("pumpSyringesType", ctypes.c_uint8),  # Offset 34
        ("pumpSyringeAVolume", ctypes.c_uint8),  # Offset 35
        ("pumpSyringeBVolume", ctypes.c_uint8),  # Offset 36
    ]


class SystemParameters_t(ctypes.Union):
    _pack_ = 1
    _fields_ = [("raw", ctypes.c_uint8 * 37), ("fields", SystemParametersFields)]


class CartridgeData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("revision", ctypes.c_uint8),
        ("serial_number", ctypes.c_char * 12),
        ("max_pressure_rating", ctypes.c_uint8),
        ("has_oven", ctypes.c_uint8),
        ("is_third_party", ctypes.c_uint8),
        ("last_write_date", ctypes.c_char * 6),
        ("total_run_time", ctypes.c_uint32),
        ("cartridge_flags", ctypes.c_uint8),
        ("detector_count", ctypes.c_uint8),
        ("detector_1_type", ctypes.c_uint8),
        ("detector_1_wavelength", ctypes.c_uint16),
        ("detector_1_path_length", ctypes.c_uint16),
        ("detector_2_type", ctypes.c_uint8),
        ("detector_2_wavelength", ctypes.c_uint16),
        ("detector_2_path_length", ctypes.c_uint16),
        ("column_count", ctypes.c_uint8),
        ("column_1_length", ctypes.c_uint8),
        ("column_1_diameter", ctypes.c_uint16),
        ("column_1_particle_size", ctypes.c_uint8),
        ("column_1_description", ctypes.c_char * 12),
        ("column_2_length", ctypes.c_uint8),
        ("column_2_diameter", ctypes.c_uint16),
        ("column_2_particle_size", ctypes.c_uint8),
        ("column_2_description", ctypes.c_char * 12),
        ("detector_1_adc_gain", ctypes.c_uint8),
        ("detector_1_adc_delay", ctypes.c_uint8),
        ("detector_1_led_pot", ctypes.c_uint8),
        ("detector_2_adc_gain", ctypes.c_uint8),
        ("detector_2_adc_delay", ctypes.c_uint8),
        ("detector_2_led_pot", ctypes.c_uint8),
        ("bluetooth_disabled", ctypes.c_uint8),
        ("report_rate", ctypes.c_uint8),
        ("word_rate", ctypes.c_uint8),
        ("unused", ctypes.c_uint8),
        ("samples_per_point", ctypes.c_uint8),
        ("bits_to_remove", ctypes.c_uint8),
    ]
CARTRIDGE_MEMORY_SIZE = ctypes.sizeof(CartridgeData)
CARTRIDGE_MEMORY_PACKET_HEADER_SIZE = 2
CARTRIDGE_MEMORY_PACKET_SIZE = CARTRIDGE_MEMORY_PACKET_HEADER_SIZE + CARTRIDGE_MEMORY_SIZE

class CartridgeMemory_t(ctypes.Union):
    _pack_ = 1
    _fields_ = [("raw", ctypes.c_uint8 * CARTRIDGE_MEMORY_SIZE), ("data", CartridgeData)]

class PacketTranscoder:
    """Class that gives the ability to encode and decode packets."""

    def __init__(self):

        # Load the packet transcoder shared library
        self.packet_transcoder = self.load_packet_transcoder()

    def load_packet_transcoder(self) -> ctypes.CDLL:
        """
        Load a packet transcoder shared library.

        Load the packet encoding / decoding library and define argument types
        and return types for various functions within the library.
        """
        target_library_name = "axcend_packets"
        target_library_path = find_library(target_library_name)

        if not target_library_path:
            raise FileNotFoundError(f"The library '{target_library_name}' does not exist.")

        print("Loading the packet encoding / decoding library")
        packet_transcoder = ctypes.CDLL(target_library_path)

        # Define the argument types and return type for each function
        packet_transcoder.packetEncode_0.argtypes = [
            ctypes.c_uint8,  # for uint8_t packet_ID
            ctypes.c_uint8,  # for uint8_t type_ID
            ctypes.POINTER(Packet),  # for Packet *packet
        ]
        packet_transcoder.packetEncode_0.restype = ctypes.c_int  # for int return type

        packet_transcoder.packetEncode_16.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(Packet),
            ctypes.c_uint16,
        ]
        packet_transcoder.packetEncode_16.restype = ctypes.c_int

        packet_transcoder.packetEncode_16_16_b.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(Packet),
            ctypes.c_uint16,
            ctypes.c_uint16,
        ]
        packet_transcoder.packetEncode_16_16_b.restype = ctypes.c_int

        packet_transcoder.packetEncode_text.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(Packet),
            ctypes.c_char_p,
        ]
        packet_transcoder.packetEncode_text.restype = ctypes.c_int

        packet_transcoder.packetEncode_8_32.argtypes = [
            ctypes.POINTER(Packet),
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint32,
        ]
        packet_transcoder.packetEncode_8_32.restype = ctypes.c_int

        packet_transcoder.packetEncode_16_8_32.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(Packet),
            ctypes.c_uint16,
            ctypes.c_uint8,
            ctypes.c_uint32,
        ]
        packet_transcoder.packetEncode_16_8_32.restype = ctypes.c_int

        packet_transcoder.packetEncode_8_all.argtypes = [
            ctypes.POINTER(Packet),
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_uint8,
        ]
        packet_transcoder.packetEncode_8_all.restype = ctypes.c_int

        packet_transcoder.packetDecode_oven_data.argtypes = [
            ctypes.POINTER(Packet),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.POINTER(ctypes.c_uint16),
        ]
        packet_transcoder.packetDecode_oven_data.restype = ctypes.c_int

        packet_transcoder.packetEncode_oven_data.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(Packet),
            ctypes.c_uint32,
            ctypes.c_uint8,
            ctypes.c_uint32,
            ctypes.c_uint8,
            ctypes.c_uint16,
        ]
        packet_transcoder.packetEncode_oven_data.restype = ctypes.c_int

        return packet_transcoder

    def create_heartbeat_packet(self) -> bytes:
        """Return the encoded heart beat packet."""
        packet = Packet()

        # Make a packet
        packet_ID = ctypes.c_uint8(ord("G"))
        type_ID = ctypes.c_uint8(ord("H"))
        self.packet_transcoder.packetEncode_0(packet_ID, type_ID, ctypes.byref(packet))

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_acknowledgement_packet(self) -> bytes:
        """Create a default acknowledgement packet hex encoded string."""
        ACK_VALUE = b"ACK"

        packet = Packet()

        # Make a packet
        packet_ID = ctypes.c_uint8(ord("D"))
        type_ID = ctypes.c_uint8(ord("A"))
        buf = ctypes.create_string_buffer(ACK_VALUE, len(ACK_VALUE))
        self.packet_transcoder.packetEncode_text(
            packet_ID, type_ID, ctypes.byref(packet), buf
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_system_parameters_packet(self) -> bytes:
        """Make the system parameters packet."""
        # Variables
        data = device_config.system_parameter_read_all()

        # Create an instance of the SystemParameters_t union
        params = SystemParameters_t()

        # Set the fields of the union according to the data in the JSON object
        params.fields.version = int(data["version"])
        params.fields.hardwareVersion = int(data["hardwareVersion"])
        params.fields.valvePumpCtlType = int(data["valvePumpCtlType"])
        params.fields.backPlateBoardType = int(data["backPlateBoardType"])
        params.fields.boardSerialNumber = int(data["boardSerialNumber"])
        params.fields.productionDate = int(data["productionDate"])
        params.fields.productionLocation = int(data["productionLocation"])
        params.fields.maxPressureRating = int(data["maxPressureRating"])
        params.fields.pressureSensorType = int(data["pressureSensorType"])
        params.fields.valveSolType = int(data["valveSolType"])
        params.fields.valveInjType = int(data["valveInjType"])
        params.fields.valvePrgType = int(data["valvePrgType"])
        params.fields.pumpSyringeAEmptyValue = int(data["pumpSyringeAEmptyValue"])
        params.fields.pumpSyringeAFullValue = int(data["pumpSyringeAFullValue"])
        params.fields.pumpSyringeBEmptyValue = int(data["pumpSyringeBEmptyValue"])
        params.fields.pumpSyringeBFullValue = int(data["pumpSyringeBFullValue"])
        params.fields.pumpLifetimeUsage = int(data["pumpLifetimeUsage"])
        params.fields.injectionVolume = int(data["injectionVolume"])
        params.fields.pumpSyringesType = int(data["pumpSyringesType"])
        params.fields.pumpSyringeAVolume = int(data["pumpSyringeAVolume"])
        params.fields.pumpSyringeBVolume = int(data["pumpSyringeBVolume"])

        # Serialize the struct into a hex encoded string
        packet_string = b"SysParams "
        packet_string += bytes(params.raw).hex().upper().encode()

        return packet_string

    def create_event_packet(self) -> bytes:
        """Create an event packet."""
        # Create a blank packet
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("D"))
        type_ID = ctypes.c_uint8(ord("E"))
        self.packet_transcoder.packetEncode_0(
            packet_ID, type_ID, ctypes.byref(packet)
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_abort_packet(self, reason: int) -> bytes:
        """Create an abort packet."""
        # Create a blank packet
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("Q"))
        self.packet_transcoder.packetEncode_16(
            packet_ID, type_ID, ctypes.byref(packet), ctypes.c_uint16(reason)
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def decode_packet(self, packet_string: str) -> list:
        """Decode a hex string packet from the firmware and return the data as a dictionary."""
        # Convert the hex encoded data to byte array
        prefix, data = packet_string.split(" ", 1)

        data = bytes.fromhex(data)

        # Not sure yet what type of packet to assign yet
        packet = None

        # Get the packet type
        packet_type = chr(data[0]) + chr(data[1])

        # Check the size of the packet
        if prefix == "proto1":
            # Make sure the packet is the correct size
            assert len(data) == PACKET_MAX_LENGTH, "Invalid packet size for proto1"

            # Move the data into a packet object
            packet = Packet()

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), data, len(data))

        elif prefix == "cartridge_config":
            assert (
                len(data) == CARTRIDGE_MEMORY_PACKET_SIZE
            ), "Invalid packet size for cartridge_config"

            # Move the data into a packet object
            packet = CartridgeMemory_t()

            # Trim first two bytes of the data (packet ID and type ID)
            data = data[CARTRIDGE_MEMORY_PACKET_HEADER_SIZE:]

            # Copy the bytes into the packet
            ctypes.memmove(ctypes.byref(packet.raw), data, len(data))

        elif prefix == "SysParams":
            pass

        return [prefix, packet_type, packet]

    def create_valve_rotate_packet(
        self, solvent_valve_position: int, injection_valve_position: int
    ) -> bytes:
        """Create a rotate valve packet."""
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("V"))
        valve_position = (injection_valve_position << 4) | solvent_valve_position
        # Call packetEncode_16_16_b
        self.packet_transcoder.packetEncode_16_16_b(
            packet_ID,
            type_ID,
            ctypes.byref(packet),
            ctypes.c_uint16(2),
            ctypes.c_uint16(valve_position),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string
    
    def create_pump_positioning_packet(self, fill_volume_a, fill_volume_b) -> bytes:
        """Create a pump positioning packet.

        Args:
            fill_volume_a: The fill volume for pump A in uL.
            fill_volume_b: The fill volume for pump B in uL.
        """
        packet = Packet()

        # Encode the packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("F"))
        # Call packetEncode_16_16_b
        self.packet_transcoder.packetEncode_16_16_b(
            packet_ID,
            type_ID,
            ctypes.byref(packet),
            ctypes.c_uint16(fill_volume_a * 10), # Firmware takes value in 0.1 uL increments
            ctypes.c_uint16(fill_volume_b * 10), # Firmware takes value in 0.1 uL increments
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_pump_pressurize_system_packet(self, target_pressure: int):
        """Create a pump load packet."""
        packet = Packet()

        # Encode the packet
        self.packet_transcoder.packetEncode_8_all(
            ctypes.byref(packet), 
            ctypes.c_uint8(ord("C")), 
            ctypes.c_uint8(ord("L")),
            ctypes.c_int8(target_pressure / 100),
            ctypes.c_int8(target_pressure / 100),
            ctypes.c_int8(0),
            (ctypes.c_uint8 * 8)(0, 0, 0, 0, 0, 0, 0, 0),
            ctypes.c_uint8(8),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_pump_set_packet(self, address_to_set: int, value_to_set: int) -> bytes:
        """Create a pump set packet."""
        packet = Packet()

        # Encode the packet
        # Call packetEncode_16_16_b
        self.packet_transcoder.packetEncode_16_8_32(
            ctypes.c_uint8(ord("C")),
            ctypes.c_uint8(ord("C")),
            ctypes.byref(packet),
            ctypes.c_uint16(address_to_set),
            ctypes.c_uint8(4), # 4 is the number of bytes to set
            ctypes.c_uint32(value_to_set),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_pump_packet(self, time_in_seconds, start_ratio, end_ratio, control_mode):
        """Encode the packet with the given time, start_ratio, end_ratio, and flags."""
        PUMP_FLAGS_TIME_SCALE_FACTOR_BIT_POSITION = 0x01
        PUMP_FLAGS_TIME_SCALE_FACTOR_BIT_MASK = 0x3
        PUMP_FLAGS_TIME_SCALE_FACTOR_1000_MS = 0x00
        PUMP_FLAGS_TIME_SCALE_FACTOR_100_MS = 0x01
        PUMP_FLAGS_TIME_SCALE_FACTOR_10_MS = 0x2
        PUMP_FLAGS_TIME_SCALE_FACTOR_1_MS = 0x3
        PUMP_FLAGS_CONTROL_MODE_BIT_POSITION = 0x4
        PUMP_FLAGS_CONTROL_MODE_BIT_MASK = 0x1
        PUMP_FLAGS_CONTROL_MODE_FLOW = 0x00
        PUMP_FLAGS_CONTROL_MODE_PRESSURE = 0x01

        def determine_time_scale_factor(time_in_seconds):
            time_in_ms = time_in_seconds * 1000
            if time_in_ms <= 0xFFFF:
                return PUMP_FLAGS_TIME_SCALE_FACTOR_1_MS, int(time_in_ms)
            
            time_in_10ms = time_in_seconds * 100
            if time_in_10ms <= 0xFFFF:
                return PUMP_FLAGS_TIME_SCALE_FACTOR_10_MS, int(time_in_10ms)
            
            time_in_100ms = time_in_seconds * 10
            if time_in_100ms <= 0xFFFF:
                return PUMP_FLAGS_TIME_SCALE_FACTOR_100_MS, int(time_in_100ms)
            
            time_in_1000ms = time_in_seconds
            if time_in_1000ms <= 0xFFFF:
                return PUMP_FLAGS_TIME_SCALE_FACTOR_1000_MS, int(time_in_1000ms)
            
            raise ValueError("Time value is too large to be represented in 16 bits with any scale factor")
                
        def encode_pump_flags(time_scale_factor, control_mode):
            flags = 0
            flags |= (time_scale_factor & PUMP_FLAGS_TIME_SCALE_FACTOR_BIT_MASK) << PUMP_FLAGS_TIME_SCALE_FACTOR_BIT_POSITION
            flags |= (control_mode & PUMP_FLAGS_CONTROL_MODE_BIT_MASK) << PUMP_FLAGS_CONTROL_MODE_BIT_POSITION
            return flags

        packet = Packet()
        
        time_scale_factor, time = determine_time_scale_factor(time_in_seconds)
        
        # Scale up values for 2 decimals of precision
        start_ratio = int(start_ratio * 100)
        end_ratio = int(end_ratio * 100)

        packet.raw[2] = (time >> 0) & 0xFF
        packet.raw[3] = (time >> 8) & 0xFF
        packet.raw[4] = (end_ratio >> 0) & 0xFF
        packet.raw[5] = (end_ratio >> 8) & 0xFF
        packet.raw[6] = (start_ratio >> 0) & 0xFF
        packet.raw[7] = (start_ratio >> 8) & 0xFF
        
        # Encode the flags
        flags = encode_pump_flags(time_scale_factor, control_mode)
        packet.raw[8] = (flags >> 0) & 0xFF

    def create_cartridge_memory_write_packet(self, cartridge_data: dict) -> bytes:
        """Create a cartridge memory write packet."""
        packet = CartridgeMemory_t()

        # Set the fields of the union according to the data in the JSON object
        for key in cartridge_data.keys():
            value = cartridge_data[key]
            if isinstance(value, str):
                value = value.encode()
            else:
                value = int(value)
            setattr(packet.data, key, value)

        # Serialize the struct into a hex encoded string
        packet_string = b"cartridge_config " + "DC".encode().hex().upper().encode()
        packet_string += bytes(packet.raw).hex().upper().encode()

        return packet_string
    
    def create_cartridge_memory_read_packet(self) -> bytes:
        """Create a cartridge memory read packet."""
        packet = Packet()

        GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE = 0x1

        # Make a packet
        packet_ID = ctypes.c_uint8(ord("C"))
        type_ID = ctypes.c_uint8(ord("A"))
        self.packet_transcoder.packetEncode_8_32(
            ctypes.byref(packet),
            packet_ID,
            type_ID,
            ctypes.c_uint8(GET_CARTRIDGE_CONFIGURATION_COMMAND_CODE),
            ctypes.c_uint32(0),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_data_acquisition_state_packet(self, state: DataAcquisitionState) -> bytes:
        """Create a data acquisition state packet."""
        # Create a blank packet
        packet = Packet()

        # Enable temperature sensor output
        self.packet_transcoder.packetEncode_16(
            ctypes.c_uint8(ord("C")),
            ctypes.c_uint8(ord("U")),
            ctypes.byref(packet),
            ctypes.c_uint16(state.value),
        )

        return PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()

    def create_cartridge_oven_set_packet(self, oven_state: bool, oven_set_point: float) -> bytes:
        """Create a packet to set the oven state and set point."""
        # Create a blank packet
        packet = Packet()

        # Encode the packet
        self.packet_transcoder.packetEncode_8_32(
            ctypes.byref(packet),
            ctypes.c_uint8(ord("C")),
            ctypes.c_uint8(ord("T")),
            ctypes.c_uint8(ord("S") if oven_state else ord("Q")),
            ctypes.c_uint32(oven_set_point * 100),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string
    
    def create_dummy_pump_status_packet(self, phase) -> bytes:
        """Create a pump status packet."""
        packet = Packet()

        # Make a packet
        self.packet_transcoder.packetEncode_8_32(
            ctypes.byref(packet),
            ctypes.c_uint8(ord("D")),
            ctypes.c_uint8(ord("V")),
            ctypes.c_uint8(phase),
            ctypes.c_uint32(0),
        )

        # Serialize the packet into a hex encoded string
        packet_string = PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()
        return packet_string

    def create_dummy_UV_data_packet(self, channel_id, time_stamp, value) -> Packet:
        """Create a mock UV data packet."""
        packet = Packet()
        packet.fields.packet_ID = ctypes.c_uint8(ord("D"))
        packet.fields.type_ID = ctypes.c_uint8(ord("U"))

        # Make a packet
        UV_data = UVData()
        UV_data.fields.channel_ID = channel_id
        UV_data.fields.time_stamp = time_stamp
        UV_data.fields.value = value

        # Copy the UV data into the packet
        ctypes.memmove(packet.fields.data, UV_data.raw, len(UV_data.raw))

        # Return the packet object
        return packet
    
    def create_dummy_cartridge_oven_status_packet(self, sequence_number, oven_state, temperature, power_output, set_point) -> str:
        """Create a cartridge oven status packet."""
        packet = Packet()

        # Make a packet
        self.packet_transcoder.packetEncode_oven_data(
            ctypes.c_uint8(ord("D")),
            ctypes.c_uint8(ord("T")),
            ctypes.byref(packet),
            ctypes.c_uint32(sequence_number),
            ctypes.c_uint8(oven_state),
            ctypes.c_uint32(temperature),
            ctypes.c_uint8(power_output),
            ctypes.c_uint16(set_point),
        )

        return PROTO_PREFIX + bytes(packet.raw).hex().upper().encode()

    def parse_values_oven_status_packet(self, packet: Packet) -> list:
        """Parse the values from the oven packet."""
        # Extract the data from the packet
        sequence_number = ctypes.c_uint32(0)
        oven_state = ctypes.c_uint8(0)
        temperature = ctypes.c_uint32(0)
        power_output = ctypes.c_uint8(0)
        set_point = ctypes.c_uint16(0)

        self.packet_transcoder.packetDecode_oven_data(
            ctypes.byref(packet),
            ctypes.byref(sequence_number),
            ctypes.byref(oven_state),
            ctypes.byref(temperature),
            ctypes.byref(power_output),
            ctypes.byref(set_point),
        )
        temperature_as_float = temperature.value / 100
        set_point = set_point.value / 100

        return [sequence_number, oven_state, temperature_as_float, power_output, set_point]

    def parse_pressure_packet(self, packet: Packet) -> list:
        """Parse the pressure value from the packet."""
        # Create a pressure info object
        pressure_info = PressureInfo()

        # Copy the packet data into the pressure info object
        ctypes.memmove(
            ctypes.byref(pressure_info.raw), ctypes.byref(packet.raw), len(packet.raw)
        )

        # Extract the pressure values
        phase = pressure_info.fields.phase.bits.phase
        stream_id = pressure_info.fields.streamId
        pressure_a = pressure_info.fields.pressureA
        pressure_b = pressure_info.fields.pressureB
        position_a = pressure_info.fields.positionA
        position_b = pressure_info.fields.positionB
        flow_rate = pressure_info.fields.flowRate

        return [
            phase,
            stream_id,
            pressure_a,
            pressure_b,
            position_a,
            position_b,
            flow_rate,
        ]
    
    def parse_UV_data_packet(self, packet: Packet) -> list:
        """Parse the UV data from the packet."""
        # Create a UV data object
        uv_data = UVData()

        # Copy the packet data into the UV data object
        ctypes.memmove(ctypes.byref(uv_data.raw), ctypes.byref(packet.fields.data), len(uv_data.raw))

        # Extract the UV data values
        channel_id = uv_data.fields.channel_ID
        time_stamp = uv_data.fields.time_stamp
        value = uv_data.fields.value

        return [channel_id, time_stamp, value]
    
    def parse_phase_packet(self, packet: Packet) -> str:
        """Parse the phase value from the packet."""
        phase_mapping: Dict[int, str] = {
            TestPhase.PhaseIdle: "Idle",
            TestPhase.PhaseRefill: "Refill",
            TestPhase.PhasePressurize: "Pressurize",
            TestPhase.PhaseEquilibrate: "Equilibrate",
            TestPhase.PhaseRun: "Run",
            TestPhase.PhaseFinalize: "Finalize",
            TestPhase.PhaseService: "Service",
            TestPhase.PhaseAbort: "Abort"
        }
        # Get the phase name from the mapping, default to "Unknown" if not found
        return phase_mapping.get(packet.fields.unitId, "Unknown")

    def parse_ack_packet(self, packet: Packet) -> list:
        """Parse the acknowledgement packet.
        
        Returns:
            A list containing the channel ID and the status of the acknowledgement.
            [channel_id, status]
        """

        # channel_id is the first byte in the data and the status is the second byte where (A)CK is success and (N)ACK is failure
        channel_id = int(chr(packet.fields.data[0]))
        status = packet.fields.data[1] == ord("A")
        return [channel_id, status]