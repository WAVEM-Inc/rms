package net.wavem.uvc.ros.domain.sensor_msgs

import id.jrosmessages.Message
import id.xfunction.XJson
import net.wavem.uvc.ros.domain.std_msgs.Header
import net.wavem.uvc.ros.domain.builtin_interfaces.Time
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.floatArrayOf

class BatteryState() : Message {

    var header : Header = Header()
    var voltage : Float = 0.0f
    var temperature : Float = 0.0f
    var current : Float = 0.0f
    var charge : Float = 0.0f
    var capacity : Float = 0.0f
    var design_capacity : Float = 0.0f
    var percentage : Float = 0.0f
    var power_supply_status : UByte = 0u
    var power_supply_health : UByte = 0u
    var power_supply_technology : UByte = 0u
    var present : Boolean = false
    var cell_voltage : FloatArray = floatArrayOf()
    var cell_temperature : FloatArray = floatArrayOf()
    var location : String = ""
    var serial_number : String = ""

    constructor(
        header : Header,
        voltage : Float,
        temperature : Float,
        current : Float,
        charge : Float,
        capacity : Float,
        design_capacity : Float,
        percentage : Float,
        power_supply_status : UByte,
        power_supply_health : UByte,
        power_supply_technology : UByte,
        present : Boolean,
        cell_voltage : FloatArray,
        cell_temperature : FloatArray,
        location : String,
        serial_number : String
    ) : this() {
        this.header = header
        this.voltage = voltage
        this.temperature = temperature
        this.current = current
        this.charge = charge
        this.capacity = capacity
        this.design_capacity = design_capacity
        this.percentage = percentage
        this.power_supply_status = power_supply_status
        this.power_supply_health = power_supply_health
        this.power_supply_technology = power_supply_technology
        this.present = present
        this.cell_voltage = cell_voltage
        this.cell_temperature = cell_temperature
        this.location = location
        this.serial_number = serial_number
    }

    override fun toString() : String {
        return XJson.asString(
            "header", this.header.toString(),
            "volatge", this.voltage,
            "temperature", this.temperature,
            "current", this.current,
            "charge", this.charge,
            "capacity", this.capacity,
            "design_capacity", this.design_capacity,
            "percentage", this.percentage,
            "power_supply_status", this.power_supply_status,
            "power_supply_health", this.power_supply_health,
            "power_supply_technology", this.power_supply_technology,
            "present", this.present,
            "cell_voltage", this.cell_voltage,
            "cell_temperature", this.cell_temperature,
            "location", this.location,
            "serial_number", this.serial_number
        )
    }

    companion object {
        const val POWER_SUPPLY_STATUS_UNKNOWN : UByte = 0u
        const val POWER_SUPPLY_STATUS_CHARGING : UByte = 1u
        const val POWER_SUPPLY_STATUS_DISCHARGING : UByte = 2u
        const val POWER_SUPPLY_STATUS_NOT_CHARGING : UByte = 3u
        const val POWER_SUPPLY_STATUS_FULL : UByte = 4u

        const val POWER_SUPPLY_HEALTH_UNKNOWN : UByte = 0u
        const val POWER_SUPPLY_HEALTH_GOOD : UByte = 1u
        const val POWER_SUPPLY_HEALTH_OVERHEAT : UByte = 2u
        const val POWER_SUPPLY_HEALTH_DEAD : UByte = 3u
        const val POWER_SUPPLY_HEALTH_OVERVOLTAGE : UByte = 4u
        const val POWER_SUPPLY_HEALTH_UNSPEC_FAILURE : UByte = 5u
        const val POWER_SUPPLY_HEALTH_COLD : UByte = 6u
        const val POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE : UByte = 7u
        const val POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE : UByte = 8u

        const val POWER_SUPPLY_TECHNOLOGY_UNKNOWN : UByte = 0u
        const val POWER_SUPPLY_TECHNOLOGY_NIMH : UByte = 1u
        const val POWER_SUPPLY_TECHNOLOGY_LION : UByte = 2u
        const val POWER_SUPPLY_TECHNOLOGY_LIPO : UByte = 3u
        const val POWER_SUPPLY_TECHNOLOGY_LIFE : UByte = 4u
        const val POWER_SUPPLY_TECHNOLOGY_NICD : UByte = 5u
        const val POWER_SUPPLY_TECHNOLOGY_LIMN : UByte = 6u

        fun read(data : ByteArray) : BatteryState {
            val buf : ByteBuffer = ByteBuffer.wrap(data)
            buf.order(ByteOrder.LITTLE_ENDIAN)

            val header : Header = Header.read(data)

            val headerBufferSize : Int = Header.getBufferSize()

            buf.position(headerBufferSize - 4)

            val voltage : Float = buf.getFloat()

            val temperature : Float = buf.getFloat()
            
            val current : Float = buf.getFloat()
            
            val charge : Float = buf.getFloat()

            val capacity : Float = buf.getFloat()

            val design_capacity : Float = buf.getFloat()

            val percentage : Float = buf.getFloat()

            val power_supply_status : UByte = buf.get().toUByte()

            val power_supply_health : UByte = buf.get().toUByte()

            val power_supply_technology : UByte = buf.get().toUByte()

            val present : Boolean = buf.get() != 0.toByte()

            val cell_voltage_size : Int = buf.getInt()

            val cell_voltage : FloatArray = FloatArray(cell_voltage_size)
            for (i in 0 .. cell_voltage_size - 1) {
                val voltages : Float = buf.getFloat()
                cell_voltage[i] = voltages
            }

            val cell_temperature_size : Int = buf.getInt()

            val cell_temperature : FloatArray = FloatArray(cell_temperature_size)
            for (i in 0 .. cell_temperature_size - 1) {
                val temperatures : Float = buf.getFloat()
                cell_temperature[i] = temperatures
            }

            var location_len : Int = buf.getInt()
            var location : String = ""
            while(location_len-- > 0) location += Char(buf.get().toUShort())
            
            buf.getShort()

            var serial_number_len : Int = buf.getInt()
            var serial_number : String = ""
            while(serial_number_len-- > 0) serial_number += Char(buf.get().toUShort())

            return BatteryState(
                header = header,
                voltage = voltage,
                temperature = temperature,
                current = current,
                charge = charge,
                capacity = capacity,
                design_capacity = design_capacity,
                percentage = percentage,
                power_supply_status = power_supply_status,
                power_supply_health = power_supply_health,
                power_supply_technology = power_supply_technology,
                present = present,
                cell_voltage = cell_voltage,
                cell_temperature = cell_temperature,
                location = location,
                serial_number = serial_number
            )
        }
    }
}