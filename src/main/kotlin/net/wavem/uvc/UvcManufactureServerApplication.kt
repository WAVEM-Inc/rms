package net.wavem.uvc

import org.springframework.boot.context.properties.ConfigurationPropertiesScan
import org.springframework.boot.autoconfigure.SpringBootApplication
import org.springframework.boot.runApplication
@ConfigurationPropertiesScan
@SpringBootApplication
class UvcManufactureServerApplication

fun main(args: Array<String>) {
    runApplication<UvcManufactureServerApplication>(*args)
}