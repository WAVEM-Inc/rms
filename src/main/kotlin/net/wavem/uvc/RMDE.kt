package net.wavem.uvc

import org.springframework.boot.autoconfigure.SpringBootApplication
import org.springframework.boot.context.properties.ConfigurationPropertiesScan
import org.springframework.boot.runApplication
import org.springframework.scheduling.annotation.EnableScheduling
import java.io.BufferedReader
import java.io.InputStreamReader

@EnableScheduling
@SpringBootApplication
@ConfigurationPropertiesScan
class RMDEApplication

fun main(args : Array<String>) {
    runApplication<RMDEApplication>(*args)
}