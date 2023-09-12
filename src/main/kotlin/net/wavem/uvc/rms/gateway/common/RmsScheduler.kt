package net.wavem.uvc.rms.gateway.common

import net.wavem.uvc.rms.gateway.location.response.LocationResponseHandler
import org.springframework.scheduling.annotation.Scheduled
import org.springframework.stereotype.Component

@Component
class RmsScheduler(
    private val locationResponseHandler: LocationResponseHandler
) {

    @Scheduled(fixedRate = 1000)
    fun executeLocationResponseHandler() {
        locationResponseHandler.handle()
    }
}