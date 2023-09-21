package net.wavem.uvc.rms.gateway.common

import net.wavem.uvc.rms.gateway.event.response.EventResponseHandler
import net.wavem.uvc.rms.gateway.location.response.LocationResponseHandler
import org.springframework.scheduling.annotation.Scheduled
import org.springframework.stereotype.Component

@Component
class RmsScheduler(
    private val locationResponseHandler: LocationResponseHandler,
    private val eventResponseHandler: EventResponseHandler
) {

    @Scheduled(fixedRate = HANDLE_RATE)
    fun executeLocationResponseHandler() {
        locationResponseHandler.handle()
    }

    @Scheduled(fixedRate = HANDLE_RATE)
    fun executeEventResponseHandler() {
        eventResponseHandler.handle()
    }

    private companion object {
        const val HANDLE_RATE: Long = 1000
    }
}