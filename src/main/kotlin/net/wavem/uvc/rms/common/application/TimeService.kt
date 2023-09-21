package net.wavem.uvc.rms.common.application

import org.springframework.stereotype.Service
import java.text.SimpleDateFormat
import java.util.*

@Service
class TimeService {

    fun getCurrentDateTime(): String {
        val sdf: SimpleDateFormat = SimpleDateFormat("yyMMddHHmmss")
        val currentDate: Date = Date()
        return sdf.format(currentDate)
    }
}