package net.wavem.uvc.rms.common.application

import org.springframework.stereotype.Service
import java.util.*

@Service
class UUIDService {

    fun generateUUID(): String {
        val uuid : UUID = UUID.randomUUID()
        val mostSigBits : String = java.lang.Long.toHexString(uuid.mostSignificantBits)
        val leastSigBits : String = java.lang.Long.toHexString(uuid.leastSignificantBits)

        val formattedMostSigBits : String = mostSigBits.padStart(16, '0')
        val formattedLeastSigBits : String = leastSigBits.padStart(16, '0')

        return "${formattedMostSigBits.substring(0, 8)}-${formattedMostSigBits.substring(8, 12)}-${formattedMostSigBits.substring(12, 16)}-${formattedLeastSigBits.substring(0, 4)}-${formattedLeastSigBits.substring(4, 16)}"
    }
}