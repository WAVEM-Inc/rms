package net.wavem.uvc.rms.domain.job.types

enum class JobResultType(
    val type: String
) {
    SUCCESS("success"),
    FAIL("fail")
}