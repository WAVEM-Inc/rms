package net.wavem.uvc.rms.domain

data class RmsHeader(
    val topicId: String,
    val robotCorpId: String,
    val workCorpId: String,
    val workSiteId: String,
    val robotId: String,
    val robotType: String
) {
}