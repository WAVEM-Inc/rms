package net.wavem.uvc.rms.domain.header

data class Header(
    val topicId: String,
    val robotCorpId: String,
    val workCorpId: String,
    val workSiteId: String,
    val robotId: String,
    val robotType: String
) {
}