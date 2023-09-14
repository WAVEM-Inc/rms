package net.wavem.uvc.rms.common.domain.header

data class RmsCommonHeader(
    val topicId: String?,
    val robotCorpId: String?,
    val workCorpId: String?,
    val workSiteId: String?,
    val robotId: String?,
    val robotType: String?
) {
}