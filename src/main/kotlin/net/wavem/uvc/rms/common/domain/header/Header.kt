package net.wavem.uvc.rms.common.domain.header

data class Header(
    val robotCorpId: String?,
    val workCorpId: String?,
    val workSiteId: String?,
    val robotId: String?,
    val robotType: String?
) {
}