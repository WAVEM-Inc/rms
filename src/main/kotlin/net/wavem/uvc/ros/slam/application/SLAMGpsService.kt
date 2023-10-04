package net.wavem.uvc.ros.slam.application

import org.locationtech.proj4j.CRSFactory
import org.springframework.stereotype.Service

@Service
class SLAMGpsService {

    private val crsFactory : CRSFactory = CRSFactory()

}