#ifndef MODEL_ENUMS__HXX
#define MODEL_ENUMS__HXX

namespace ktp
{
    namespace enums
    {
        enum DriveStatus
        {
            WAIT = 0,
            DRIVING_NORMALLY = 1,
            DRIVING_NORMALLY_COMPLETED = 2,
            DRIVING_CANCELED = 3,
            OBJECT_DETECTED = 4,
            DRIVING_FAILED = 5,
            EMERGENCY_STOP_BY_KTP = 6,
            TEMP_STOP_BY_KTP = 7,
            UI_STOP = 8,
            PYSHICAL_BUTTON_STOP = 9,
            ELIVATOR_ON_BOARD = 10,
            ELIVATOR_ON_QUIT = 11,
            UNABLE_TO_DRIVE_ON_ERROR = 12,
            RELEASE_STOP = 13,
            UNABLE_TO_PERFORM_MISSION = 14
        };
    }
}

#endif