#ifndef MODEL_ENUMS__HXX
#define MODEL_ENUMS__HXX

#include <map>
#include <iostream>
#include <string>

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

        enum TaskCode
        {
            RETURNING = 0,
            MOVING = 1,
            PATROL = 2,
            TRACKING = 3,
            DELIVERING = 4
        };

        const std::map<TaskCode, std::string> TaskCodeMap = {
            {TaskCode::RETURNING, "returning"},
            {TaskCode::MOVING, "moving"},
            {TaskCode::PATROL, "patrol"},
            {TaskCode::TRACKING, "tracking"},
            {TaskCode::DELIVERING, "delivering"}
        };

        enum Status
        {
            STARTED = 0,
            SOURCE_ARRIVED = 1,
            TAKEN = 2,
            ON_PROGRESS = 3,
            DEST_ARRIVED = 4,
            END = 5,
            CANCELLED = 6,
            FAILED = 7
        };

        const std::map<Status, std::string> StatusMap = {
            {Status::STARTED, "Started"},
            {Status::SOURCE_ARRIVED, "SourceArrived"},
            {Status::TAKEN, "Taken"},
            {Status::ON_PROGRESS, "OnProgress"},
            {Status::DEST_ARRIVED, "DestArrived"},
            {Status::END, "End"},
            {Status::CANCELLED, "Cancelled"},
            {Status::FAILED, "Failed"},
        };
    }
}

#endif