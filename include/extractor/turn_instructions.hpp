#ifndef TURN_INSTRUCTIONS_HPP
#define TURN_INSTRUCTIONS_HPP

namespace osrm
{
namespace extractor
{

enum class TurnInstruction : unsigned char
{
    None = 0,
    Continue,
    BearRight,
    TurnRight,
    SharpRight,
    UTurn,
    SharpLeft,
    TurnLeft,
    BearLeft,
    ReachedWaypointLocation,
    EnterRoundAbout,
    LeaveRoundAbout,
    StayOnRoundAbout,
    Depart,
    Arrive
};

// Translates between angles and their human-friendly directional representation
//               180°
//               (w)
//          o     ^     o
//        o       ^       o
//                ^
// 270°  o       (v)       o  90°
//                ^
//        o       ^       o
//          o     ^     o
//               (u)
//                0°
inline TurnInstruction getTurnDirection(const double angle)
{
    if (angle >= 23 && angle < 67)
    {
        return TurnInstruction::SharpRight;
    }
    if (angle >= 67 && angle < 113)
    {
        return TurnInstruction::TurnRight;
    }
    if (angle >= 113 && angle < 158)
    {
        return TurnInstruction::BearRight;
    }
    if (angle >= 158 && angle < 202)
    {
        return TurnInstruction::Continue;
    }
    if (angle >= 202 && angle < 248)
    {
        return TurnInstruction::BearLeft;
    }
    if (angle >= 248 && angle < 292)
    {
        return TurnInstruction::TurnLeft;
    }
    if (angle >= 292 && angle < 336)
    {
        return TurnInstruction::SharpLeft;
    }
    return TurnInstruction::UTurn;
}

}
}

#endif /* TURN_INSTRUCTIONS_HPP */
