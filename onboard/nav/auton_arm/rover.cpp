#include "rover.hpp"

Rover::RoverStatus::RoverStatus() : mCurrentState(AutonArmState::Off) {
    mAutonState.is_auton = false;
}

AutonArmState& Rover::RoverStatus::currentState() {
    return mCurrentState;
} //currentState()

AutonState& Rover::RoverStatus::autonState() {
    return mAutonState;
} //autonState()

Target& Rover::RoverStatus::target() {
    return mTarget;
} //target()

Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcmObject) : mRoverConfig(config), mLcmObject(lcmObject) {}

bool Rover::updateRover(RoverStatus newRoverStatus) {
    // Rover currently on.
    if( mRoverStatus.autonState().is_auton )
    {
        // Rover turned off
        if( !newRoverStatus.autonState().is_auton )
        {
            mRoverStatus.autonState() = newRoverStatus.autonState();
            return true;
        }

        // If any data has changed, update all data
        if(!isEqual( mRoverStatus.target(), newRoverStatus.target()))
        {
            mRoverStatus.target() = newRoverStatus.target();
            return true;
        }

        return false;
    }

    // Rover currently off.
    else
    {
        // Rover turned on.
        if( newRoverStatus.autonState().is_auton )
        {
            mRoverStatus = newRoverStatus;
            return true;
        }
        return false;
    }
    return false;
} //updateRover(newRoverStatus)

Rover::RoverStatus& Rover::roverStatus(){
    return mRoverStatus;
}

bool Rover::isEqual( const Target& target1, const Target& target2 ) const
{
    return (target1.distance == target2.distance && target1.bearing == target2.bearing);
} // isEqual( Target )