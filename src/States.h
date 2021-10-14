enum RobotStates{
    PAUSED,
    FOLLOWLINE,
    TURN,
    PICKUPPLATE,
    DROPOFFPLATE,
    CROSSLINE,
    GRIP45,
    AFTERPICKUPRESET,
    DRIVEFOR,
    EDIT,
    DRIVETOOBJECT,
    BACKUP,

};

enum HelloDarknessMyOldFriend{
    PAUSEDD,
    PICKUP45D,
    PICKUP25D,
    EDITD,
    PICKUPG,
    DROPOFF45,
    DROPOFFG,
    LINEFOLLOWD,
    CROSSOVER,

};

char *desperateEnum(RobotStates rbs){
    switch(rbs){
        case PAUSED:
            return "Paused";
            break;
        case FOLLOWLINE:
            return "FollowLine";
            break;
        case TURN:
            return "Turn";
            break;
    }
}