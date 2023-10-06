package frc.robot;

public final class Constants {
    public static final double K_SPEED = 0.5; //speed for turning
    public static final double K_WHEEL_DIAMETER_INCH = 6; //diameter of the wheels in inches
    public static final double K_WHEEL_RADIUS_INCH = 3; //diameter of the wheels in inches
    public static final double K_DRIVETRAIN_GEAR_RATIO = 10.75;
    public static final double K_NEO_TICKS = 42;
    public static final double K_PI = 3.14;
    public static final double K_WHEEL_PERIMETER_INCH = K_WHEEL_DIAMETER_INCH * K_PI;
    public static final double K_WHEEL_PERIMETER_FEET = K_WHEEL_DIAMETER_INCH * K_PI / 12.0;
    public static final double K_TICKS_PER_INCH = K_NEO_TICKS / K_WHEEL_PERIMETER_INCH;
    public static final double K_TICKS_PER_FEET = K_NEO_TICKS / K_WHEEL_PERIMETER_FEET;

    
    // public static final double K_TICKS_PER_FEET = 40.964489;
    /*----------
        AUTO
    -----------*/
    public static final double K_FORWARDS_FEET = 9* K_TICKS_PER_FEET; //17.666666
    public static final double K_BACKWARDS_FEET = 6 * K_TICKS_PER_FEET; //6.572500
    public static final float K_TURN_ERROR_RANGE = 2f;
    public static final double K_DEC_TO_PI = 0.01745;

    // robot is 27 x 36 in    | 3 ft
    // Different distances are 11ft 3/8in and 16 ft 1 1/4in
    // GRID Depth 4ft 8 1/4 in
    // Grid to Community 6 ft 4 1/4 in (close) and 11 ft 4 in (far)
    

    // Place Cube Constants
    public static final double K_LEAVE_COMMUNITY_FAR = 10*12; // For Place Cube Autogroup Far
    public static final double K_LEAVE_COMMUNITY_CLOSE = 6*12; // For Place Cube Autogroup Close

    /*
     * MOVE: How many inches to move forward after raising pivot and how many back object placed
     * ANGLE: How high to raise pivot when placing the object
     */
    public static final double K_MOVE_AUTO_LOW = 36.0; 
    public static final double K_ANGLE_AUTO_LOW = 15.0;

    public static final double K_MOVE_AUTO_MID = 36.0;
    public static final double K_ANGLE_AUTO_MID = 70;

    public static final double K_MOVE_AUTO_HIGH = 36.0;
    public static final double K_ANGLE_AUTO_HIGH = 92.0;

    // Leave Community Distance For Auto
    public static final double K_LEAVE_COMMMUNITY_DIST = 4*12;


    //Turn By
    public static final float K_MIN_TURNING_SPEED = 0.2f; //minimum speed to turn at
    public static final float K_MAX_TURNING_SPEED = 0.5f; //maximum speed to turn at

    //Balancing
    public static final double K_PLAT_DEGREE_THRESH = 10; //angle at which the robot is considered to be on the platform
    public static final double K_BALANCE_THRESH_DEG = 4.2; //angle at within which the robot is considered to be balanced
    public static final double K_FWD_SPEED = 0.47; //starting speed towards platform
    public static final double K_ADJUST_SPEED = .3;  //speed to adjust angle or brake
    public static final double K_ADJUST_ROTATE = 0.1; // speed to adjust z-axis rotation incase slippage

    // Vision constants 
    public static final double K_LIMELIGHT_MOUNT_ANG_DEG = 25.0; // LIMELIGHT ANGLE FROM VERTICAL! NOT ANGLE OF ATTACK!
    public static final double K_LIMELIGHT_LENS_HEIGHT_INCH = 20.0; // HEIGHT FROM FLOOR OF LIMELIGHT



    public static final class K_ExtSub {
        public static final boolean isUsingExt = true;     //If is using the extension subsystem
        public static final boolean devMode = false; // Turn on ext dev mode
        public static final double extInchesPerSecond = 12; // Max extension speed (motor speed itself already seems to be maxed out though)
        public static final int gearRatio = 60; // Gear ratio for extension motor
        public static final double gearRadius = 14.0/16; // inches (measured from gear in contact with inner component)
        public static final double tolerance = 1; // 1 inch tolerance
    }

    public static final class K_PivotSub {
        public static final boolean isUsingPivot = true;     //If is using the pivot subsystem
        public static final boolean devMode = false; // Turn on pivot dev mode
        public static final double pivotSpeed = 3.5;
        public static final int gearRatio = 60; // Gear ratio of pivot motor
        public static final double tolerance = 5; // 5 degree tolerance before ending PivotMoveToAngleWait
    }   
    
    public static final class K_IntakeSub{
        public static final boolean isUsingIntake = true;    //If is using the claw subsystem
        public static final double calibrateStartingAngle = 90;
        public static final double calibrateEndingAngle = 180;
        public static final double calibrateAngleEncoderValue = 9.57146931;
        public static final double clampVoltage = 1.5;
        public static final double coneMaxCurrent = .3;
        public static final double cubeMaxCurrent = .2;
        public static final double InVoltage = 4.5; // Voltage at which intake sucks in
        public static final double OutVoltage = 1.7; // Voltage at which intake throws out
    }
}
