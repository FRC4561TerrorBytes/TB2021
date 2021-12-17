/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Show Debug
    public static final boolean DRIVE_DEBUG = false;
    public static final boolean SHOOTER_DEBUG = false;
    public static final boolean MAGAZINE_DEBUG = false;
    public static final boolean CLIMBER_DEBUG = false;
    
    // Robot tick rate in seconds
    public static final double ROBOT_LOOP_PERIOD = 1.0 / 60.0;

    // Drive PID values
    public static final double DRIVE_kP = 0.018; //0.018
    public static final double DRIVE_kD = 0.00029; //0.00029
    public static final double DRIVE_TURN_SCALAR = 30.0;
    public static final double DRIVE_ACCELERATION_LIMIT = 0.5;
    public static final String DRIVE_TRACTION_CONTROL_CURVE = "X / 4.106";
    public static final String DRIVE_THROTTLE_INPUT_CURVE = "4.106 * X";
    public static final int DRIVE_RESPONSE_EXPONENT = 1;

    // Auto drive config values
    public static final double ANGLE_TOLERANCE = 15.0; // Degrees
    public static final double TARGET_DISTANCE = 0.0; // Inches
    public static final double AUTO_STOPPING_DISTANCE = 0.0; // Inches
    public static final double AUTO_SPEED = 0.0; // [0, 1]
    public static final double TURN_DAMPER = 0.1; // [0, 1]
		public static final double TURRET_TURN_DAMPER = 0.15; // [0, 1]

    // Motor RPMs, encoder values, and gear ratios
    public static final int FALCON_500_MAX_RPM = 6380;
    public static final int BAG_MAX_RPM = 13180;
    public static final int CTRE_MAG_ENCODER_TICKS_PER_ROTATION = 4096;
    public static final int CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION = 2048;

    public static final double TURRET_GEAR_RATIO_BEFORE_ENCODER = 2.0 / 1.0;
    public static final double TURRET_GEAR_RATIO_AFTER_ENCODER = 94.0 / 15.0;
    public static final double HOOD_GEAR_RATIO_BEFORE_ENCODER = 298.0 / 25.0;
    public static final double HOOD_GEAR_RATIO_AFTER_ENCODER = 1.0 / 1.0;

    // Intake Arm PID config
    private static final double ARM_kP = 12.0;
    private static final double ARM_kD = 4.0;
    private static final double ARM_TOLERANCE = 10;
    private static final double ARM_LOWER_LIMIT = -1500;
    private static final double ARM_UPPER_LIMIT = 0;
    private static final double ARM_VELOCITY = 1;
    private static final double ARM_ACCLERATION = 0.1;
    private static final int ARM_MOTION_SMOOTHING = 7;
    private static final int ARM_TICKS_PER_ROTATION = CTRE_MAG_ENCODER_TICKS_PER_ROTATION;
    private static final int ARM_MAX_RPM = 0;
    private static final boolean ARM_SOFT_LIMITS = true;
    private static final boolean ARM_SENSOR_PHASE = true;
    private static final boolean ARM_INVERT_MOTOR = false;

    // Intake Arm Positions
    public static final int ARM_TOP_POSITION = -1450;
    public static final int ARM_BOTTOM_POSITION = 0;
    public static final double ARM_MANUAL_INCREMENT = 75;


    // Shooter PID Values
    private static final double FLYWHEEL_kP = 1.15;
    private static final double FLYWHEEL_kI = 0.98;
    private static final double FLYWHEEL_kD = 0.11;
    private static final double FLYWHEEL_TOLERANCE = 100;
    private static final double FLYWHEEL_MAX_RPM = FALCON_500_MAX_RPM;
    private static final double FLYWHEEL_TICKS_PER_ROTATION = CTRE_TALONFX_ENCODER_TICKS_PER_ROTATION;
    private static final boolean FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE = false;
    private static final boolean FLYWHEEL_MASTER_MOTOR_INVERTED = false;

    private static final double HOOD_kP = 1;
    private static final double HOOD_kD = 0.01;
    private static final double HOOD_TOLERANCE = 5.0;
    private static final boolean HOOD_SOFT_LIMITS = false;
    private static final double HOOD_VELOCITY = 100 * HOOD_GEAR_RATIO_AFTER_ENCODER;
    private static final double HOOD_ACCELERATION = 50 * HOOD_GEAR_RATIO_AFTER_ENCODER;
    private static final int HOOD_MOTION_SMOOTHING = 3;
    private static final double HOOD_TICKS_PER_ROTATION = CTRE_MAG_ENCODER_TICKS_PER_ROTATION;
    private static final double HOOD_MAX_RPM = BAG_MAX_RPM * HOOD_GEAR_RATIO_BEFORE_ENCODER;
    private static final boolean HOOD_ENCODER_SENSOR_PHASE = false;
    private static final boolean HOOD_MOTOR_INVERTED = false;
    public static final int HOOD_BOTTOM_POSITION = 0;
    public static final int HOOD_TOP_POSITION = -4770;

    private static final double TURRET_kP = 0.75;
    private static final double TURRET_kD = 0.02;
    private static final double TURRET_TOLERANCE = 80;
    private static final boolean TURRET_SOFT_LIMITS = true;
    private static final double TURRET_VELOCITY = 10 * TURRET_GEAR_RATIO_AFTER_ENCODER;
    private static final double TURRET_ACCELERATION = 2 * TURRET_GEAR_RATIO_AFTER_ENCODER;
    private static final int TURRET_MOTION_SMOOTHING = 4;
    private static final double TURRET_TICKS_PER_ROTATION = CTRE_MAG_ENCODER_TICKS_PER_ROTATION;
    private static final double TURRET_MAX_RPM = BAG_MAX_RPM / TURRET_GEAR_RATIO_BEFORE_ENCODER;
    private static final boolean TURRET_ENCODER_SENSOR_PHASE = false;
    private static final boolean TURRET_MOTOR_INVERTED = false;
    
    // Shooter Positions
    public static final int TURRET_FRONT_LIMIT_POSITION = 0;
    public static final int TURRET_STRAIGHT_POSITION = 3300;
    public static final int TURRET_MIDDLE_POSITION = 9700;
    public static final int TURRET_SIXBALL_POSITION = 13000;
    public static final int TURRET_BACK_POSITION = 18350;
    public static final int TURRET_BACK_LIMIT_POSITION = 22000;

    // Set PID for Flywheel
    public static final TalonPIDConfig FLYWHEEL_MASTER_CONFIG = new TalonPIDConfig(FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE,
                                                                                    FLYWHEEL_MASTER_MOTOR_INVERTED,
                                                                                    FLYWHEEL_MAX_RPM,
                                                                                    FLYWHEEL_TICKS_PER_ROTATION,
                                                                                    FLYWHEEL_kP,
                                                                                    FLYWHEEL_kI,
                                                                                    FLYWHEEL_kD,
                                                                                    FLYWHEEL_TOLERANCE);
                                   
    // Set PID for Hood
    public static final TalonPIDConfig HOOD_CONFIG = new TalonPIDConfig(HOOD_ENCODER_SENSOR_PHASE,
                                                                        HOOD_MOTOR_INVERTED,
                                                                        HOOD_TICKS_PER_ROTATION,
                                                                        HOOD_MAX_RPM,
                                                                        HOOD_kP,
                                                                        0.0,
                                                                        HOOD_kD,
                                                                        HOOD_TOLERANCE,
                                                                        HOOD_TOP_POSITION,
                                                                        HOOD_BOTTOM_POSITION,
                                                                        HOOD_SOFT_LIMITS,
                                                                        HOOD_VELOCITY,
                                                                        HOOD_ACCELERATION,
                                                                        HOOD_MOTION_SMOOTHING);

    // Set PID for Turret
    public static final TalonPIDConfig TURRET_CONFIG = new TalonPIDConfig(TURRET_ENCODER_SENSOR_PHASE,
                                                                            TURRET_MOTOR_INVERTED,
                                                                            TURRET_TICKS_PER_ROTATION,
                                                                            TURRET_MAX_RPM,
                                                                            TURRET_kP,
                                                                            0.0,
                                                                            TURRET_kD,
                                                                            TURRET_TOLERANCE,
                                                                            TURRET_FRONT_LIMIT_POSITION,
                                                                            TURRET_BACK_LIMIT_POSITION,
                                                                            TURRET_SOFT_LIMITS,
                                                                            TURRET_VELOCITY,
                                                                            TURRET_ACCELERATION,
                                                                            TURRET_MOTION_SMOOTHING);
    // Mouse Droid PID values
    public static final double MOUSE_kP = 0.0;
    public static final double MOUSE_kD = 0.0;

    // Arm Config
    public static final TalonPIDConfig ARM_CONFIG = new TalonPIDConfig(ARM_SENSOR_PHASE,
                                                                        ARM_INVERT_MOTOR,
                                                                        ARM_TICKS_PER_ROTATION,
                                                                        ARM_MAX_RPM,
                                                                        ARM_kP,
                                                                        0.0,
                                                                        ARM_kD,
                                                                        ARM_TOLERANCE,
                                                                        ARM_LOWER_LIMIT,
                                                                        ARM_UPPER_LIMIT,
                                                                        ARM_SOFT_LIMITS,
                                                                        ARM_VELOCITY,
                                                                        ARM_ACCLERATION,
                                                                        ARM_MOTION_SMOOTHING);

    // Analog stick deadband value
    public static final double DEADBAND = 0.009;
    
    // Joystick Ports
    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;

    // Xbox controller
    public static final int XBOX_CONTROLLER_PORT = 2;    

    // Drive Motor Ports 
    public static final int FRONT_LEFT_MOTOR_PORT = 0;
    public static final int REAR_LEFT_MOTOR_PORT = 1;

    public static final int FRONT_RIGHT_MOTOR_PORT = 2;
    public static final int REAR_RIGHT_MOTOR_PORT = 3;

    // Climber Motor Ports
    public static final int CLIMBER_LIFT_MOTOR_PORT = 6;
    public static final int CLIMBER_HOOK_MOTOR_PORT = 11;
    public static final int CLIMBER_BALANCE_MOTOR_PORT = 7;

    // Climber Movement Constants
    public static final double CLIMBER_LIFT_CONSTANT = 0.5;
    public static final double CLIMBER_HOOK_CONSTANT = 0.69;

    // Speed limiting [0.0, 1.0]
    public static final double CLIMBER_SPEED_LIMIT = 1.0; 

    //Magazine Motor Port
    public static final int MAGAZINE_MOTOR_PORT = 10;

    //Intake Motor Port
    public static final int INTAKE_MOTOR_PORT = 13;
    public static final int ARM_MOTOR_PORT = 12;

    //Magazine proximity sensor ports
    public static final int MAGAZINE_ULTRASONIC_BOT = 0;
    public static final int MAGAZINE_ULTRASONIC_TOP = 1;

    // Shooter Motor Ports
    public static final int FLYWHEEL_MASTER_MOTOR_PORT = 5;
    public static final int FLYWHEEL_SLAVE_MOTOR_PORT = 4;
    public static final int HOOD_MOTOR_PORT = 9;
    public static final int TURRET_MOTOR_PORT = 8;

    // PWM Ports
    public static final int LIDAR_PORT = 6;

    // Digital ports
    public static final int LED_RING_PORT = 7;

    // Motor Speeds - @author utkarsh
    public static final double MOTOR_STOP = 0;
    public static final double INTAKE_MOTOR_SPEED = 0.75; // 0.75
    public static final double OUTTAKE_MOTOR_SPEED = 1;
    public static final double LIFT_MOTOR_SPEED = 1.0;
    public static final int UPTAKE_MOTOR_DELAY = 500; //in milliseconds
    public static final double MAGAZINE_UP_MOTOR_SPEED = 0.75;
    public static final double MAGAZINE_DOWN_MOTOR_SPEED = 0.2;
    public static final double MOUSE_DROID_SPEED = .5;

    public static boolean VISION_ENABLE = true;
}
