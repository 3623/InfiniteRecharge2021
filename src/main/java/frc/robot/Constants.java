/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    public static final class Drivetrain {
        public static final int RIGHT_MOTOR_MASTER = 3;
        public static final int RIGHT_MOTOR_FOLLOWER = 4;
        public static final int LEFT_MOTOR_MASTER = 1;
        public static final int LEFT_MOTOR_FOLLOWER = 2;
        public static final int FALCON_COOLER_SOLENOID = 5;
    }

    public static final class Shooter {
        public static final int SHOOTER_LEFT_MOTOR_NEO = 2;
        public static final int SHOOTER_RIGHT_MOTOR_NEO = 1;
        public static final int SHOOTER_TURRET_MOTOR_SRX = 6;
        public static final int SHOOTER_HOOD_MOTOR_SPX = 4;
        // Limelight Pipeline Constants
        public static final int CONTROL_PANEL_REAR_PIPELINE = 0;
        public static final int INITITATION_LINE_PIPELINE = 1;
        public static final int CONTROL_PANEL_FRONT_PIPELINE = 2;
        public static final int END_OF_TRENCH_PIPELINE = 3;
        public static final int POINT_BLANK_RANGE_PIPELINE = 4;
        // Limelight LED Settings Definitions
        public static final int LIMELIGHT_LED_DEFAULT = 0;
        public static final int LIMELIGHT_LED_FORCE_OFF = 1;
        public static final int LIMEIGHT_LED_BLINK = 2;
        public static final int LIMELIGHT_LED_FORCE_ON = 3;

        public static final int FEEDER_MOTOR_SRX = 7;
        public static final int SPINDEXER_MOTOR_SPX = 2;
    }

    public static final class Intake {
        public static final int INTAKE_COLLECTOR_MOTOR_SPX = 1;
        public static final int INTAKE_DROP_SOLENOID = 1;
    }

    public static final class Spinner {
        public static final int SPINNER_MOTOR_SPX = 2;
    }

    public static final class Shifter {
        public static final int SHIFTER_SOLENOID = 4;
    }

    public static final class Climber {
        public static final int ENGAGE_PTO = 2;
        public static final int EXTEND_CLIMBER = 3;
    }

    public static final class IO {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

}
