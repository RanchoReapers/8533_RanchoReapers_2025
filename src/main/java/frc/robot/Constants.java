
package frc.robot;

//import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ANCHOR - THIS PART OF THE CODE IS IMPORTANT AS IT MAY NEED TO CHANGE INTEGERS
    // IF YOU CHANGE CONTROLLERS, WE ARE USING AN XBOX AS OF TESTING
    public final class USB {
        public static final int DRIVER_CONTROLLER = 0; // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 1; // Operator controller USB ID
        public static final int OPERATOR_LY = 1;
        public static final int OPERATOR_LX = 0;
        public static final int OPERATOR_RY = 5;
        public static final int OPERATOR_RX = 4;
        public static final int OPERATOR_RT = 3;
        public static final int OPERATOR_LT = 2;
    }

    // SECTION - IGNORE FOR NOW
    // NOTE - IGNORE THIS CODE FOR NOW UNLESS NEEDED FOR THE FUTURE!
    /*
     * 
     * public final class WRIST{
     * public static final int ID = 12;
     * public static final double MAX_SPEED_UP = 0.3;
     * public static final double MAX_SPEED_DOWN = 0.2;
     * public static final double POSITION_TOLERANCE = 0.05;
     * public static final double UP_POSITION = -9;
     * public static final double DOWN_POSITION = 0;
     * 
     * public static final double kP = 0;
     * public static final double kI = 0;
     * public static final double kD = 0;
     * }
     * 
     * public final class ELBOW{
     * public static final int ID = 11;
     * public static final boolean INVERTED = false;
     * public static final double MAX_SPEED = 0.2;
     * public static final double POSITION_TOLERANCE = 0.05;
     * public static final double UP_POSITION = -9;
     * public static final double DOWN_POSITION = 0;
     * 
     * public static final double kP = 0;
     * public static final double kI = 0;
     * public static final double kD = 0;
     * }
     * 
     * public final class SHOULDER{
     * public static final int LEFT = 9;
     * public static final int RIGHT = 10;
     * public static final boolean LEFT_INVERTED = false;
     * public static final boolean RIGHT_INVERTED = true;
     * public static final double MAX_SPEED = 0.2;
     * 
     * public static final double kP = 0.09;
     * public static final double kI = 0.1;
     * public static final double kD = 0.03;
     * }
     */
    // !SECTION
    public final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10;
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.42857142857143;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRPM2RadPerSec = kTurnEncoderRot2Rad / 60;
        public static final double kPTurning = 0.6;
    }

    public final class Sensors {
        public static final int GYRO_ID = 16;
    }

    public static final class DriveConstants {
        // SECTION - BASE WXH
        // NOTE - THIS PART OF THE CODE CAN CHANGE DEPENDING ON IF THE BASE IS CHANGED!
        public static final double kTrackWidth = Units.inchesToMeters(20.5); // was 18.5
        // Distance between right and left wheels

        public static final double kWheelBase = Units.inchesToMeters(20.5); // was 30.75
        // Distance between front and back wheels
        // !SECTION
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                // REVIEW - THIS IS THE NEW VERSION OF THE CODE, COMMENT THIS OUT IF REVERTING
                // TO THE OLD ONE
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // REVIEW - THIS IS THE OLD VERSION OF THE CODE, COMMENT THIS OUT IF REVERTING
        // TO THE NEW ONE
        // new Translation2d(kTrackWidth / 2, kWheelBase / 2),
        // new Translation2d(kTrackWidth / 2, -kWheelBase / 2),
        // new Translation2d(-kTrackWidth / 2, kWheelBase / 2),
        // new Translation2d(-kTrackWidth / 2, -kWheelBase / 2));
        // SECTION - Motor ports
        // NOTE - DO NOT CHANGE! INTEGERS ARE THE IMPORTS FOR EACH MOTOR
        // NOTE - THIS IS THE FRONT DRIVE MOTOR PORTS
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kFrontLeftTurningMotorPort = 5;

        // NOTE - THIS IS THE FRONT TURNING MOTOR PORTS
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 6;

        // NOTE - THIS IS THE BACK DRIVE MOTOR PORTS
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 2;

        // NOTE - THIS IS THE BACK TURNING MOTOR PORTS
        public static final int kBackRightDriveMotorPort = 9;
        public static final int kBackRightTurningMotorPort = 8;
        // !SECTION

        // SECTION - Turning & Drive reverse values
        // NOTE - THIS IS THE FRONT TURNING ENCODER REVERSE VALUES
        // REVIEW - WE MAY NEED TO ALTER SOME CODE, MAKE SURE TO REVERT BACK TO THE
        // OTHER VALUES IF CHANGED AND IT GETS WORSE!
        // public static final boolean kFrontLeftTurningEncoderReversed = true;
        // public static final boolean kFrontRightTurningEncoderReversed = true;

        // //NOTE - THIS IS THE BACK TURNING ENCODER REVERSE VALUES
        // public static final boolean kBackLeftTurningEncoderReversed = true;
        // public static final boolean kBackRightTurningEncoderReversed = true;

        // DELETE THIS IF NO WORK UNCOMMENT WORK ABOVE
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        // NOTE - THIS IS THE FRONT DRIVE REVERSE VALUES
        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = true; // was true
        // NOTE - THIS IS THE BACK DRIVE REVERSE VALUES
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kBackRightDriveReversed = true; // was true
        // !SECTION

        // SECTION - ENCODER PORTS
        // NOTE - THIS IS THE ENCODER PORTS FOR THE DRIVE, IN NUMERICAL ORDER
        public static final int kBackRightDriveAbsoluteEncoderPort = 13;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        // !SECTION

        // SECTION - ABS Drive Encoder Reverse values
        // NOTE - THIS IS THE FRONT ABSOLUTE DRIVE ENCODER REVERSED
        // public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        // public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;

        // //NOTE - THIS IS THE BACK ABSOLUTE DRIVE ENCODER REVERSED
        // public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        // public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // IF NO WORK DELETE AND UNCOMMENT CODE ABOVE
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        // !SECTION

        // SECTION - OFFSET VALUES
        // NOTE - THESE OFFSET VALUES ALL SEEM TO WORK, DON'T CHANGE UNLESS TOLD TO!

        /*
         * public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.150635
         * * Math.PI * 2;
         * //back left ^
         * public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =
         * -0.376221 * Math.PI * 2;
         * //front right ^
         * public static final double kBackRightDriveAbsoluteEncoderOffsetRad =
         * -0.455566 * Math.PI * 2;
         * // back right ^
         * public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =
         * -0.449951 * Math.PI * 2;
         * // front left ^
         * //!SECTION
         * 
         * // uncomment if this does not fix the motors
         */

        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI * 2;
        // back left ^
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI * 2;
        // front right ^
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI * 2;
        // back right ^
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI * 2;
        // front left ^

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double kSlowButtonDriveModifier = 0.25;
        public static final double kSlowButtonTurnModifier = 0.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond
                / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

}
