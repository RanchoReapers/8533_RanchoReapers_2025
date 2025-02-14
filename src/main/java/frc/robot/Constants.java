package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
public final class Constants {

    public final class LimelightConstants {
        public static final double KpAim = -0.1f;
        public static final double KpDistance = -0.1f;
        public static final double MinAimCommand = 0.05f;
    }

    /* READ -- Cage Claw set to 0 for Arm testing purposes & safety, 
    once you are sure that Arm works properly, set Cage Claw voltage to 0.5 volts and arm to 0,
    once sure that Cage Claw works properly, test to find proper voltages for each. */
    public final class ArmConstants {
        public static final double ArmVoltage = 0.5;
    }

    public final class CageClawConstants {
        public static final double CageClawVoltage = 0;
        public static final double CageClawTravelAngle = 30;
    }

    public final class IntakeConstants {
        public static final double IntakeVoltage = 3;
    }

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
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        // Distance between right and left wheels

        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(

                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // FRONT DRIVE MOTOR PORTS
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kFrontLeftTurningMotorPort = 5;

        // FRONT TURNING
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 6;

        // BACK DRIVE
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 2;

        // BACK TURNING
        public static final int kBackRightDriveMotorPort = 9;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = true; // was true
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kBackRightDriveReversed = true; // was true

        // SECTION - ENCODER PORTS
        // NOTE - THIS IS THE ENCODER PORTS FOR THE DRIVE, IN NUMERICAL ORDER
        public static final int kBackRightDriveAbsoluteEncoderPort = 13;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;
        // !SECTION

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        // !SECTION

        // SECTION - OFFSET VALUES

        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.409668 - (-0.45725);
        // back left ^
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.127686 - (-0.494500);
        // front right ^
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.375 - 0.165039;
        // back right ^
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.33000 - 0.236572;
        // front left ^

        // CHASSIS OFFSET REFERENCE
        public static final double kBackLeftDriveChassisOffset = 60 * Math.PI / 180;
        public static final double kFrontRightDriveChassisOffset = 120 * Math.PI / 180;
        public static final double kBackRightDriveChassisOffset = 110 * Math.PI / 180;
        public static final double kFrontLeftDriveChassisOffset = 45 * Math.PI / 180;

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
