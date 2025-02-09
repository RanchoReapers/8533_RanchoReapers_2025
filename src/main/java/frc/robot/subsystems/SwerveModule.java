package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;



public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;
    private final SparkMaxConfig sparkConfigDrive;
    private final SparkMaxConfig sparkConfigTurn;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turnPidController;

    private final CANcoder absoluteEncoder;
    @SuppressWarnings("unused")
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final double chassisOffset = 0;

    public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean isAbsoluteEncoderReversed, double chassisOffset) {

        absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = isAbsoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        chassisOffset = this.chassisOffset;

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = absoluteEncoderOffset;

        absoluteEncoder.getConfigurator().apply(config);

        driveMotor = new SparkMax(driveMotorId, SparkMax.MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorId, SparkMax.MotorType.kBrushless);
        sparkConfigDrive = new SparkMaxConfig();
        sparkConfigTurn = new SparkMaxConfig();

        sparkConfigDrive
                .idleMode(IdleMode.kBrake)
                .inverted(false);
        sparkConfigDrive.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        sparkConfigDrive.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        sparkConfigDrive.smartCurrentLimit(60, 60);

        sparkConfigTurn
                .idleMode(IdleMode.kBrake)
                .inverted(true);
        sparkConfigTurn.encoder
                .positionConversionFactor(Math.PI * 2)
                .velocityConversionFactor(Math.PI * 2);
        sparkConfigTurn.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        sparkConfigTurn.smartCurrentLimit(60, 60);

        driveMotor.configure(sparkConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(sparkConfigDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        sparkConfigDrive.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        sparkConfigDrive.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        sparkConfigTurn.encoder.positionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        sparkConfigTurn.encoder.velocityConversionFactor(ModuleConstants.kTurnEncoderRPM2RadPerSec);
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turnEncoder.setPositionConversionFactor(ModuleConstants.kTurnEncoderRot2Rad);
        //turnEncoder.setVelocityConversionFacto r(ModuleConstants.kTurnEncoderRPM2RadPerSec);

        //turnPidController = new PIDController(0.06, 0.15, 0);
        turnPidController = new PIDController(0.05, 0.09, 0.005);
        //best ^
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
    
        resetEncoders();
    }
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        Rotation2d rot = Rotation2d.fromRadians((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
        return rot.minus(Rotation2d.fromRadians(chassisOffset)).getRadians();}

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turnEncoder.setPosition(getAbsoluteEncoderRad());
        //absoluteEncoder.setPosition(0);
    }

    public void resetTurn() {
        double position = getAbsoluteEncoderRad();
        turnEncoder.setPosition(position);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad() - chassisOffset));
    }

    public void setDesiredState(SwerveModuleState state) {
        
        SwerveModuleState correctedSwerveModuleState = state;
        correctedSwerveModuleState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedSwerveModuleState.angle = state.angle.plus(Rotation2d.fromRadians(chassisOffset));
        
        if (Math.abs(correctedSwerveModuleState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        correctedSwerveModuleState.optimize(getState().angle);
        //state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turnPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        // turnMotor.set(turnPidController.calculate(getTurningPosition(),
        // state.angle.getDegrees()));
    }

    public void stop() {
        //driveMotor.setIdleMode(IdleMode.kBrake);
        //turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                Rotation2d.fromRadians(getAbsoluteEncoderRad()));
    }

}