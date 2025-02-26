package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Preferences;

public class ArmSubSystem extends SubsystemBase {

  SparkMax armDriveLeft;
  SparkMax armDriveRight;

  SparkMaxConfig sparkConfigDriveRight;
  SparkMaxConfig sparkConfigDriveLeft;

  RelativeEncoder RELarmEncoderLeft;
  RelativeEncoder RELarmEncoderRight;

  boolean armLow = false;
  // lockouts to prevent user switching arm state too often
  boolean armInUseUp = false; // Arm is currently being used to move upwards
  boolean armInUseDown = false; // Arm is currently being used to move downwards

  double absolutePositionArmMotorLeft;
  double absolutePositionArmMotorRight;
  // defines the deviation of motors using the relative encoder acting as an absolute encoder

  private static final String PREF_KEY_ARM_MOTOR_LEFT = "ArmMotorAbsolutePositionLeft";
  private static final String PREF_KEY_ARM_MOTOR_RIGHT = "ArmMotorAbsolutePositionRight";
  // preferences keys for saving absolute motor positions in memory

  public ArmSubSystem(int armLeftCANId, int armRightCanId) {
    armDriveLeft = new SparkMax(armLeftCANId, SparkMax.MotorType.kBrushless);
    armDriveRight = new SparkMax(armRightCanId, SparkMax.MotorType.kBrushless);

    sparkConfigDriveRight = new SparkMaxConfig();
    sparkConfigDriveLeft = new SparkMaxConfig();

    RELarmEncoderLeft = armDriveLeft.getEncoder();
    RELarmEncoderRight = armDriveRight.getEncoder();

    absolutePositionArmMotorLeft = Preferences.getDouble(PREF_KEY_ARM_MOTOR_LEFT, 0.0);
    absolutePositionArmMotorRight = Preferences.getDouble(PREF_KEY_ARM_MOTOR_RIGHT, 0.0);

    sparkConfigDriveLeft
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigDriveLeft.encoder
        .positionConversionFactor(0.003048 * Math.PI * 2)
        .velocityConversionFactor(0.003048 * Math.PI * 2);
    sparkConfigDriveLeft.smartCurrentLimit(60, 60);

    sparkConfigDriveRight
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigDriveRight.encoder
        .positionConversionFactor(0.003048 * Math.PI * 2)
        .velocityConversionFactor(0.003048 * Math.PI * 2);
    sparkConfigDriveRight.smartCurrentLimit(60, 60);

    armDriveLeft.configure(sparkConfigDriveLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armDriveRight.configure(sparkConfigDriveRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void updateArmAbsolutePosition() {
    absolutePositionArmMotorLeft += RELarmEncoderLeft.getPosition();
    absolutePositionArmMotorRight += RELarmEncoderRight.getPosition();
    
    RELarmEncoderLeft.setPosition(0.0);
    RELarmEncoderRight.setPosition(0.0);
    //reset encoders to 0 after reading
  }

  public void endArmMotors() {
    armDriveLeft.stopMotor();
    armDriveRight.stopMotor();
  }

  public void armControl2State(boolean xButton) {
    if (xButton == true) {
      armLow = !armLow;
    }
    if (absolutePositionArmMotorLeft - absolutePositionArmMotorRight <= 5 * Math.PI / 180) {
      if (armLow == true && armInUseDown == false) {
        if (absolutePositionArmMotorLeft >= 1 * Math.PI / 180) {
          armDriveLeft.setVoltage(-ArmConstants.ArmVoltage);
          armDriveRight.setVoltage(-ArmConstants.ArmVoltage);
          armInUseUp = true;
        } else {
          endArmMotors();
          armInUseUp = false;
        }
      } else if (armLow == false && armInUseUp == false) {
        if (absolutePositionArmMotorLeft <= 29 * Math.PI / 180) {
          armDriveLeft.setVoltage(ArmConstants.ArmVoltage);
          armDriveRight.setVoltage(ArmConstants.ArmVoltage);
          armInUseDown = true;
        } else {
          endArmMotors();
          armInUseDown = false;
        }
      }
    } else {
      System.out.println("Attempted to move arm but failed. Arm motors more than 5 degrees out of sync. Check your SmartDashboard data.");
      endArmMotors();
      armInUseDown = false;
      armInUseUp = false;
    }
  }

  public void saveArmAbsoluteMotorPositions() {
    Preferences.setDouble(PREF_KEY_ARM_MOTOR_LEFT, absolutePositionArmMotorLeft);
    Preferences.setDouble(PREF_KEY_ARM_MOTOR_RIGHT, absolutePositionArmMotorRight);
  }

  public void periodicOdometry() {

    updateArmAbsolutePosition();

    // post to smart dashboard periodically
    SmartDashboard.putBoolean("armLow", armLow);
    SmartDashboard.putBoolean("armInUseDown", armInUseDown);
    SmartDashboard.putBoolean("armInUseUp", armInUseUp);

    // Arm debug
    SmartDashboard.putNumber("SPARK-REL Left ARM motor position", RELarmEncoderLeft.getPosition() * 180/Math.PI);
    SmartDashboard.putNumber("SPARK-REL Right ARM motor position", RELarmEncoderRight.getPosition() * 180/Math.PI);
    SmartDashboard.putNumber("CONV-ABS Left ARM motor position", absolutePositionArmMotorLeft * 180/Math.PI);
    SmartDashboard.putNumber("CONV-ABS Right ARM motor position", absolutePositionArmMotorRight * 180/Math.PI);
  }

}