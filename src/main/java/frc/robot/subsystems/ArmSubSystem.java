package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubSystem extends SubsystemBase {

  SparkMax armDriveLeft;
  SparkMax armDriveRight;

  SparkMaxConfig sparkConfigDriveRight;
  SparkMaxConfig sparkConfigDriveLeft;

  AbsoluteEncoder armEncoderLeft;
  AbsoluteEncoder armEncoderRight;

  boolean armLow = false;
  // lockouts to prevent user switching arm state too often
  boolean armInUseUp = true; // Arm is currently being used to move upwards
  boolean armInUseDown = false; // Arm is currently being used to move downwards

  public ArmSubSystem(int armLeftCANId, int armRightCanId) {
    armDriveLeft = new SparkMax(armLeftCANId, SparkMax.MotorType.kBrushless);
    armDriveRight = new SparkMax(armRightCanId, SparkMax.MotorType.kBrushless);

    sparkConfigDriveRight = new SparkMaxConfig();
    sparkConfigDriveLeft = new SparkMaxConfig();

    armEncoderLeft = armDriveLeft.getAbsoluteEncoder();
    armEncoderRight = armDriveRight.getAbsoluteEncoder();

    sparkConfigDriveLeft
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigDriveLeft.encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor(Math.PI * 2);
    sparkConfigDriveLeft.smartCurrentLimit(60, 60);

    sparkConfigDriveRight
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigDriveRight.encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor(Math.PI * 2);
    sparkConfigDriveRight.smartCurrentLimit(60, 60);

    armDriveLeft.configure(sparkConfigDriveLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armDriveRight.configure(sparkConfigDriveRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void endArmMotors() {
    armDriveLeft.stopMotor();
    armDriveRight.stopMotor();
  }

  public void armControl2State(boolean xButton) {
    if (xButton == true) {
      armLow = !armLow;
    }
    if (armEncoderLeft.getPosition() - armEncoderRight.getPosition() <= 5 * Math.PI / 180) {
      if (armLow == true && armInUseDown == false) {
        if (armEncoderLeft.getPosition() >= 30 * Math.PI / 180) {
          armDriveLeft.setVoltage(-ArmConstants.ArmVoltage);
          armDriveRight.setVoltage(-ArmConstants.ArmVoltage);
          armInUseUp = true;
        } else {
          endArmMotors();
          armInUseUp = false;
        }
      } else if (armLow == false && armInUseUp == false) {
        if (armEncoderLeft.getPosition() <= 60 * Math.PI / 180) {
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

  public void periodicOdometry() {

    // post to smart dashboard periodically
    SmartDashboard.putBoolean("armLow", armLow);
    SmartDashboard.putBoolean("armInUseDown", armInUseDown);
    SmartDashboard.putBoolean("armInUseUp", armInUseUp);

  }
}