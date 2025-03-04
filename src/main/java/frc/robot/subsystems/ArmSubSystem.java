package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ArmConstants;
import frc.robot.util.Elastic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubSystem extends SubsystemBase {

  SparkMax armDriveLeft;
  SparkMax armDriveRight;

  SparkMaxConfig sparkConfigDriveRight;
  SparkMaxConfig sparkConfigDriveLeft;

  RelativeEncoder armEncoderLeft;
  RelativeEncoder armEncoderRight;

  boolean armLow = false; // where you are TRYING to go
  // lockouts to prevent user switching arm state too often
  boolean armInUseUp = false; // Arm is currently being used to move upwards
  boolean armInUseDown = false; // Arm is currently being used to move downwards

  Elastic.Notification armSyncError = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Arm Motors Out Of Sync", "Attempted to move arm but failed. Arm motors more than 5 degrees out of sync.");

  public ArmSubSystem(int armLeftCANId, int armRightCanId) {
    armDriveLeft = new SparkMax(armLeftCANId, SparkMax.MotorType.kBrushless);
    armDriveRight = new SparkMax(armRightCanId, SparkMax.MotorType.kBrushless);

    sparkConfigDriveRight = new SparkMaxConfig();
    sparkConfigDriveLeft = new SparkMaxConfig();

    armEncoderLeft = armDriveLeft.getEncoder();
    armEncoderRight = armDriveRight.getEncoder();

    sparkConfigDriveLeft
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigDriveLeft.encoder
        .positionConversionFactor(0.021 * Math.PI * 2)
        .velocityConversionFactor(0.021 * Math.PI * 2);
    sparkConfigDriveLeft.smartCurrentLimit(60, 60);

    sparkConfigDriveRight
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigDriveRight.encoder
        .positionConversionFactor(0.021 * Math.PI * 2)
        .velocityConversionFactor(0.021 * Math.PI * 2);
    sparkConfigDriveRight.smartCurrentLimit(60, 60);

    armDriveLeft.configure(sparkConfigDriveLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    armDriveRight.configure(sparkConfigDriveRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void endArmMotors() {
    armDriveLeft.stopMotor();
    armDriveRight.stopMotor();
  }


  public void switchArmLow() {
    if (armInUseDown == false && armInUseUp == false) {
      armLow = !armLow;
    }
  }
  
  public void armControl2State() {
    // set the number of degrees to be one lower/higher depending on direction for movement to allow for stopping time
    if (Math.abs(armEncoderLeft.getPosition()) - Math.abs(armEncoderRight.getPosition()) <= 5 * Math.PI / 180) {
      if (armLow == true && armInUseDown == false) {
        if (armEncoderLeft.getPosition() >= -90 * Math.PI / 180) { // higher
          armInUseUp = true;
          armDriveLeft.setVoltage(-ArmConstants.ArmVoltage);
          armDriveRight.setVoltage(-ArmConstants.ArmVoltage);
        } else {
          endArmMotors();
          armInUseUp = false;
        }
      } else if (armLow == false && armInUseUp == false) {
        if (armEncoderLeft.getPosition() <= -58.7 * Math.PI / 180) { // lower
          armInUseDown = true;
          armDriveLeft.setVoltage(ArmConstants.ArmVoltage);
          armDriveRight.setVoltage(ArmConstants.ArmVoltage);
        } else {
          endArmMotors();
          armInUseDown = false;
        }
      }
    } else {
      Elastic.sendNotification(armSyncError);
      endArmMotors();
      armInUseDown = false;
      armInUseUp = false;
    }
  }

  public void periodicOdometry() {
    SmartDashboard.putBoolean("armLow", armLow);
    SmartDashboard.putBoolean("armInUseDown", armInUseDown);
    SmartDashboard.putBoolean("armInUseUp", armInUseUp);

    SmartDashboard.putNumber("Left ARM motor position", armEncoderLeft.getPosition() * 180 / Math.PI);
    SmartDashboard.putNumber("Right ARM motor position", armEncoderRight.getPosition() * 180 / Math.PI);
  }

}