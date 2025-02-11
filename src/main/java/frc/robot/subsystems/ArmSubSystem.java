package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSystem extends SubsystemBase {

  SparkMax armDriveLeft;
  SparkMax armDriveRight;

  SparkMaxConfig sparkConfigDriveRight;
  SparkMaxConfig sparkConfigDriveLeft;

  AbsoluteEncoder armEncoderLeft;
  AbsoluteEncoder armEncoderRight;

  boolean armLow = false;

  public ArmSubSystem(int armLeftCANId, int armRightCanId) {
    armDriveLeft = new SparkMax(armLeftCANId, SparkMax.MotorType.kBrushed);
    armDriveRight = new SparkMax(armRightCanId, SparkMax.MotorType.kBrushed);

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

  }

  public void endArmMotors() {
    armDriveLeft.stopMotor();
    armDriveRight.stopMotor();
  }

  public void armControl2State(boolean xButton) {
    if(xButton == true) {
      armLow = !armLow;
    }
    if (armEncoderLeft.getPosition() - armEncoderRight.getPosition() <= 5 * Math.PI / 180) {
      if (armLow == true) {
        if (armEncoderLeft.getPosition() >= 30 * Math.PI / 180) {
          armDriveLeft.setVoltage(-6);
          armDriveRight.setVoltage(-6);
        } else {
          endArmMotors();
        }
      } else if (armLow == false) {
        if (armEncoderLeft.getPosition() <= 60 * Math.PI / 180) {
          armDriveLeft.setVoltage(6);
          armDriveRight.setVoltage(6);
        } else {
          endArmMotors();
        }
      }
    } else {
      System.out.println("Attempted to move arm but failed. Arm motors more than 5 degrees out of sync. Value of bool armLow: " + armLow);
      endArmMotors();
    }
  }

}