package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageClawConstants;

public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;

  AbsoluteEncoder clawEncoder;

  boolean clawOpen = false;
  // lockouts to prevent user switching arm state too often
  boolean clawInUseOpen = false; // Arm is currently being used to move open
  boolean clawInUseClosed = true; // Arm is currently being used to move closed

  public CageClawSubSystem(int clampOpenCanId) {

    cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushless);

    sparkConfigCageClawMotor = new SparkMaxConfig();

    clawEncoder = cageClawMotor.getAbsoluteEncoder();

    sparkConfigCageClawMotor
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigCageClawMotor.encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor(Math.PI * 2);
    sparkConfigCageClawMotor.smartCurrentLimit(60, 60);

    cageClawMotor.configure(sparkConfigCageClawMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void endClawMotors() {
    cageClawMotor.stopMotor();
  }

  // fix this does not work
  public void clampControl(boolean yButton) {
    if (yButton == true) {
      clawOpen = !clawOpen;
    } 
    if (clawOpen == true && clawInUseOpen == false) {
      if (clawEncoder.getPosition() <= CageClawConstants.CageClawTravelAngle * Math.PI / 180) {
        cageClawMotor.setVoltage(-CageClawConstants.CageClawVoltage);
      } else {
        endClawMotors();
      }
    } else if (clawOpen == false && clawInUseClosed == true) {
      if (clawEncoder.getPosition() >= -CageClawConstants.CageClawTravelAngle * Math.PI / 180) {
        cageClawMotor.setVoltage(CageClawConstants.CageClawVoltage);
      } else {
        endClawMotors();
      }
    } else {
      endClawMotors();
    }
  }

}
