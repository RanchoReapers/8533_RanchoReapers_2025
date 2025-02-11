package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;
  boolean clampOpen = false;
  AbsoluteEncoder clawEncoder;

  public CageClawSubSystem(int clampOpenCanId) {

    cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushed);

    sparkConfigCageClawMotor = new SparkMaxConfig();

    clawEncoder = cageClawMotor.getAbsoluteEncoder();

    sparkConfigCageClawMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigCageClawMotor.encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor(Math.PI * 2);
    sparkConfigCageClawMotor.smartCurrentLimit(60, 60);

  }

  public void endClawMotors() {
    cageClawMotor.stopMotor();
  }
// fix this does not work
  public void clampControl(boolean yButton) {
    if (yButton == true) {
      clampOpen = !clampOpen;
    }  // if open

    if (clampOpen == true) {
      // if closed
      if (clawEncoder.getPosition() <= 60 * Math.PI / 180) {
        // move till open
        cageClawMotor.setVoltage(6);
      } else {
        endClawMotors();
        // when open stop moving
      }
    } else if (clampOpen == false) {
      // if open
      if (clawEncoder.getPosition() >= -60 * Math.PI / 180) {
        // move till closed
        cageClawMotor.setVoltage(-6);
      } else {
        // when closed stop moving
        endClawMotors();
      }
    } else {
      endClawMotors();
    }
  }

}
