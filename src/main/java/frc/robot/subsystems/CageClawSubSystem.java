package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;

  AbsoluteEncoder clawEncoder;

  boolean clawOpen = false;

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

  public void clampControl() {
      if (clawEncoder.getPosition() >= 60 * Math.PI / 180) {
        clawOpen = false;
      } else if (clawEncoder.getPosition() <= -60 * Math.PI / 180) {
        clawOpen = true;
      }
      if (clawOpen == true) {
        if (clawEncoder.getPosition() >= -60 * Math.PI / 180) {
          cageClawMotor.setVoltage(-6);
        } else {
          endClawMotors();
        }
      } else if (clawOpen == false) {
        if (clawEncoder.getPosition() <= 60 * Math.PI / 180) {
          cageClawMotor.setVoltage(6);
        } else {
          endClawMotors();
        }
      }
  }

}
