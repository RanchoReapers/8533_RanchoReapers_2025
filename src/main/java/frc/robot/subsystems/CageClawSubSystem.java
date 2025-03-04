package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageClawConstants;
import com.revrobotics.RelativeEncoder;
;


public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;

  RelativeEncoder clawEncoder;

  boolean clawOpen = false;
  // lockouts to prevent user switching arm state too often
  boolean clawInUseOpen = false; // Arm is currently being used to move open
  boolean clawInUseClosed = false; // Arm is currently being used to move closed

  public CageClawSubSystem(int clampOpenCanId) {

    cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushless);

    sparkConfigCageClawMotor = new SparkMaxConfig();

    clawEncoder = cageClawMotor.getEncoder();

    sparkConfigCageClawMotor
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigCageClawMotor.encoder
        .positionConversionFactor(0.0133333 * Math.PI * 2)
        .velocityConversionFactor(0.0133333 * Math.PI * 2);
    sparkConfigCageClawMotor.smartCurrentLimit(60, 60);

    cageClawMotor.configure(sparkConfigCageClawMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void endClawMotors() {
    cageClawMotor.stopMotor();
  }

  public void switchClawOpen() {
    if (clawInUseOpen == false && clawInUseClosed == false) {
      clawOpen = !clawOpen;
    }
  }

  public void clampControl() {
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

  public void periodicOdometry() {
    SmartDashboard.putBoolean("clawOpen", clawOpen);
    SmartDashboard.putBoolean("clawInUseOpen", clawInUseOpen);
    SmartDashboard.putBoolean("clawInUseClosed", clawInUseClosed);

    SmartDashboard.putNumber("CLAW motor position", clawEncoder.getPosition() * 180/Math.PI);
  }

}