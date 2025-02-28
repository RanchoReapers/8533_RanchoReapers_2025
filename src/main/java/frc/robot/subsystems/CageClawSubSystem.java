package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CageClawConstants;

public class CageClawSubSystem extends SubsystemBase {

  SparkMax cageClawMotor;
  SparkMaxConfig sparkConfigCageClawMotor;

  RelativeEncoder RELclawEncoder;

  boolean clawOpen = false;
  // lockouts to prevent user switching arm state too often
  boolean clawInUseOpen = false; // Arm is currently being used to move open
  boolean clawInUseClosed = false; // Arm is currently being used to move closed

  double absolutePositionClawMotor;
  // defines the deviation of motors using the relative encoder acting as an absolute encoder

  private static final String PREF_KEY_CLAW_MOTOR = "ClawMotorAbsolutePosition";
  // preferences keys for saving absolute motor positions in memory

  public CageClawSubSystem(int clampOpenCanId) {

    cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushless);

    sparkConfigCageClawMotor = new SparkMaxConfig();

    RELclawEncoder = cageClawMotor.getEncoder();

    absolutePositionClawMotor = Preferences.getDouble(PREF_KEY_CLAW_MOTOR, 0.0);

    sparkConfigCageClawMotor
        .idleMode(IdleMode.kBrake)
        .inverted(false);
    sparkConfigCageClawMotor.encoder
        .positionConversionFactor(0.0133333 * Math.PI * 2)
        .velocityConversionFactor(0.0133333 * Math.PI * 2);
    sparkConfigCageClawMotor.smartCurrentLimit(60, 60);

    cageClawMotor.configure(sparkConfigCageClawMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void updateClawAbsolutePosition() {
    absolutePositionClawMotor += RELclawEncoder.getPosition();

    RELclawEncoder.setPosition(0.0);
    //reset encoders to 0 after reading
  }

  public void endClawMotors() {
    cageClawMotor.stopMotor();
  }

  public void clampControl(boolean yButton) {
    if (yButton == true && clawInUseOpen == false && clawInUseClosed == false) {  
      clawOpen = !clawOpen;
    }
    if (clawOpen == true && clawInUseOpen == false) {
      if (absolutePositionClawMotor <= 119.7 * Math.PI / 180) {
        clawInUseClosed = true;
        cageClawMotor.setVoltage(CageClawConstants.CageClawVoltage);
      } else {
        clawInUseClosed = false;
        clawOpen = false;
        pleaseCloseClaw();
      }
    }
  }

  public void pleaseCloseClaw() {
    if (clawOpen == false && clawInUseClosed == false) {
      if (absolutePositionClawMotor >= 0.7 * Math.PI / 180) {
        clawInUseOpen = true;
        cageClawMotor.setVoltage(-CageClawConstants.CageClawVoltage);
      } else {
        endClawMotors();
        clawInUseOpen = false;
      }
    }
  }

  public void saveClawAbsoluteMotorPositions() {
    Preferences.setDouble(PREF_KEY_CLAW_MOTOR, absolutePositionClawMotor);
  }

  public void periodicOdometry() {

    updateClawAbsolutePosition();

    // post to smart dashboard periodically
    SmartDashboard.putBoolean("clawOpen", clawOpen);
    SmartDashboard.putBoolean("clawInUseOpen", clawInUseOpen);
    SmartDashboard.putBoolean("clawInUseClosed", clawInUseClosed);

    // Arm debug
    SmartDashboard.putNumber("SPARK-REL CLAW motor position", RELclawEncoder.getPosition() * 180/Math.PI);
    SmartDashboard.putNumber("CONV-ABS CLAW motor position", absolutePositionClawMotor * 180/Math.PI);
  }

}
