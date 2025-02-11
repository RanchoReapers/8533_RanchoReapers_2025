package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {
  boolean intakeOut = false;
  SparkMax intakeMotor;

  SparkMaxConfig sparkConfigIntakeMotor;

  public IntakeSubSystem(int intakeRollersCanId) {
    intakeMotor = new SparkMax(intakeRollersCanId, SparkMax.MotorType.kBrushed);

    sparkConfigIntakeMotor = new SparkMaxConfig();

    sparkConfigIntakeMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigIntakeMotor.encoder
        .positionConversionFactor(Math.PI * 2)
        .velocityConversionFactor(Math.PI * 2);
    sparkConfigIntakeMotor.smartCurrentLimit(60, 60);

  }

  public void endIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void intakeControl(boolean aPress) {
    if (aPress == true) {
    intakeOut = !intakeOut;
    }
    
    if (intakeOut == true) {
      intakeMotor.setVoltage(-0.5);
    } else {
      intakeMotor.setVoltage(0.5);
    }
  }

}
