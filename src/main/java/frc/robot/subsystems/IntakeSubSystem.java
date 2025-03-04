package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {
  boolean intakeOut = false;
  double l2Force = 0.0;
  double r2Force = 0.0;
  boolean intakeMotorStopped = true;

  SparkMax intakeMotor;

  SparkMaxConfig sparkConfigIntakeMotor;

  public IntakeSubSystem(int intakeRollersCanId) {
    intakeMotor = new SparkMax(intakeRollersCanId, SparkMax.MotorType.kBrushless);

    sparkConfigIntakeMotor = new SparkMaxConfig();

    sparkConfigIntakeMotor
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    sparkConfigIntakeMotor.encoder
        .positionConversionFactor(0.037037037 * Math.PI * 2)
        .velocityConversionFactor(0.037037037 * Math.PI * 2);
    sparkConfigIntakeMotor.smartCurrentLimit(60, 60);

  }

  public void endIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void intakeTriggerReleased() {
    intakeMotorStopped = true;
  }

  public void intakeOut() {
    intakeOut = true;
    intakeMotorStopped = false;
    l2Force = RobotContainer.driverController.getL2Axis();
  }

  public void intakeIn() {
    intakeOut = false;
    intakeMotorStopped = false;
    r2Force = RobotContainer.driverController.getR2Axis();
  }

  public void intakeControl() {
    if (intakeOut == true && intakeMotorStopped == false) {
      intakeMotor.setVoltage(l2Force * -IntakeConstants.IntakeVoltage);
    } else if (intakeOut == false && intakeMotorStopped == false) {
      intakeMotor.setVoltage(r2Force * IntakeConstants.IntakeVoltage);
    } else {
      endIntakeMotor();
    }
  }

}
