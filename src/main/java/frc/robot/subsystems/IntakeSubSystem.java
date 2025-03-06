package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {
  boolean intakeOut = false;
  double l2Force = 1;
  double r2Force = 1;
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
    System.out.println("endIntakeMotor() has been called");
  }

  public void intakeTriggerReleased() {
    if (r2Force <= 0.3 && l2Force <= 0.3) {
    intakeMotorStopped = true;
    System.out.println("intakeTriggerReleased() has been called -- && stopping motor");

    }
    System.out.println("intakeTriggerReleased() has been called");
  }

  public void intakeOut() {
    intakeOut = true;
    r2Force = RobotContainer.driverController.getR2Axis();
    intakeMotorStopped = false;
    System.out.println("intakeOut() has been called");

  }

  public void intakeIn() {
    intakeOut = false;
    l2Force = RobotContainer.driverController.getL2Axis();
    intakeMotorStopped = false;
    System.out.println("intakeIn() has been called");

  }

  public void intakeControl() {
    if (intakeOut == true && intakeMotorStopped == false) {
      intakeMotor.setVoltage(l2Force * -IntakeConstants.IntakeVoltage);
    } else if (intakeOut == false && intakeMotorStopped == false) {
      intakeMotor.setVoltage(r2Force * IntakeConstants.IntakeVoltage);
    } else {
      endIntakeMotor();
      System.out.println("else on intakeControl");

    }
  }

  public void intakePeriodic() {
    SmartDashboard.putNumber("l2Force", l2Force);
    SmartDashboard.putNumber("r2Force", r2Force);
    SmartDashboard.putBoolean("intakeOut", intakeOut);
    SmartDashboard.putBoolean("intakeMotorStopped", intakeMotorStopped);
  }

}
