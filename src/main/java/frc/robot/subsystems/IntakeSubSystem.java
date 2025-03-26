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
  double l2ForceFixed = 0;
  double r2ForceFixed = 0;
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
    if (!RobotContainer.driverController.getL1Button() && !RobotContainer.driverController.getR1Button()) {
    intakeMotorStopped = true;
    }
  }

  public void intakeOut() {
    intakeOut = true;
    //l2Force = RobotContainer.driverController.getR2Axis();
    intakeMotorStopped = false;
  }

  public void intakeIn() {
    intakeOut = false;
    //r2Force = RobotContainer.driverController.getL2Axis();
    intakeMotorStopped = false;
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

  public double getl2Force() {
    if (l2Force == -1) {
      l2ForceFixed = 0;
    } else if (l2Force < 0 ) {
      l2ForceFixed = Math.abs(l2Force);
    }
    return l2ForceFixed;
  }

  public double getr2Force() {
    return r2ForceFixed;
  }                                       

  public void intakePeriodic() {
    SmartDashboard.putNumber("l2Force", l2Force);
    SmartDashboard.putNumber("r2Force", r2Force);
    SmartDashboard.putBoolean("intakeOut", intakeOut);
    SmartDashboard.putBoolean("intakeMotorStopped", intakeMotorStopped);
  }

}
