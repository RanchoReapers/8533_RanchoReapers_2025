package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubSystem;

public class armJoystickCmd extends Command {
    ArmSubSystem armSubsystem;
    public armJoystickCmd(ArmSubSystem armSubSystem) {
        armSubsystem = armSubSystem;
        addRequirements(armSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTrigger = RobotContainer.driverController.getRightTriggerAxis();
    double leftTrigger = RobotContainer.driverController.getLeftTriggerAxis();
    // boolean rightBumper = RobotContainer.driverController.getAButton();
    // boolean leftBumper = RobotContainer.driverController.getBButton();
    armSubsystem.armControl(rightTrigger, leftTrigger);
    // armSubsystem.armControl(rightBumper, leftBumper);
    // ^^ You can uncomment the above 3 to switch to arm control using A & B buttons instead of lt & rt
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.endMotors();// stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
