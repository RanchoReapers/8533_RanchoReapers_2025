package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CageClawSubSystem;
import frc.robot.subsystems.IntakeSubSystem;

public class IntakeCmd extends Command {
    IntakeSubSystem intakeSubSystem;
    public IntakeCmd(IntakeSubSystem intakeSubSystem) {
        this.intakeSubSystem = intakeSubSystem;
        addRequirements(intakeSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // boolean rightBumper = RobotContainer.driverController.getAButton();
    boolean bPress = RobotContainer.driverController.getBButton();
    intakeSubSystem.intakeControl(bPress);
    // armSubsystem.armControl(rightBumper, leftBumper);
    // ^^ You can uncomment the above 3 to switch to arm control using A & B buttons instead of lt & rt
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubSystem.endMotors();// stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
