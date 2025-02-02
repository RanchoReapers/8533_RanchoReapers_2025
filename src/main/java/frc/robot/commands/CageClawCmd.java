package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CageClawSubSystem;

public class CageClawCmd extends Command {
    CageClawSubSystem cageClawSubSystem;
    public CageClawCmd(CageClawSubSystem cageClawSubSystem) {
        this.cageClawSubSystem = cageClawSubSystem;
        addRequirements(cageClawSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // boolean rightBumper = RobotContainer.driverController.getAButton();
    boolean aPress = RobotContainer.driverController.getAButton();
    cageClawSubSystem.clampControl(aPress);
    // armSubsystem.armControl(rightBumper, leftBumper);
    // ^^ You can uncomment the above 3 to switch to arm control using A & B buttons instead of lt & rt
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cageClawSubSystem.endMotors();// stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
