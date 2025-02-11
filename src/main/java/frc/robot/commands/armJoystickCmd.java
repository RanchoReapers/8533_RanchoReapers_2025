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
      armSubsystem.armControl2State(RobotContainer.driverController.getXButtonPressed());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.endArmMotors(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
