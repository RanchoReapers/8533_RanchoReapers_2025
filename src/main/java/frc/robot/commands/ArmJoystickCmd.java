package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubSystem;

public class ArmJoystickCmd extends Command {
    ArmSubSystem armSubsystem;
    public ArmJoystickCmd(ArmSubSystem armSubSystem) {
        armSubsystem = armSubSystem;
        addRequirements(armSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      armSubsystem.armControl3State();
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
