package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ButtonDebugSubSystem;

public class ButtonDebugCmd extends Command {
    ButtonDebugSubSystem buttonDebugSubSystem;
    public ButtonDebugCmd(ButtonDebugSubSystem buttonDebugSubsystem) {
        buttonDebugSubSystem = buttonDebugSubsystem;
        addRequirements(buttonDebugSubSystem);
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      buttonDebugSubSystem.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    buttonDebugSubSystem.endSubSystemDebug(); // stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
