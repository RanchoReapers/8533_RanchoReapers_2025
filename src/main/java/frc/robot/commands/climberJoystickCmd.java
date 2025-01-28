package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climberSubSystem;

public class climberJoystickCmd extends Command {
    climberSubSystem climberSubSystem;

    public climberJoystickCmd(climberSubSystem ClimberSubSystem) {
        climberSubSystem = ClimberSubSystem;
        addRequirements(climberSubSystem);

    }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean xPress = RobotContainer.driverController.getXButton();
    boolean yPress = RobotContainer.driverController.getYButton();
    // boolean rightBumper = RobotContainer.driverController.getAButton();
    // boolean leftBumper = RobotContainer.driverController.getBButton();
    climberSubSystem.climbControl(xPress, yPress);
    // climberSubSystem.climbControl(rightBumper, leftBumper);
    // ^^ You can uncomment the above 3 to switch to climb control using A & B buttons instead of x & y
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubSystem.endMotors();// stop motors once interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
