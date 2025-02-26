package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CageClawCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CageClawSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.SwerveSubSystem;


public class RobotContainer {
  // Define subsystems and commands
  public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
  public final static ArmSubSystem armSubsystem = new ArmSubSystem(14,15);
  public final static CageClawSubSystem cageClawSubsystem = new CageClawSubSystem(16);
  public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(17);
  public final static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public final static XboxController operatorController = new XboxController(USB.OPERATOR_CONTROLLER);
  
  public final static Trigger xButton = new JoystickButton(driverController, XboxController.Button.kX.value);


  // contains subsystems, OI devices, and commands
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
     () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
     () -> -driverController.getRawAxis(OIConstants.kDriverXAxis), 
     () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis), 
     () -> !driverController.getLeftBumperButton()));

     armSubsystem.setDefaultCommand(new ArmJoystickCmd(armSubsystem));
     cageClawSubsystem.setDefaultCommand(new CageClawCmd(cageClawSubsystem));
     intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));
  }


  public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(0.075, 0),
                        new Translation2d(0.-.075, 0)),
                new Pose2d(-0.25 , 0.01, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
          swerveControllerCommand,
          new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public void disabledPeriodic() {
        //telemetry for debugging
        swerveSubsystem.disabledPeriodic();
        armSubsystem.periodicOdometry();
        SmartDashboard.putBoolean("Joystick Arm State", driverController.getXButtonPressed());
        SmartDashboard.putBoolean("Joystick Claw State", driverController.getYButtonPressed());
        SmartDashboard.putBoolean("Joystick Intake State", driverController.getAButtonPressed());
    }

    public void enabledInit() {
        RobotContainer.armSubsystem.updateArmAbsolutePosition();
        RobotContainer.cageClawSubsystem.updateClawAbsolutePosition();
    }

    public void disabledInit() {
        RobotContainer.armSubsystem.saveArmAbsoluteMotorPositions();
        RobotContainer.cageClawSubsystem.saveClawAbsoluteMotorPositions();
    }
    
}