package frc.robot;

import java.util.List;

import choreo.Choreo;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;


import frc.robot.commands.ArmJoystickCmd;
import frc.robot.commands.CageClawCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;

import frc.robot.subsystems.ArmSubSystem;
import frc.robot.subsystems.CageClawSubSystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.SwerveSubSystem;
import frc.robot.subsystems.LimelightDetectionSubSystem;
//import frc.robot.subsystems.LimelightDetectionSubSystem;

public class RobotContainer {
  // Define subsystems and commands
  public final static SwerveSubSystem swerveSubsystem = new SwerveSubSystem();
  public final static ArmSubSystem armSubsystem = new ArmSubSystem(14,15);
  public final static CageClawSubSystem cageClawSubsystem = new CageClawSubSystem(16);
  public final static IntakeSubSystem intakeSubsystem = new IntakeSubSystem(17);
  public final static PS4Controller driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  public final static LimelightDetectionSubSystem limelightDetectionSubsystem = new LimelightDetectionSubSystem();
  
  public final static Trigger circleButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kCircle.value);
  public final static Trigger squareButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kSquare.value);
  public final static Trigger triangleButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kTriangle.value);
  public final static Trigger shareButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kShare.value);

  public final static Trigger l2ButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kL2.value);
  public final static Trigger r2ButtonTrigger = new JoystickButton(driverController, PS4Controller.Button.kR2.value);

  public final static Field2d m_field = new Field2d();


  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(swapDriveControlMethod());

    /* swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
     () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
     () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
     () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis), 
     () -> !driverController.getL1Button()));
    */
    
     squareButtonTrigger.debounce(0.1).onTrue(armSwitchLowVar()); // press cross and you get square
     triangleButtonTrigger.debounce(0.1).onTrue(callSwitchClawArmVar()); // press triangle and you get triangle
     circleButtonTrigger.debounce(0.1).onTrue(clawSwitchOpenVar()); // press square and you get circle
     // crossButtonTrigger.debounce(0.1).onTrue(intakeSwitchDirectionVar()); // press circle and you get cross

     r2ButtonTrigger.debounce(0.1).whileTrue(callIntakeOut()).whileFalse(callIntakeTriggerReleased());
     l2ButtonTrigger.debounce(0.1).whileTrue(callIntakeIn()).whileFalse(callIntakeTriggerReleased());

     cageClawSubsystem.setDefaultCommand(new CageClawCmd(cageClawSubsystem));
     armSubsystem.setDefaultCommand(new ArmJoystickCmd(armSubsystem));
     intakeSubsystem.setDefaultCommand(new IntakeCmd(intakeSubsystem));
  }

  public Command swapDriveControlMethod() {
    return new ConditionalCommand(new SwerveJoystickCmd(swerveSubsystem,
    () -> limelightDetectionSubsystem.getXSpeedLimelight(), 
    () -> limelightDetectionSubsystem.getYSpeedLimelight(),
    () -> limelightDetectionSubsystem.getTurnAngleLimelight(), 
    () -> false), 
       new SwerveJoystickCmd(swerveSubsystem,
       () -> -driverController.getRawAxis(OIConstants.kDriverYAxis), 
       () -> -driverController.getRawAxis(OIConstants.kDriverXAxis),
       () -> -driverController.getRawAxis(OIConstants.kDriverRotAxis), 
       () -> !driverController.getL1Button()), 
           limelightDetectionSubsystem.getAimAssistActive());
  }

  public Command callIntakeOut() {
      return new InstantCommand(() -> intakeSubsystem.intakeOut());
  }

  public Command callIntakeTriggerReleased() {
    return new InstantCommand(() -> intakeSubsystem.intakeTriggerReleased());
  }

  public Command callIntakeIn() {
      return new InstantCommand(() -> intakeSubsystem.intakeIn());
  }

  public Command armSwitchLowVar() {
      return new InstantCommand(() -> armSubsystem.switchArmLow());
  } 

  public Command clawSwitchOpenVar() {
      return new InstantCommand(() -> cageClawSubsystem.switchClawOpen());
  }

  public Command callSwitchClawArmVar() {
    return new InstantCommand(() -> armSubsystem.switchClawArmLow());
  }


  public Command getAutonomousCommand() {

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
        swerveSubsystem.periodic();
        swerveSubsystem.disabledPeriodic();
        armSubsystem.periodicOdometry();
        SmartDashboard.putBoolean("Joystick Arm State", driverController.getSquareButton());
        SmartDashboard.putBoolean("Joystick Claw State", driverController.getTriangleButton());
        SmartDashboard.putNumber("Joystick Intake State R", driverController.getR2Axis());
        SmartDashboard.putNumber("Joystick Intake State L", driverController.getL2Axis());

        cageClawSubsystem.periodicOdometry();
        SmartDashboard.putNumber("Left Y Joystick Axis", driverController.getRawAxis(OIConstants.kDriverYAxis));
        SmartDashboard.putNumber("Left X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverXAxis));
        SmartDashboard.putNumber("Right X Joystick Axis", driverController.getRawAxis(OIConstants.kDriverRotAxis));
        limelightDetectionSubsystem.periodicOdometry();

        // joystick debug bindings
        SmartDashboard.putBoolean("JOYSTICK Share", driverController.getShareButton());
        SmartDashboard.putBoolean("JOYSTICK Options", driverController.getOptionsButton());
        SmartDashboard.putBoolean("JOYSTICK L1", driverController.getL1Button());
        SmartDashboard.putBoolean("JOYSTICK R1", driverController.getR1Button());
        SmartDashboard.putBoolean("JOYSTICK L2", driverController.getL2Button());
        SmartDashboard.putBoolean("JOYSTICK R2", driverController.getR2Button());
        SmartDashboard.putBoolean("JOYSTICK Triangle", driverController.getTriangleButton());
        SmartDashboard.putBoolean("JOYSTICK Square", driverController.getCircleButton());
        SmartDashboard.putBoolean("JOYSTICK Circle", driverController.getCrossButton());
        SmartDashboard.putBoolean("JOYSTICK Cross", driverController.getSquareButton());
        SmartDashboard.putBoolean("JOYSTICK Touchpad", driverController.getTouchpadButton());
        SmartDashboard.putBoolean("JOYSTICK PS", driverController.getPSButton());
        SmartDashboard.putBoolean("JOYSTICK R3", driverController.getR3Button());
        SmartDashboard.putBoolean("JOYSTICK L3", driverController.getL3Button());


        

    }

    public void enabledInit() {

    }

    public void disabledInit() {
        
    }
    
}