package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.Commands;

public class LimelightDetectionSubSystem {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    boolean bButton = false;
    boolean aimLockout = false;

    public LimelightDetectionSubSystem() {

    }

    public void initLockout(boolean bButton) {
        if (bButton == false) {
            aimLockout = false;
        } else if (bButton == true) {
            aimLockout = true;
            Commands.waitSeconds(7);
            aimLockout = false;
        }
    }

    public void aimAssist(boolean bButton) {
        if (aimLockout == false) {
            if (bButton == false && aimLockout == false) {
                double heading_error = -tx;
                double distance_error = -ty;
                double steering_adjust = 0.0f;

        // tank drive -- rewrite for swerve
        /*
            if (tx > 1.0) {
                steering_adjust = LimelightConstants.KpAim * heading_error - LimelightConstants.MinAimCommand;
            } else if (tx < -1.0) {
                steering_adjust = LimelightConstants.KpAim * heading_error + LimelightConstants.MinAimCommand;
            }

            double distance_adjust = LimelightConstants.KpDistance * distance_error;

            left_command += steering_adjust + distance_adjust;
            right_command -= steering_adjust + distance_adjust;
        */

            }
        }
    }

    public void periodicOdometry() {

        // read values periodically
        double x = tx;
        double y = ty;
        double area = ta;

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

}
