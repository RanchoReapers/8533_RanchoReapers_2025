package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Constants.LimelightConstants;

public class LimelightDetectionSubSystem {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);


    public LimelightDetectionSubSystem() {

    }


    boolean bButton = false;

    public void aimAssist(boolean bButton) {
        if (bButton) {
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
