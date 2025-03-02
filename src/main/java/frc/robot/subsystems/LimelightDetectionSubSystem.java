package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDetectionSubSystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);
    long tid = table.getEntry("tid").getInteger(0);
    // double[]{} botpose = table.getEntry("botpose").getDoubleArray();

    public LimelightDetectionSubSystem() {
        /*
            if (bButton == false) {
                double heading_error = -tx; //right pos left neg
                double distance_error = -ty; //up pos down neg
                double steering_adjust = 0.0f;
                long tagNumber = tid;

        // tank drive -- rewrite for swerve

            if (tx > 1.0) { //if off to the right
                steering_adjust = LimelightConstants.KpAim * heading_error - LimelightConstants.MinAimCommand;
            } else if (tx < -1.0) { // if off to the left
                steering_adjust = LimelightConstants.KpAim * heading_error + LimelightConstants.MinAimCommand;
            }

            double distance_adjust = LimelightConstants.KpDistance * distance_error;

            left_command += steering_adjust + distance_adjust;
            right_command -= steering_adjust + distance_adjust;

            }
        */
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


