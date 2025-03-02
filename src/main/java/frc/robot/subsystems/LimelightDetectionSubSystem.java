package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDetectionSubSystem extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    long tid = table.getEntry("tid").getInteger(0);

    int tagsInView = 1;

    boolean aimAssistActive = false;
    boolean limelightOverride = false;

    // double[]{} botpose = table.getEntry("botpose").getDoubleArray();

    public Command activateLimelightOverride() {
      limelightOverride = !limelightOverride;
      return new InstantCommand();
    }

    public LimelightDetectionSubSystem() {
        if((tid == 12 || tid == 13 || tid == 2 || tid == 1) && tagsInView == 1 && limelightOverride == false) {
        //if limelight sees only one tag & it is at one of the pickup stations
            aimAssistActive = true;
        } else if((tid == 17 || tid == 18 || tid == 19 || tid == 20 || tid == 21 || tid == 22 || tid == 6 || tid == 7 || tid == 8 || tid == 9 || tid == 10 || tid == 11) && tagsInView == 1 && limelightOverride == false) {
        // if limelight sees only one tag & it is at one of the dropoff stations (yes this is the worst possible way of writing this code but it works)
            aimAssistActive = true;
        } else {
            aimAssistActive = false;
        }

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
        double limelightX = tx;
        double limelightY = ty;
        double limelightArea = ta;
        double currentTid = tid;

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", limelightX);
        SmartDashboard.putNumber("LimelightY", limelightY);
        SmartDashboard.putNumber("LimelightArea", limelightArea);
        SmartDashboard.putNumber("LimelightID", currentTid);
        SmartDashboard.putBoolean("aimAssistActive", aimAssistActive);

    }

}


