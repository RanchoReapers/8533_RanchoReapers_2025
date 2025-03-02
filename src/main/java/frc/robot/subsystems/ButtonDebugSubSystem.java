package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonDebugSubSystem extends SubsystemBase{
    boolean buttonPressed = false;

    public ButtonDebugSubSystem() {

    }

    public void endSubSystemDebug() {
    }

    public void run() {
        buttonPressed = !buttonPressed;
    }

    public void debugOdometry() {
    // post to smart dashboard periodically
    SmartDashboard.putBoolean("buttonDebug", buttonPressed);

  }
}