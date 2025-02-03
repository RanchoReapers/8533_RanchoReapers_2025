package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubSystem extends SubsystemBase {
    
    SparkMax armDriveLeft;
    SparkMax armDriveRight;

    public ArmSubSystem(int armUpCANid, int armDownCanid, int handCANid, int ringHoldingCANid) {
        armDriveLeft = new SparkMax(armUpCANid, SparkMax.MotorType.kBrushed);
        armDriveRight = new SparkMax(armDownCanid, SparkMax.MotorType.kBrushed);
    }
    public void armControl(double rightTrigger, double leftTrigger) {
        if (leftTrigger > 0.02 && rightTrigger < 0.02) {
          armDriveLeft.setVoltage(-6 * leftTrigger);
          armDriveRight.setVoltage(12 * rightTrigger);
        }
        else if (rightTrigger > 0.02 && leftTrigger < 0.02) {
          armDriveLeft.setVoltage(12 * rightTrigger);
          armDriveRight.setVoltage(-6 * leftTrigger);
        }
        else {
          armDriveLeft.stopMotor();
          armDriveRight.stopMotor();
          
        }
    }
      public void endMotors() {
        armDriveLeft.stopMotor();
        armDriveRight.stopMotor();
  }
}
