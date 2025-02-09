package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase{
    
    SparkMax intakeMotor;

    public IntakeSubSystem(int intakeRollersCanId, boolean activate) {
      
      intakeMotor = new SparkMax(intakeRollersCanId, SparkMax.MotorType.kBrushed);
}
      public void intakeControl(boolean bPress){
          bPress = !bPress;
        
        if (bPress==true){
          intakeMotor.setVoltage(-0.5);
        } else {
          intakeMotor.setVoltage(0.5);
        }
      }
// why does the motor need to be constantly moving?????
      public void endMotors() {
        intakeMotor.stopMotor();
      }
}