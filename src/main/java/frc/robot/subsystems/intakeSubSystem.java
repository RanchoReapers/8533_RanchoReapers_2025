package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase{
    
    SparkMax intakeMotor;

    public IntakeSubSystem(int clampOpenCanId, boolean activate/*int armCANid, int handCANid, int ringHoldingCANid*/) {
      
      intakeMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushed);
}
      public void intakeControl(boolean bPress){
          bPress = !bPress;
        
        if (bPress==true){
          intakeMotor.setVoltage(-0.5);
        } else {
          intakeMotor.setVoltage(0.5);
        }
      }

      public void endMotors() {
        intakeMotor.stopMotor();
        //ringHolding.stopMotor();
        //handSuction.stopMotor();
      }
}