package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageClawSubSystem extends SubsystemBase{
    
    SparkMax cageClawMotor;

    public CageClawSubSystem(int clampOpenCanId, boolean activate/*int armCANid, int handCANid, int ringHoldingCANid*/) {
      
      cageClawMotor = new SparkMax(clampOpenCanId, SparkMax.MotorType.kBrushed);
}
      public void clampControl(boolean aPress){
        aPress = !aPress;
        
        if (aPress==true){
          cageClawMotor.setVoltage(-0.5);
        } else {
          cageClawMotor.setVoltage(0.5);
        }
      }

      public void endMotors() {
        cageClawMotor.stopMotor();
        //ringHolding.stopMotor();
        //handSuction.stopMotor();
      }
}