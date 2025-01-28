package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class climberSubSystem extends SubsystemBase {

    SparkMax climbLeft;
    SparkMax climbRight;
    SparkMaxConfig climbLeftConfig = new SparkMaxConfig();

    public climberSubSystem(int canID_right,int canID_left) {
        climbRight = new SparkMax(canID_right, SparkMax.MotorType.kBrushless);
        climbLeft = new SparkMax(canID_left, SparkMax.MotorType.kBrushless);

        climbLeftConfig
        .inverted(true);
        
        climbLeft.configure(climbLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void climbControl(boolean xPress, boolean yPress) {
        if (xPress == true && yPress != true) {
          climbRight.setVoltage(8);
          climbLeft.setVoltage(8);
        }
        else if (yPress == true && xPress != true) {
          climbRight.setVoltage(-8);
          climbLeft.setVoltage(-8);
        }
        else {
          climbRight.stopMotor();
          climbLeft.stopMotor();
        }

      }

      public void endMotors() {
        climbRight.stopMotor();
        climbLeft.stopMotor();
      }
}
