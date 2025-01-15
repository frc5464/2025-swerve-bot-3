package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem {

    SparkMax leftEl = new SparkMax(0, MotorType.kBrushless);
    SparkMax rightEl = new SparkMax(0, MotorType.kBrushless);

    public void goElevate(){
        leftEl.set(0);
        rightEl.set(0);
    }

    public void reverseElevate(){
        leftEl.set(0);
        rightEl.set(0);
    }

    public void stopElevate(){
        leftEl.set(0);
        rightEl.set(0);
    }

    
}
