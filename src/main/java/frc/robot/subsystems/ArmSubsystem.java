package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
    
    public void init(){
        //Encoderstuff
    }

SparkMax armAlgae = new SparkMax(0, MotorType.kBrushless);
SparkMax armCoral = new SparkMax(0, MotorType.kBrushless);
//third motor for rotation

RelativeEncoder relativeEncoder;
public double encoderPos;
public void periodic(){
    encoderPos = relativeEncoder.getPosition();
    SmartDashboard.putNumber("Encoder", encoderPos);
}

public void rotArm(){
    armCoral.set(0);
    armAlgae.set(0);
}

public void stopArm(){
    armAlgae.set(0);
    armCoral.set(0);
} 


}
