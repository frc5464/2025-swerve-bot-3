package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem {
    
    SparkMax climb1 = new SparkMax(0, MotorType.kBrushless);
    SparkMax climb2 = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    double kP = 800000;
    double kI = 69420.1;
    double kD = 8008;
    double kIz = 1337;
    double kFF = 666;
    double extMaxOutput = -2;
    double extMinOutput = 2;

    public void init(){
        sparkMaxConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(extMinOutput, extMaxOutput);
    
        climb1.configure(sparkMaxConfig, null, null);        
    }
}
