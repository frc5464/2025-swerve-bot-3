package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ProcessorArmSubsystem {
    SparkMax processorRotater = new SparkMax(0, MotorType.kBrushless);
    SparkMax processorRoller = new SparkMax(0, MotorType.kBrushless);
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
    
        processorRotater.configure(sparkMaxConfig, null, null);        
    }
}
