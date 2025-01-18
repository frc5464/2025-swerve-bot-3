package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ElevatorSubsystem {

    SparkMax leftEl = new SparkMax(37, MotorType.kBrushless);
    SparkMax rightEl = new SparkMax(36, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    // SparkClosedLoopController elPid;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double extMaxOutput = -2;
    double extMinOutput = 2;
    

    public void init(){
        // elPid = leftEl.getClosedLoopController();
        sparkMaxConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(extMinOutput, extMaxOutput);
        
        leftEl.configure(sparkMaxConfig, null, null);
    }

    public void goElevate(){
        leftEl.set(-0.075);
        rightEl.set(-0.075);
    }

    public void reverseElevate(){
        leftEl.set(0.05);
        rightEl.set(0.05);
    }

    public void stopElevate(){
        leftEl.set(0);
        rightEl.set(0);
    }

    
}
