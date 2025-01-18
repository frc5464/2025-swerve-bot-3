package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ElevatorSubsystem {

    SparkMax leftEl = new SparkMax(0, MotorType.kBrushless);
    SparkMax rightEl = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    // SparkClosedLoopController elPid;
    double kP = 800000;
    double kI = 69420.1;
    double kD = 8008;
    double kIz = 1337;
    double kFF = 666;
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
        leftEl.set(0.5);
        // rightEl.set(-0.5);
    }

    public void reverseElevate(){
        leftEl.set(-0.5);
        // rightEl.set(0.5);
    }

    public void stopElevate(){
        leftEl.set(0);
        // rightEl.set(0);
    }

    
}
