package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem {
    
    SparkMax armAlgae = new SparkMax(0, MotorType.kBrushless);
    SparkMax armCoral = new SparkMax(0, MotorType.kBrushless);
    SparkMax armRot = new SparkMax(0, MotorType.kBrushless);
    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    double kP = 800000;
    double kI = 69420.1;
    double kD = 8008;
    double kIz = 1337;
    double kFF = 666;
    double extMaxOutput = -2;
    double extMinOutput = 2;

    RelativeEncoder relativeEncoder;
    public double encoderPos;
    public void periodic(){
    encoderPos = relativeEncoder.getPosition();
    SmartDashboard.putNumber("Encoder", encoderPos);
    }

    public void init(){
    //Encoderstuff
    sparkMaxConfig.closedLoop
        .p(kP)
        .i(kI)
        .d(kD)
        .outputRange(extMinOutput, extMaxOutput);
    
    armRot.configure(sparkMaxConfig, null, null);
    }

    public void rotArm(){
    armCoral.set(0);
    armAlgae.set(0);
    }

    public void stopArm(){
    armAlgae.set(0);
    armCoral.set(0);
    } 

    public void lvl4Arm(){
    
    }


    }
