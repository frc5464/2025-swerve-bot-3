package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem {
  
  SparkMax climber = new SparkMax(3, MotorType.kBrushless);
  DigitalInput limit_sw = new DigitalInput(0);
  public RelativeEncoder climbEncoder;

  private static final boolean ENABLED = true;
  
  double max_extension_counts = -250;
  double max_retraction_counts = 0;

  public double climbEncoderPos;
  public double counts;
  public boolean zeroed = false;

  public ClimbSubsystem(){
    climbEncoder = climber.getEncoder();
    climbEncoder.setPosition(0);
  }

  public void periodic(){
      SmartDashboard.putNumber("ClimbEncoder", climbEncoderPos);
      SmartDashboard.putBoolean("ClimbLimit", limit_sw.get());
      climbEncoderPos = climbEncoder.getPosition();
      if((zeroed == false) && limit_sw.get()){
        zeroed = true;
        reBoot();
      }
    } 
    
    // @Override
    public boolean isEnabled() {
      return ENABLED;
    }

    public void bringOut(){
      if(zeroed &&
        (climbEncoder.getPosition() > max_extension_counts)){
        climber.set(-1);
      }
      else{
        climber.set(0);
      }
    }
    public void bringIn(){
      if(limit_sw.get()){
        climber.set(0);
        reBoot();
      }
      else if(zeroed == false){
        climber.set(1);
      }
      else{
        climber.set(1);
      }
    }

    public void stop(){
      climber.set(0);
    }

    public void reBoot(){
      climbEncoder.setPosition(0);
    }
}

