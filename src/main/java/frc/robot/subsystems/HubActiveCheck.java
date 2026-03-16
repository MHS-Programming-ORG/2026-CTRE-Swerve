// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubActiveCheck extends SubsystemBase {
  public boolean hubIsActive = false;
  public Timer timer = new Timer();
  public HubActiveCheck() {
    
  }

  public void resetTimer(){
    timer.reset();
  }

  public void setHubActivity(boolean weWin){
    hubIsActive = weWin;
  }

  public void swapHubActivitiy(){
    if(hubIsActive){
      hubIsActive = false;
    } else{
      hubIsActive = true;
    }
  }

  boolean transition = false; 
  boolean phase2 = false; 
  boolean phase3 = false; 
  boolean phase4 = false; 

  @Override
  public void periodic() {
    hubIsActive = true;
    if(timer.get() >= 10 && !transition){
      transition = true;
    }
    if(timer.get() >= 25 && !phase2){
      swapHubActivitiy();
      phase2 = true;
    }

  }
}
