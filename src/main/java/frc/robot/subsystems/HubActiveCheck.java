// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubActiveCheck extends SubsystemBase {
  public boolean hubIsActive = true;
  public boolean didWeWin = false;
  public Timer timer = new Timer();

  public HubActiveCheck() {
  }

  public void resetTimer(){
    timer.reset();
  }

  public void setHubActivity(boolean WeWin){
    didWeWin = WeWin;
  }

  public void swapHubActivitiy(){
    if(hubIsActive){
      hubIsActive = false;
    } else{
      hubIsActive = true;
    }
  }

  boolean transitionPassed = false; 
  boolean shift1Passed = false; 
  boolean shift2Passed = false; 
  boolean shift3Passed = false; 
  boolean shift4Passed = false; 

  @Override
  public void periodic() {
    hubIsActive = true;
    SmartDashboard.putBoolean("HubActive", hubIsActive);
    if(timer.get() >= 10 && !transitionPassed){
      hubIsActive = didWeWin;
      transitionPassed = true;
    }
    if(timer.get() >= 35 && !shift1Passed){
      swapHubActivitiy();
      shift1Passed = true;
    }
    if(timer.get() >= 60 && !shift2Passed){
      swapHubActivitiy();
      shift2Passed = true;
    }
    if(timer.get() >= 85 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
    }
    if(timer.get() >= 110 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
    }
    if(timer.get() >= 135 && !shift4Passed){
      hubIsActive = true;
      shift4Passed = true;
    }
  }
}
