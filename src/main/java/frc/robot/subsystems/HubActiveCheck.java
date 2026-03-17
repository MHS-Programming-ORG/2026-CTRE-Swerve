// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubActiveCheck extends SubsystemBase {
  public boolean hubIsActive = true;
  public boolean didWeWin = true;
  public Timer timer = new Timer();
  public Timer countdownTimer = new Timer();
  public double countdown = 10;

  boolean transitionPassed = false; 
  boolean shift1Passed = false; 
  boolean shift2Passed = false; 
  boolean shift3Passed = false; 
  boolean shift4Passed = false; 

  public HubActiveCheck() {
  }

  public void restartTimer(){
    timer.reset();
    timer.start();
  }

  public void startCountdown(){
    countdownTimer.reset();
    countdownTimer.start();
  }

  public void setTransitionShift(){
    hubIsActive = true;
  }

  public void setCountdown(){
    if(!transitionPassed){
      countdown = 10;
    }
    else if(!shift1Passed || !shift2Passed || !shift3Passed || !shift4Passed){
      countdown = 25;
    }
    else{
      countdown = 30;
    }
  }

  public void setHubActivity(){
    if(didWeWin){
      didWeWin = false;
    } else{
      didWeWin = true;
    }
  }

  public void swapHubActivitiy(){
    if(hubIsActive){
      hubIsActive = false;
    } else{
      hubIsActive = true;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("HubActive", hubIsActive);
    SmartDashboard.putBoolean("Did We WIN?", didWeWin);
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Phase Countdown", countdown);

    if(timer.get() >= 10 && !transitionPassed){
      hubIsActive = !didWeWin;
      transitionPassed = true;
      setCountdown();
    }
    if(timer.get() >= 35 && !shift1Passed){
      swapHubActivitiy();
      shift1Passed = true;
      setCountdown();
    }
    if(timer.get() >= 60 && !shift2Passed){
      swapHubActivitiy();
      shift2Passed = true;
      setCountdown();
    }
    if(timer.get() >= 85 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
      setCountdown();
    }
    if(timer.get() >= 110 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
      setCountdown();
    }
    if(timer.get() >= 135 && !shift4Passed){
      hubIsActive = true;
      shift4Passed = true;
       setCountdown();
    }
    
    
    if(countdownTimer.get() >= 1){
      countdown--;
      startCountdown();
    }
  }
}
