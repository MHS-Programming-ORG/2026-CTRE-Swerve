// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubActiveCheck extends SubsystemBase {
  private boolean hubIsActive;
  public boolean didWeWin;
  private Timer timer;
  private Timer countdownTimer;
  private double countdown;
  
  private boolean transitionPassed = false; 
  private boolean shift1Passed = false; 
  private boolean shift2Passed = false; 
  private boolean shift3Passed = false; 
  private boolean shift4Passed = false; 

  public HubActiveCheck() {
    hubIsActive = true;
    didWeWin = true;
    timer = new Timer();
    countdownTimer = new Timer();
    countdown = 10;
  }

  public void setTransitionShift(){
    countdown = 10;
  }

  public void restartTimer(){
    timer.reset();
    timer.start();
  }

  public void startCountdown(){
    countdownTimer.reset();
    countdownTimer.start();
  }

  public void stopTimer(){
    timer.stop();
  }

  public void stopCountdown(){
    countdownTimer.stop();
  }

  public void setHubActivity(){
    if(didWeWin){
      didWeWin = false;
    } else{
      didWeWin = true;
    }
  }

  private void swapHubActivitiy(){
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
    SmartDashboard.putBoolean("Shift 4", shift4Passed);
    SmartDashboard.putNumber("Phase Countdown", countdown);

    if(timer.get() >= 10 && !transitionPassed){
      hubIsActive = !didWeWin;
      transitionPassed = true;
      countdown = 25;
    }
    if(timer.get() >= 35 && !shift1Passed){
      swapHubActivitiy();
      shift1Passed = true;
      countdown = 25;
    }
    if(timer.get() >= 60 && !shift2Passed){
      swapHubActivitiy();
      shift2Passed = true;
      countdown = 25;
    }
    if(timer.get() >= 85 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
      countdown = 25;
    }
    if(timer.get() >= 110 && !shift4Passed){
      hubIsActive = true;
      shift4Passed = true;
      countdown = 30;
    }

    if(countdownTimer.get() >= 1){
      countdown--;
      startCountdown();
    }
  }
}
