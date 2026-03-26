// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubActiveCheck extends SubsystemBase {
  public boolean hubIsActive;
  public BooleanSupplier didWeWin;
  public boolean win;
  public Timer timer;
  public Timer countdownTimer;
  public double countdown;

  boolean transitionPassed = false; 
  boolean shift1Passed = false; 
  boolean shift2Passed = false; 
  boolean shift3Passed = false; 
  boolean endgamePassed = false; 

  public HubActiveCheck(BooleanSupplier didWeWin) {
    hubIsActive = true;
    this.didWeWin = didWeWin;
    win = true;
    timer = new Timer();
    countdownTimer = new Timer();
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

  public void setTransitionShift(){
    hubIsActive = true;
  }

  public void setHubActivity(){
    if(!didWeWin.getAsBoolean()){
      win = false;
    }
    else{
      win = true;
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
    SmartDashboard.putBoolean("Did We WIN?", win);
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("Phase Countdown", countdown);

    if(timer.get() >= 10 && !transitionPassed){
      hubIsActive = !win;
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
    if(timer.get() >= 110 && !shift3Passed){
      swapHubActivitiy();
      shift3Passed = true;
      countdown = 30;
    }
    if(timer.get() >= 135 && !endgamePassed){
      hubIsActive = true;
      endgamePassed = true;
    }
    
    
    if(countdownTimer.get() >= 1){
      countdown--;
      startCountdown();
    }
  }
}
