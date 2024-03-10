// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NinjasSubsytem extends SubsystemBase {
  /** Creates a new NinjasSubsytem. */
  
 
  public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {}
    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();

    // Optional pattern for checking if attached devices have healthy configurations
    public boolean checkDeviceConfiguration() {
        return true;
    };

    // Optional pattern for checking if attached devices have healthy configurations
    public void rewriteDeviceConfiguration() {};

    
     @Override
  public void periodic() {
   writePeriodicOutputs();
   outputTelemetry();
  }

}
