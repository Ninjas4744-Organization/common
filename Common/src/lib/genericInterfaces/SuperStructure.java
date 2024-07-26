// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.genericInterfaces;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {

  NinjaSubsytem[] subsystems;
  RobotStates _currentState;
  /** Creates a new SuperStructure. */
  public SuperStructure(NinjaSubsytem... systems) {
    subsystems = systems;
  }
  public enum RobotStates{
    INTAKE,
    IDLE,
    STOP,
    MANUAL,
    RESET,
  }
  @Override
  public void periodic() {
    for(NinjaSubsystem sys:subsystems){
      sys.setState(_currentState);
    }
  }



  
}
