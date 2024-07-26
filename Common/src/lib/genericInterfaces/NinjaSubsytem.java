// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.genericInterfaces;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.NinjaMotorController;

public abstract class NinjaSubsystem extends SubsystemBase {

  
  protected NinjaMotorController _master;
  RobotStates _currentState;
  
  /** Creates a new NinjaSubsytem. */
  public NinjaSubsytem(NinjaMotorController _master) {
    this._master = _master;
  }

  

  public Command runProfile(State pos) {
    return Commands.runOnce(() -> _master.set(pos), this);
  }
  
  /**
   * Run motors with percentage using {@link StartEndCommand},
   * on the starts setting motors to the given percentage and at the end stopping the motors.
   * @param percentage amount to give to the motor group
   * @return command to run
   */
  public Command runMotors(double percentage) {
    return Commands.startEnd(
        () -> _master.set(percentage),
        () -> _master.stop(),
        this);
  }

  public Command close() {
    return runProfile(new State(0, 0));
  }
  public void setState(RobotStates newState){
    _currentState = newState;
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
    _master.writePeriodicOutputs();
    _master.outputTelemetry(DriverStation.isTest());
    switch (_currentState) {
      case INTAKE:
        intake();
        break;
      case STOP:
        stop();
        break;
      case MANUAL:
        manual();
        break;
      case RESET:
        reset();
        break;
      default:
        idle();
        break;
    }
    super.periodic();
  }

  public abstract void intake();
  public abstract void idle();
  public abstract void manual();
  public abstract void reset();

  public void stop(){
    _master.stop();
  }
}
