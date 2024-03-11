// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NinjaSubsytem extends SubsystemBase {

  NinjaMotorController _master;
  public NinjaSubsytem(NinjaMotorController _master) {
    this._master = _master;
  }

  /** Creates a new NinjaSubsytem. */
  

  
  public Command runProfile(State pos) {
    return Commands.runOnce(() -> _master.set(pos), this);
  }

  public Command runMotors(double percentage) {
    return Commands.startEnd(
        () -> _master.set(percentage),
        () -> _master.stop(),
        this);
  }

  @Override
  public void periodic() {
    _master.writePeriodicOutputs();
    _master.outputTelemetry(DriverStation.isTest());
    super.periodic();
  }
}
