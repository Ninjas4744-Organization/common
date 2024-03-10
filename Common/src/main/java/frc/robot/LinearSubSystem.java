// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class LinearSubSystem extends GenericNeoSubsystem{
      private DigitalInput _limitSwitch;

    public LinearSubSystem(NinjaMotorSubsystemConstants constants) {
        super(constants);
        _limitSwitch = new DigitalInput(0);

    }

    public boolean isLimitSwitch() {
        return !_limitSwitch.get();
      }

    
    @Override
  public void periodic() {
    if(isLimitSwitch())
      zeroSensors();

    if(isLimitSwitch() && get() < 0)
      stop();

      
  }

  public Command setPoint(double _pos){
    return Commands.runOnce(() -> {
        set(new State(1,0));
    }, this);
  }

   public Command Reset(){
    return Commands.startEnd(
      () -> {set(-0.5);},
      () -> {stop();},
      this
    ).until(() -> {return isLimitSwitch();});
  }

@Override
public void outputTelemetry(boolean testing) {

    // enter new loggers
    super.outputTelemetry(testing);
}

    
}
