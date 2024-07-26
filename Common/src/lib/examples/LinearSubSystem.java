// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class LinearSubSystem extends NinjaSubsytem {
    private DigitalInput _limitSwitch;

    public LinearSubSystem(NinjaMotorController master) {
        super(master);
        _limitSwitch = new DigitalInput(0);
    }

    public boolean isLimitSwitch() {
        return !_limitSwitch.get();
    }

    @Override
    public void periodic() {
        
        
        if (isLimitSwitch())
            _master.zeroSensors();

        if (isLimitSwitch() && _master.get() < 0)
            _master.stop();

    }

    

    

}
