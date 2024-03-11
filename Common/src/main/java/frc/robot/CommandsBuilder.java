package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandsBuilder {
    public Command runIntake(NinjaMotorController angle, NinjaMotorController rollers){
        return Commands.run(() -> {
            angle.set(new State());
            rollers.set(0);
        }, angle,rollers);
    }

    public Command elevatorUp(NinjaMotorController elevator){
        return Commands.run(() -> {
            elevator.set(new State());
            
        }, elevator);
    }
}
