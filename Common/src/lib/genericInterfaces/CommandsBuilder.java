package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class CommandsBuilder {
    public Command runIntake(NinjaSubsytem angle, NinjaSubsytem rollers){
        return Commands.run(() -> {
            angle._master.set(new State());
            rollers._master.set(0);
        }, angle,rollers);
    }

    public Command elevatorUp(NinjaSubsytem elevator){
        return Commands.run(() -> {
            elevator._master.set(new State());
            
        }, elevator);
    }
}
