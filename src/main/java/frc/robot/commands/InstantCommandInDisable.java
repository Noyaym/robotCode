package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InstantCommandInDisable extends InstantCommand {

    public InstantCommandInDisable(Runnable toRun, SubsystemBase... requierments) {
        super(toRun, requierments);
    }
    
    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
