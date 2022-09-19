package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BreakouCoast extends CommandBase {

    private boolean BorC;
    private DriveSubsystem drive;

    public BreakouCoast(boolean BorC, DriveSubsystem drive) {
        this.BorC = BorC;
        this.drive = drive;
    }

    @Override
    public void execute() {
        if (BorC) {
            drive.setBrake();

        }
        else {
            drive.setCoast();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    public boolean getMode() {
        return this.BorC;
    }


    
}