package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChangePose extends CommandBase {

    private double heading;
    private double Rposition;
    private double Lposition;
    private DriveSubsystem drive;

    public ChangePose(double heading, double Rposition, double Lposition, DriveSubsystem drive) {
        this.heading = heading;
        this.Rposition = Rposition;
        this.Lposition = Lposition;
        this.drive = drive;
    }
    
    @Override
    public void execute() {
        drive.setPose(heading, Lposition, Rposition);
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        // TODO Auto-generated method stub
        return true;
    }



}
