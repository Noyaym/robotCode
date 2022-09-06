package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;



public class Autonomi extends CommandBase {

    private DriveSubsystem drive;
    private double dist;
    private double power;
    private double start;
    

    public Autonomi(double dist, double power, DriveSubsystem drive) {
        this.dist = dist;
        this.power = power;
        this.drive=drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        start = drive.getLeftPosition();
        
    }

    @Override
    public void execute() {
            drive.setPower(power, power);
        
    }

    @Override
    public boolean isFinished() {
        return drive.getLeftPosition()-this.start>=this.dist/3;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
    }

    @Override
    protected void finalize() throws Throwable {
        // TODO Auto-generated method stub
        super.finalize();
    }

}