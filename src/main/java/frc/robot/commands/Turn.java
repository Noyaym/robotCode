package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Turn extends CommandBase{

    private DriveSubsystem drive;
    private double power;
    private double degree;
    private double start;

    public Turn(double degree, double power, DriveSubsystem drive) {
        this.degree = degree;
        this.power = power;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        start = drive.getAngle();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if (Math.signum(degree)==-1) {
            drive.setPower(power, -power);
        }
        else {
            drive.setPower(-power, power);
        }
        System.out.println(drive.getAngle() + " " + power + " " + start);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        System.out.println(drive.getAngle() - start <=degree);
        return Math.abs(drive.getAngle() - start)>=Math.abs(degree);

    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
    }
    
}
