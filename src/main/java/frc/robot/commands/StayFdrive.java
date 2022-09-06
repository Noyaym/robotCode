package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StayFdrive  extends CommandBase{
    private double startDis;
    private double vr;
    private double vl;
    private double goaldis;
    private DriveSubsystem drive;
    private PIDController PID1;
    private PIDController PID2;
    private double startAngle;
    private double corr;

    public StayFdrive(double goaldis, DriveSubsystem drive) {
        this.drive=drive;
        this.goaldis=goaldis;
        this.PID1 = new PIDController(1, 0, 0);
        this.PID1.setSetpoint(goaldis);
        PID2 = new PIDController(0.01, 0, 0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        startDis = drive.getLeftPosition();
        startAngle = drive.getAngle();
        PID2.setSetpoint(startAngle);
    }

    public void execute() {
        // TODO Auto-generated method stub
        vl = PID1.calculate(drive.getLeftPosition()-startDis);
        corr = PID2.calculate(Math.abs(drive.getAngle()));
        vr = vl; 
        if (startAngle-drive.getAngle()>0) {
            vl = vl -corr;
        }
        if (startAngle-drive.getAngle()<0) {
            vr = vr -corr;
        }
        drive.setV(vl, vr);

        System.out.println(drive.getVelocityL()+ " " + drive.getAngle());
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return drive.getLeftPosition()-this.startDis>=goaldis;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
    }
    
}
