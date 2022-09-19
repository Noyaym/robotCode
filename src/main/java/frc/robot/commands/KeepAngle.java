package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class KeepAngle extends CommandBase {
    private double goalAngle;
    private DriveSubsystem drive;
    private PIDController PID;
    private PIDController PIDV;
    private double vl;
    private double vr;
    private double corr;
    private double startAngle;

    public KeepAngle(DriveSubsystem drive, double goalAngle) {
        this.drive = drive;
        this.goalAngle = goalAngle;
        this.PID = new PIDController(0.015, 0.00, 0);
        this.PID.setSetpoint(goalAngle);
        this.PIDV = new PIDController(0.032, 0, 0);
        this.PIDV.setSetpoint(goalAngle);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        startAngle = drive.getAngle();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        vl = PIDV.calculate(drive.getAngle()-startAngle);
        vl = Math.abs(vl);
        if (vl>0.8) {
            vl = 0.8;
        }

        vr = vl;
        corr = PID.calculate(drive.getAngle()-startAngle);
        if (corr>0.4) {
            corr = 0.4;
        } else if(corr < -0.4) {
            corr = -0.4;
        }

        if (((drive.getAngle()-startAngle)-goalAngle<4) && 
        (0<(drive.getAngle()-startAngle)-goalAngle)) {
            vl-= corr;
            drive.stopM1();
            drive.stopM2();
        }
        else if (((drive.getAngle()-startAngle)-goalAngle<0) &&
         (-4<(drive.getAngle()-startAngle)-goalAngle)) {
            vr+= corr;
            drive.stopM3();
            drive.stopM4();
        }
        else {
        vr = vl + corr;
        vl -= corr; 

        }
        drive.setV(vl, vr);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub;
        return Math.abs((drive.getAngle()-startAngle)-goalAngle)<1;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
        drive.setCoast();
    }
    
}
