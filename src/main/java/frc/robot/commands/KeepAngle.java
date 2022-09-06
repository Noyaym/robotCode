package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class KeepAngle extends CommandBase {
    private double goalAngle;
    private double angl;
    private double power;
    private DriveSubsystem drive;
    private PIDController PID;

    public KeepAngle(DriveSubsystem drive, double goalAngle) {
        this.drive = drive;
        this.goalAngle = goalAngle;
        this.PID = new PIDController(0.0044, 0.00, 0);
        this.PID.setSetpoint(goalAngle);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        angl=drive.getAngle();
        if (angl>360) {
            angl=(double)angl - (angl/360-1)*360;
        }
        if(angl<360) {
            angl=(double)angl+(angl/360-1)*360;

        }
        power = PID.calculate(angl);
        if (Math.abs(goalAngle-angl)>3) {
            if (goalAngle-angl>0) {
                drive.setPower(-power, power);
            }
            else {
                drive.setPower(power, -power);
            }
            System.out.println(angl);

        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return Math.abs(goalAngle-drive.getAngle())<=3;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
    }
    
}
