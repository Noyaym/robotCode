package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {

    private double startDis;
    private double power;
    private double goalDis;
    private DriveSubsystem drive;
    private PIDController PID;
    public DriveForward(double goalDis, DriveSubsystem drive) {
        this.goalDis = goalDis;
        this.drive = drive;
        this.PID = new PIDController(0.7, 0.07, 0);
        this.PID.setSetpoint(goalDis);
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        this.startDis = drive.getLeftPosition();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        power = PID.calculate(drive.getLeftPosition()-this.startDis);
        System.out.println(power);
        System.out.println(drive.getLeftPosition()-this.startDis);
        if (power>0.35) {
            power = 0.35;
        }
        drive.setPower(power, power);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return drive.getLeftPosition()-this.startDis>=goalDis ;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        drive.setPower(0, 0);
    }
    
    
}
