package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ConstantV extends CommandBase {
    private double startV;
    private double power;
    private double goalV;
    private DriveSubsystem drive;
    public ConstantV(double goalV, DriveSubsystem drive) {
        this.goalV = goalV;
        this.drive = drive;
        addRequirements(drive); }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        startV = drive.getVelocityL();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        drive.setV(goalV, goalV);
        System.out.println(drive.getVelocityL());
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return goalV-drive.getVelocityL()<=0;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        try {
            wait(2);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        drive.setV(0, 0);
    }

}