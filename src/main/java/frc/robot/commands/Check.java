package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Check extends CommandBase {

    private double power;
    private DriveSubsystem drive;


    public Check(double power, DriveSubsystem drive) {
        this.power = power;
        this.drive = drive;
    }
    
    @Override
    public void initialize() {
        drive.setPower(power, power);
        new WaitCommand(2);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("velocity:" + drive.getVelocityL());
        drive.setPower(0, 0);
        drive.setCoast();
    }
}
