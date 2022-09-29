package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterExperiment;

public class setVel extends CommandBase {

    private double velocity;
    private double power;
    private ShooterExperiment shoot;

    public setVel(ShooterExperiment shoot, double power, double velocity) {
        this.velocity = velocity;
        this.power = power;
        this.shoot = shoot;
        addRequirements(shoot);
        
    }


    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        shoot.setP(power);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (shoot.getVelocity()>=velocity);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        shoot.setP(0);
    }
    
}
