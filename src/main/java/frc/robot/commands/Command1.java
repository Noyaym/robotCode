package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
//import frc.robot.subsystems.ExampleSubsystem;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Command1 extends CommandBase {


    public DriveSubsystem drive;

    public Command1(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Double L = getJSVal(RobotContainer.getMe().LJS);
        Double R =  getJSVal(RobotContainer.getMe().RJS);
        drive.setPower(L, R);
        
    }
    private double getJSVal(Joystick j) {
        double y = -j.getY();
        if ((y<Constants.range) && (y>-Constants.range)) {
            y=0.0;
        }
        double val = Math.signum(y)*((1/1.1)*Math.pow(y, 2)+(1-1/1.1));
        return val;


    }
    

    
}
