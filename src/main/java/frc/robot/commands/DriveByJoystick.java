package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByJoystick extends CommandBase{

    private DriveSubsystem drive;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveWheelSpeeds whSpeed;

    public DriveByJoystick(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
        kinematics = new DifferentialDriveKinematics(Constants.wheelBase);
    }
    
    private double getJSYVal(Joystick j) {
        double y = -j.getY();
        double val;
        if ((y<Constants.range) && (y>-Constants.range)) {
            val=0.0;
        }
        else {
            val = Math.signum(y)*((1/1.1)*Math.pow(y, 2)+(1-1/1.1));

        }
        return val;
    }

    private double getJSXVal(Joystick j) {
        double x = j.getX();
        double val;
        if (Math.abs(x) < Constants.range) {
            val=0.0;
        }
        else {
            val = Math.signum(x)*((1/1.1)*Math.pow(x, 2)+(1-1/1.1));

        }
        return val;
    }


    @Override
    public void execute() {
        double radiansPerSec = getJSXVal(RobotContainer.getMe().LJS) * Constants.DriveConstants.MAX_ANGULR_VELOCITY;
        SmartDashboard.putNumber("Joystick x", radiansPerSec);
        double speed = getJSYVal(RobotContainer.getMe().LJS) * Constants.DriveConstants.maxSpeed;
        whSpeed = kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, radiansPerSec));
        drive.setV(whSpeed.leftMetersPerSecond, whSpeed.rightMetersPerSecond);
        
    }
    
}
