package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterExperiment extends SubsystemBase {

    private TalonFX motor;
    private PIDController PID;

    public ShooterExperiment() {
        motor = new TalonFX(Constants.motorPortNum);
        PID = new PIDController(Constants.kp, Constants.ki, Constants.kd);
        motor.config_kD(0, Constants.kdMotor);
        motor.config_kP(0, Constants.kpMotor);
        motor.config_kI(0, Constants.kiMotor);

    }

    public void setP(double p) {
        motor.set(ControlMode.PercentOutput, p);
    }

    public void setV(double v) {
        motor.set(ControlMode.Velocity, v*Constants.meterIndicatorS/10, 
        DemandType.ArbitraryFeedForward, Constants.shooterKs*Math.signum(v)+Constants.shooterKv*v);
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity()/Constants.meterIndicatorS*10;
    }



    public void periodic() {
        SmartDashboard.putNumber("velocity", getVelocity());
    }



    
}
