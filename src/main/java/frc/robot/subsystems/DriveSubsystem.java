package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Autonomi;
import frc.robot.commands.Command1;

public class DriveSubsystem extends SubsystemBase {
    private TalonFX LeftM3;
    private TalonFX LeftM4;
    private TalonFX RightM1;
    private TalonFX RightM2;
    private PigeonIMU jyro;

    public void resetEncoder() {
        this.LeftM3.setSelectedSensorPosition(0);
        this.RightM1.setSelectedSensorPosition(0);
    }

    public DriveSubsystem() {
        this.LeftM3 = new TalonFX(Constants.l3);
        this.LeftM4 = new TalonFX(Constants.l4);
        this.LeftM3.setInverted(Constants.InverTypeLeft);
        this.LeftM4.setInverted(Constants.InverTypeLeft);
        this.RightM1 = new TalonFX(Constants.r1);
        this.RightM2 = new TalonFX(Constants.r2);
        this.RightM1.setInverted(Constants.InverTypeRight);
        this.RightM2.setInverted(Constants.InverTypeRight);
        this.jyro = new PigeonIMU(Constants.jyro);
        this.LeftM4.follow(this.LeftM3);
        this.RightM2.follow(this.RightM1);
        setDefaultCommand(new Command1(this));
        LeftM3.config_kP(0, 0.000006);
        LeftM3.config_kI(0, 0.00);
        LeftM3.config_kD(0, 0);
        RightM1.config_kP(0, 0.000006);
        RightM1.config_kI(0, 0.00);
        RightM1.config_kD(0, 0);
        


    }

    public void setPower(double L, double R) {
        this.LeftM3.set(ControlMode.PercentOutput, L);
        this.RightM1.set(ControlMode.PercentOutput, R);
    }

    public void setV(double LV, double RV) {
        this.LeftM3.set(ControlMode.Velocity, LV*Constants.meterIndicator/10, 
        DemandType.ArbitraryFeedForward, Constants.ks*Math.signum(LV)+Constants.kv*LV);
        this.RightM1.set(ControlMode.Velocity, RV*Constants.meterIndicator/10, 
        DemandType.ArbitraryFeedForward, Constants.ks*Math.signum(RV)+Constants.kv*RV);
    }

    public double getLeftPosition() {
        return LeftM3.getSelectedSensorPosition()/Constants.meterIndicator;
    }

    public double getRightPosition() {
        return RightM1.getSelectedSensorPosition()/Constants.meterIndicator;
    }

    public double getAngle() {
        return jyro.getFusedHeading();
    }

    public double getVelocityL() {
        return LeftM3.getSelectedSensorVelocity()/Constants.meterIndicator*10;
    }
    public double getVelocityR() {
        return RightM1.getSelectedSensorVelocity()/Constants.meterIndicator*10;
    }



@Override
public void periodic() {
    SmartDashboard.putNumber("Left Position", getLeftPosition());
    SmartDashboard.putNumber("Right Position", getRightPosition());
    SmartDashboard.putNumber("angle", getAngle());
    SmartDashboard.putNumber("velocity", getVelocityL());
}


    
}

