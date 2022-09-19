package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.core.json.DupDetector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.BreakouCoast;
import frc.robot.commands.ChangePose;
import frc.robot.commands.Check;
import frc.robot.commands.Command1;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.commands.RunCommandEnable;

public class DriveSubsystem extends SubsystemBase {
    private TalonFX LeftM3;
    private TalonFX LeftM4;
    private TalonFX RightM1;
    private TalonFX RightM2;
    private PigeonIMU jyro;
    private DifferentialDriveOdometry odometry;
    private Field2d field2d;
    private boolean brake;
    private Pose2d pose;
    private double power;

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
        setDefaultCommand(new DriveByJoystick(this));
        LeftM3.config_kP(0, 0.000006);
        LeftM3.config_kI(0, 0.00);
        LeftM3.config_kD(0, 0);
        RightM1.config_kP(0, 0.000006);
        RightM1.config_kI(0, 0.00);
        RightM1.config_kD(0, 0);
        odometry = new DifferentialDriveOdometry(getRotation());
        field2d = new Field2d();
        SmartDashboard.putData("Drive", this);


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

    public double getAverageLeftPosition() {
        double p1 = LeftM3.getSelectedSensorPosition()/Constants.meterIndicator;
        double p2 = LeftM4.getSelectedSensorPosition()/Constants.meterIndicator;
        return (double) (p1+p2)/2;
    }

    public double getRightPosition() {
        return RightM1.getSelectedSensorPosition()/Constants.meterIndicator;
    }

    public double getAverageRightPosition() {
        double p1 = RightM1.getSelectedSensorPosition()/Constants.meterIndicator;
        double p2 = RightM2.getSelectedSensorPosition()/Constants.meterIndicator;
        return (double) (p1+p2)/2;
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
    public PigeonIMU getJyro() {
        return jyro;
    }

    public Pose2d getPose() {
        updatePose();
        pose = odometry.getPoseMeters();
        return pose;
    }

    public Rotation2d getRotation() {
        Rotation2d rotation2d = new Rotation2d(this.getAngle());
        return rotation2d;
    }

    public void updatePose() {
        odometry.update(getRotation(), getAverageLeftPosition() , getAverageRightPosition());
        field2d.setRobotPose(getPose());
    }

    public void setPose(double heading, double lpose, double rpose) {
        odometry.update(new Rotation2d(heading), lpose, rpose);
        field2d.setRobotPose(getPose());
    }

    public Field2d getField2d() {
        return field2d;
    }

    public void stopM1() {
        RightM1.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM2() {
        RightM2.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM3() {
        LeftM3.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM4() {
        LeftM4.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoast() {
        RightM1.setNeutralMode(NeutralMode.Coast);
        RightM2.setNeutralMode(NeutralMode.Coast);
        LeftM3.setNeutralMode(NeutralMode.Coast);
        LeftM4.setNeutralMode(NeutralMode.Coast);

    }

    public void setBrake() {
        RightM1.setNeutralMode(NeutralMode.Brake);
        RightM2.setNeutralMode(NeutralMode.Brake);
        LeftM3.setNeutralMode(NeutralMode.Brake);
        LeftM4.setNeutralMode(NeutralMode.Brake);
    }

    public void setNeutralMode(boolean brake) {
        this.brake = brake;
        NeutralMode mode = brake ? NeutralMode.Brake: NeutralMode.Coast;
        RightM1.setNeutralMode(mode);
        RightM2.setNeutralMode (mode);
        LeftM3.setNeutralMode(mode);
        LeftM4.setNeutralMode(mode);
        System.out.println(brake);

    }

    public boolean getBrake() {
        return brake;
    }


@Override
   public void initSendable( SendableBuilder builder) {
    SmartDashboard.putData("NeutralMode", new InstantCommandInDisable(() -> {setNeutralMode(!brake);}));
       builder.addDoubleProperty("Left Position", this::getLeftPosition,null);
       builder.addDoubleProperty("Right Position", this::getRightPosition, null);
       builder.addDoubleProperty("angle", this::getAngle, null);
       builder.addDoubleProperty("velocity", this::getVelocityL, null);
       //SmartDashboard.putData("pose", (Sendable) pose);
       SmartDashboard.putData("Robot Position", field2d);
       SmartDashboard.putData("Change Robot Pose", new InstantCommandInDisable
       (()-> {setPose(0, 0, 0);}, this));
   //SmartDashboard.putData("Find Ks & Kv", new RunCommandEnable(() -> {setPower(power, power);}, null )
   //.withTimeout(2).andThen(()-> System.out.println(getVelocityL())).andThen(()-> {setPower(0, 0);}));
   }





@Override
public void periodic() {
    odometry.update(getRotation(), getLeftPosition(), getRightPosition());
    field2d.setRobotPose(odometry.getPoseMeters());
}




    
}

