package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.fasterxml.jackson.core.json.DupDetector;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.BreakouCoast;
import frc.robot.commands.ChangePose;
import frc.robot.commands.Check;
import frc.robot.commands.Command1;
import frc.robot.commands.DriveByJoystick;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.commands.RunCommandEnable;

public class DriveSubsystem extends SubsystemBase {
    private TalonFX leftM3;
    private TalonFX leftM4;
    private TalonFX rightM1;
    private TalonFX rightM2;
    private PigeonIMU jyro;
    private DifferentialDriveOdometry odometry;
    private Field2d field2d;
    private boolean brake;
    private Pose2d pose;
    private double power;
    private final SimpleMotorFeedforward ff;

    public void resetEncoder() {
        leftM3.setSelectedSensorPosition(0);
        rightM1.setSelectedSensorPosition(0);
    }

    public DriveSubsystem() {
        leftM3 = new TalonFX(Constants.l3);
        leftM4 = new TalonFX(Constants.l4);
        leftM3.setInverted(Constants.InverTypeLeft);
        leftM4.setInverted(Constants.InverTypeLeft);
        rightM1 = new TalonFX(Constants.r1);
        rightM2 = new TalonFX(Constants.r2);
        rightM1.setInverted(Constants.InverTypeRight);
        rightM2.setInverted(Constants.InverTypeRight);
        jyro = new PigeonIMU(Constants.jyro);
        leftM4.follow(this.leftM3);
        rightM2.follow(this.rightM1);
        setDefaultCommand(new DriveByJoystick(this));
        leftM3.config_kP(0, 0.000006);
        leftM3.config_kI(0, 0.00);
        leftM3.config_kD(0, 0);
        rightM1.config_kP(0, 0.000006);
        rightM1.config_kI(0, 0.00);
        rightM1.config_kD(0, 0);
        odometry = new DifferentialDriveOdometry(getRotation());
        field2d = new Field2d();
        ff = new SimpleMotorFeedforward(Constants.ks, Constants.kv);

    }

    public void setPower(double L, double R) {
        leftM3.set(ControlMode.PercentOutput, L);
        rightM1.set(ControlMode.PercentOutput, R);
    }

    public void setV(double LV, double RV) {
        leftM3.set(ControlMode.Velocity, LV*Constants.meterIndicator/10, 
        DemandType.ArbitraryFeedForward, ff.calculate(LV));
        rightM1.set(ControlMode.Velocity, RV*Constants.meterIndicator/10, 
        DemandType.ArbitraryFeedForward, ff.calculate(RV));
    }

    public double getLeftPosition() {
        return leftM3.getSelectedSensorPosition()/Constants.meterIndicator;
    }

    public double getAverageLeftPosition() {
        double p1 = leftM3.getSelectedSensorPosition()/Constants.meterIndicator;
        double p2 = leftM4.getSelectedSensorPosition()/Constants.meterIndicator;
        return (p1+p2)/2;
    }

    public double getRightPosition() {
        return rightM1.getSelectedSensorPosition()/Constants.meterIndicator;
    }

    public double getAverageRightPosition() {
        double p1 = rightM1.getSelectedSensorPosition()/Constants.meterIndicator;
        double p2 = rightM2.getSelectedSensorPosition()/Constants.meterIndicator;
        return (double) (p1+p2)/2;
    }

    public double getAngle() {
        return jyro.getFusedHeading();
    }

    public double getVelocityL() {
        return leftM3.getSelectedSensorVelocity()/Constants.meterIndicator*10;
    }
    public double getVelocityR() {
        return rightM1.getSelectedSensorVelocity()/Constants.meterIndicator*10;
    }
    public PigeonIMU getGyro() {
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
        rightM1.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM2() {
        rightM2.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM3() {
        leftM3.setNeutralMode(NeutralMode.Brake);
    }
    public void stopM4() {
        leftM4.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoast() {
        rightM1.setNeutralMode(NeutralMode.Coast);
        rightM2.setNeutralMode(NeutralMode.Coast);
        leftM3.setNeutralMode(NeutralMode.Coast);
        leftM4.setNeutralMode(NeutralMode.Coast);

    }

    public void setBrake() {
        rightM1.setNeutralMode(NeutralMode.Brake);
        rightM2.setNeutralMode(NeutralMode.Brake);
        leftM3.setNeutralMode(NeutralMode.Brake);
        leftM4.setNeutralMode(NeutralMode.Brake);
    }

    public void resetNeutralMode() {
        System.out.println(" reset from " + brake);
        setNeutralMode(!brake);
    }
    public void setNeutralMode(boolean brake) {
        this.brake = brake;
        if(brake) {
            setBrake();
        } else {
            setCoast();
        }
        System.out.println(brake);
    }

    public boolean getBrake() {
        return brake;
    }

double p;
@Override
   public void initSendable( SendableBuilder builder) {
    SmartDashboard.putData("Reset NeutralMode", new InstantCommandInDisable(() -> {
        setNeutralMode(!brake);
    }, this));

    SmartDashboard.putNumber("Power", 0);
    p = SmartDashboard.getNumber("Power", 0);
    SmartDashboard.putData("give power", new InstantCommand(()-> {setPower(p, p);})
    .andThen(new WaitCommand(2))
    .andThen(new InstantCommand(()-> {System.out.println(getVelocityL());}))
    .andThen(new InstantCommand(()-> {setPower(0, 0);})));

    builder.addDoubleProperty("Left Position", this::getLeftPosition,null);
    builder.addDoubleProperty("Right Position", this::getRightPosition, null);
    builder.addDoubleProperty("angle", this::getAngle, null);
    builder.addDoubleProperty("velocity", this::getVelocityL, null);
    builder.addBooleanProperty("Brake", this::getBrake, null);
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

