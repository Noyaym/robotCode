package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;
    
    private final AnalogInput absolutEncoder;
    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, int absEncoderID, boolean driveMotorRev,
    boolean turningMotorRev, boolean absEncoderRev, double absEncoderOffset) {
        this.absEncoderReversed = absEncoderRev;
        this.absEncoderOffsetRad = absEncoderOffset;
        absolutEncoder = new AnalogInput(absEncoderID);

        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new TalonFX(turningMotorID);

        driveMotor.setInverted(driveMotorRev);
        turningMotor.setInverted(turningMotorRev);

        turningPidController = new PIDController(0, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()/ModuleConstants.Pulse2Meter;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()/ModuleConstants.Pulse2Meter*10;
    }

    public double getTurnPosition() {
        return turningMotor.getSelectedSensorPosition()/ModuleConstants.Pulse2Meter;
    }

    public double getTurnVelocity() {
        return turningMotor.getSelectedSensorVelocity()/ModuleConstants.Pulse2Meter*10;
    }

    public double getAbsoluteEncoderRad() {
        double angle = absolutEncoder.getVoltage(); //how do i get the encoder reading?
        angle = angle*2.0*Math.PI - absEncoderOffsetRad;
        return angle*(absEncoderReversed ? -1.0: 1.0);
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.maxSpeed);
        
    
    }

    
}
