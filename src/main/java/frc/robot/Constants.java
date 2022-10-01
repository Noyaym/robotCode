// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int r1 = 1;
    public static final int r2 = 2;
    public static final boolean InverTypeLeft = false;
    public static final boolean InverTypeRight = true;
    public static final int l3 = 3;
    public static final int l4 = 4;
    public static final int jyro = 14;
    public static final int RJST = 0;
    public static final int LJST = 0;
    public static final double range = 0.1;
    public static final double inch = 0.0254;
    public static final double diam = 6*inch;
    public static final double peremiter = diam*Math.PI;
    public static final int gearRatio = 12;
    public static final int pulsePerRotation = 2048;
    public static final double meterIndicator = 1/peremiter*gearRatio*pulsePerRotation;
    public static final double ks=0.00307;
    public static final double kv=0.2564;
    public static final double wheelBase=0.57;



    public static final class ModuleConstants {
        public static final double SwerveWheelDiam = 0;
        public static final double SwerveWheelPeremiter = SwerveWheelDiam*Math.PI;
        public static final double gearRatioSwerve = 0;
        public static final int pulsePerRotation = 2048;
        public static final double Pulse2Meter = 1/SwerveWheelPeremiter*gearRatioSwerve*pulsePerRotation;



    }

    public static final class DriveConstants {
        public static final double maxSpeed = 2.5; //in m/s
        public static final double MAX_ANGULR_VELOCITY = Math.PI; //in r/s
    }

    //shooter+hood stuff
        public static final int motorPortNum = 3; 
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double kpMotor = 0.05;
        public static final double kiMotor = 0.005;
        public static final double kdMotor = 0;
        public static final double shooterKs = 0.02;
        public static final double shooterKv = 0.03;
        public static final double inchS = 0.0254;
    public static final double diamS = 4*inch;
    public static final double peremiterS = diam*Math.PI;
    public static final int gearRatioS = 1;
    public static final int pulsePerRotationS = 2048;
    public static final double meterIndicatorS = 1/peremiter*gearRatio*pulsePerRotation;


    


}

