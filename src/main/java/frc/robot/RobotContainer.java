// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomi;
import frc.robot.commands.ConstantV;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.KeepAngle;
import frc.robot.commands.StayFdrive;
import frc.robot.commands.Turn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * 
   * 
   */




   

  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    return s.getSelected();

  }


  private static RobotContainer me;
  public Joystick LJS;
  public Joystick RJS;
  public DriveSubsystem drive;
  private SendableChooser<Command> s = new SendableChooser<Command>();
  
  public RobotContainer() {
    me=this;
    this.LJS = new Joystick(Constants.LJST);
    this.RJS = new Joystick(Constants.RJST);
    this.drive = new DriveSubsystem();
    initChooser();


  }

  private void initChooser() {
    s.setDefaultOption("drive forward", new DriveForward(1, drive));
    s.addOption("drive forward stay focused", new StayFdrive(1, 0.8, drive));
    s.addOption("turn", new KeepAngle(drive, 90));
    SmartDashboard.putData("autoChooser", s);
  }
  public static RobotContainer getMe() {
    return me;
  }





}
