// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}

/** Original H
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>

#include "OI.h"
#include "Constants.h"

#include "commands/TeleOpDrive.h"
#include "commands/AutoDrive.h"
#include "commands/AutoDriveDistance.h"
#include "commands/AutoDelay.h"
#include "commands/Deploy.h"
#include "commands/Stow.h"
#include "commands/Reject.h"
#include "commands/Launch.h"
#include "commands/Climb.h"
#include "commands/ExtendClimber.h"
#include "commands/Test.h"
#include "commands/ExampleCommand.h"

#include "subsystems/DriveTrain.h"
#include "subsystems/Launcher.h"
#include "subsystems/Intake.h"
#include "subsystems/Climber.h"
#include "subsystems/Vision.h"
#include "subsystems/LEDs.h"


 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 
class RobotContainer {
  public:
 
   RobotContainer();
   void ConfigureButtonBindings();
   void RobotInit();
   void DisabledInit();
   void DisabledPeriodic();
   void TeleopInit();
   void AutonomousInit();
 
   frc::ShuffleboardTab* m_sbt_Robot;
   nt::NetworkTableEntry m_nte_CodeVersion;
   frc2::Command* GetAutonomousCommand();
   frc2::Command* GetDisabledCommand();
 
   // The driver's game controller
   frc::XboxController driver_control{ConXBOXControl::DRIVER_CONTROLLER_PORT};
   // The codriver's game controller
   frc::XboxController codriver_control{ConXBOXControl::CODRIVER_CONTROLLER_PORT};
 
  private:
   // The robot's subsystems and commands are defined here...
   // Subsystems
   DriveTrain m_driveTrain;
   Launcher m_launcher;
   Intake m_intake;
   Climber m_climber;
   Vision m_vision;
   LEDs m_leds;
   ExampleSubsystem m_testing;
 
   // Commands...
   AutoDrive *m_autoDrive = nullptr;
 };
 
 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/StartEndCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Button.h>
#include <frc/DriverStation.h>
#include <frc/Relay.h>
#include <cameraserver/CameraServer.h>
#include "RobotContainer.h"
#include "Constants.h"

RobotContainer::RobotContainer() {

  // ANOTHER WAY OF CONSTRUCTING: m_autoDrive = AutoDrive(&m_driveTrain);
  // Initialize all of your commands and subsystems here
}

void RobotContainer::RobotInit() {

  // Configure the button bindings
  ConfigureButtonBindings();
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // #define ENABLE_FLIGHTSTICK
  #ifdef ENABLE_FLIGHTSTICK
    // Set up default drive command
    m_driveTrain.SetDefaultCommand(TeleOpDrive(
      &m_driveTrain,
      [this] { return driver_control.GetRawAxis(ConFlightControl::ELEVATOR); },
      [this] { return driver_control.GetRawAxis(ConFlightControl::RUDDER); }));

  #else // !ENABLE_FLIGHTSTICK
    // Set up default drive command
    m_driveTrain.SetDefaultCommand(TeleOpDrive(
      &m_driveTrain,
      [this] { return driver_control.GetRawAxis(ConXBOXControl::RIGHT_TRIGGER) - driver_control.GetRawAxis(ConXBOXControl::LEFT_TRIGGER); },
      // Scale turning by a 25% for at-home challenges  
      // To swap front & back of robot, add/remove a minus sign in the following line, and swap the INVERTED/NONINVERTED in DriveTrain.cpp
      [this] { return -driver_control.GetRawAxis(ConXBOXControl::LEFT_JOYSTICK_X/4); }));
  #endif

  #ifdef ENABLE_USB_CAMERA
  // Start the Camera Server
  // Get the USB camera from CameraServer
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  // Set the resolution
  camera.SetResolution(640, 480);
  camera.SetFPS(15);
  #endif
  // Create and get reference to SB tab
  m_sbt_Robot = &frc::Shuffleboard::GetTab(ConShuffleboard::RobotTab);
  // See https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/shuffleboard/layouts-with-code/using-tabs.html

  // Create widget for code version
  #define CODE_VERSION ROBOT_VERSION_STRING " " __DATE__ " " __TIME__ 
  m_nte_CodeVersion = m_sbt_Robot->Add("Code Version", CODE_VERSION).WithSize(3, 1).WithPosition(0, 0).GetEntry();
}

// Called ONCE when the robot is disabled
void RobotContainer::DisabledInit() {
#ifdef ENABLE_DRIVETRAIN
  m_driveTrain.ResetEncoders();
#endif // ENABLE_DRIVETRAIN
#ifdef ENABLE_VISION
  //m_vision.InitVision(); // Causes camera stream to freeze
#endif // ENABLE_VISION
  m_launcher.SetLaunchSoftLimits();
  m_climber.SetClimberSoftLimits();
  m_intake.Stow();
  m_leds.Init();
  m_leds.SetMode(ConLED::DISABLED);
}

// Called periodically while the robot is disabled
void RobotContainer::DisabledPeriodic() {
  m_driveTrain.SetAutonomousParameters();
  m_launcher.SetLaunchSoftLimits();
}

void RobotContainer::TeleopInit() {
  m_launcher.SetLaunchSoftLimits();
  m_launcher.SetupClose();
  m_climber.SetClimberSoftLimits();
  m_launcher.Retract();
  // Set Limelight Camera options
  m_vision.InitVision(); // Causes camera stream to freeze
  m_leds.SetMode(ConLED::TELEOP);
}


Uncomment this and add declaration in h file if we need live adjustments
also call this method  from Robot::TelopPeriodic();

void RobotContainer::TeleopPeriodic() {
  m_launcher.SetLaunchSoftLimits();
}

void RobotContainer::AutonomousInit() {
  m_driveTrain.SetAutonomousParameters();
  m_launcher.SetLaunchSoftLimits();
  m_launcher.SetupClose();
  m_climber.SetClimberSoftLimits();
#ifdef ENABLE_DRIVETRAIN
  m_driveTrain.ResetGyro();
#endif // ENABLE_DRIVETRAIN

  if (frc::DriverStation::IsFMSAttached()) {
    m_driveTrain.BurnFlash();
    m_launcher.BurnFlash();
    m_intake.BurnFlash();
    m_climber.BurnFlash();
  }
  m_leds.SetMode(ConLED::AUTONOMOUS);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
#ifdef ENABLE_DRIVETRAIN
  // Commence reduced speed driving when bumper(s) pressed
  // frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::RIGHT_BUMPER); }).WhenHeld(new TeleOpSlowDrive(&m_driveTrain));
  // frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::LEFT_BUMPER); }).WhenHeld(new TeleOpSlowDrive(&m_driveTrain));
#endif // ENABLE_DRIVETRAIN

#ifdef ENABLE_LAUNCHER
  // Duplicate Launch OI controls on both driver & codriver inputs
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::LEFT_BUMPER); }).WhileHeld(
      frc2::StartEndCommand( [&] {m_launcher.LaunchBert(1.0);}, [&] {m_launcher.RetractBert();}, {&m_launcher} ));
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::RIGHT_BUMPER); }).WhileHeld(
      frc2::StartEndCommand( [&] {m_launcher.LaunchErnie(1.0);}, [&] {m_launcher.RetractErnie();}, {&m_launcher} ));

  frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::LEFT_BUMPER); }).WhileHeld(
      frc2::StartEndCommand( [&] {m_launcher.LaunchBert(1.0);}, [&] {m_launcher.RetractBert();}, {&m_launcher} ));
  frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::RIGHT_BUMPER); }).WhileHeld(
      frc2::StartEndCommand( [&] {m_launcher.LaunchErnie(1.0);}, [&] {m_launcher.RetractErnie();}, {&m_launcher} ));

  frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::A); }).WhileHeld(
      frc2::StartEndCommand( [&] {m_launcher.Launch();}, [&] {m_launcher.Retract();}, {&m_launcher} ));
  
#if 0 // turn these off so as not to conflict with led demo
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::SELECT); }).WhenPressed(
      frc2::InstantCommand( [&] { m_launcher.SetupFar(); }, { &m_launcher }));
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::START); }).WhenPressed(
      frc2::InstantCommand( [&] { m_launcher.SetupClose(); }, { &m_launcher }));

  frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::SELECT); }).WhenPressed(
      frc2::InstantCommand( [&] { m_launcher.SetupFar(); }, { &m_launcher }));
  frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::START); }).WhenPressed(
      frc2::InstantCommand( [&] { m_launcher.SetupClose(); }, { &m_launcher }));
#endif
#endif

#ifdef ENABLE_LED_DEMO
frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::SELECT); }).WhenPressed(
    frc2::InstantCommand( [&] { m_leds.On(); }, { &m_leds }));
frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::START); }).WhenPressed(
    frc2::InstantCommand( [&] { m_leds.Off(); }, { &m_leds }));
#endif // ENABLE_LED_DEMO

#ifdef ENABLE_INTAKE
  // Duplicate Intake OI controls on both driver & codriver inputs
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::A); }).ToggleWhenPressed(
      frc2::StartEndCommand( [&] {m_intake.Deploy();}, [&] {m_intake.Stow();}, {&m_intake} ));
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::B); }).WhileHeld(
    frc2::StartEndCommand( [&] {m_intake.Reject(); }, [&] {m_intake.Load();} ));

  // Disableing intake control for co driver
  // frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::A); }).ToggleWhenPressed(
     // frc2::StartEndCommand( [&] {m_intake.Deploy();}, [&] {m_intake.Stow();}, {&m_intake} ));
  // frc2::Button([this] { return codriver_control.GetRawButton(ConXBOXControl::B); }).WhileHeld(
      // frc2::StartEndCommand( [&] {m_intake.Reject(); }, [&] {m_intake.Load();} ));
#endif

#ifdef ENABLE_CLIMBER
  frc2::Button([this] {return codriver_control.GetRawButton(ConXBOXControl::X); }).WhileHeld(new Climb(&m_climber));
  frc2::Button([this] {return codriver_control.GetRawButton(ConXBOXControl::Y); }).WhileHeld(new ExtendClimber(&m_climber));
#endif // ENABLE_CLIMBER

#ifdef ENABLE_TESTING
// Servo Test
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::X); }).WhenPressed(Test(&m_testing, 1));
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::Y); }).WhenPressed(Test(&m_testing, 0));
#endif

#ifdef ENABLE_TESTING
  frc2::Button([this] { return driver_control.GetRawButton(ConXBOXControl::SELECT); }).WhenPressed(
    frc2::InstantCommand( [&] {m_vision.ToggleLight();}, { &m_vision } ));

  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::X); }).ToggleWhenPressed(
      frc2::StartEndCommand( [&] {m_vision.LightBlink();}, [&] {m_vision.LightOff();}, {&m_vision} ));

   PiP Not working: Freezes stream
  frc2::Button([this] {return driver_control.GetRawButton(ConXBOXControl::Y); }).WhenPressed(
      frc2::InstantCommand( [&] {m_vision.PiPStream();}, {&m_vision} ));
  

#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  if (m_autoDrive != nullptr) {
    m_autoDrive->Cancel();
    m_autoDrive = nullptr;
  }  
  m_autoDrive = new AutoDrive(&m_driveTrain, &m_launcher, &m_intake);
  return m_autoDrive;
}

 */
