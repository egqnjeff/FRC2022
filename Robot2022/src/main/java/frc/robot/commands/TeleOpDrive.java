// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleOpDrive extends CommandBase {
  /** Creates a new TeleOpDrive. */
  public TeleOpDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/** Original H

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 

class TeleOpDrive
    : public frc2::CommandHelper<frc2::CommandBase, TeleOpDrive> {
 public:
  explicit TeleOpDrive(DriveTrain *drivetrain,
                       std::function<double()> speed,
                       std::function<double()> rotation);

#ifdef ENABLE_DRIVETRAIN
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
#endif // ENABLE_DRIVETRAIN

 private:
  DriveTrain *m_driveTrain;
  std::function<double()> m_speed;
  std::function<double()> m_rotation;
  // Digital filter outputs
  double m_speedOut = 0.0;
  double m_rotationOut = 0.0;
};

 */

/** Original CPP

#include "commands/TeleOpDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>

TeleOpDrive::TeleOpDrive(DriveTrain *drivetrain,
                         std::function<double()> speed,
                         std::function<double()> rotation)
            : m_driveTrain(drivetrain),
              m_speed(speed),
              m_rotation(rotation) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drivetrain);
}

#ifdef ENABLE_DRIVETRAIN
// Called when the command is initially scheduled.
void TeleOpDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleOpDrive::Execute() {
  // Digital filter lengths- between 1.0 (no filter) and 20.0 (90% at 1 second) (11.0 is 90% at 0.5 sec)
  // Idea from simple filter at https://www.chiefdelphi.com/t/moderating-acceleration-deceleration/77960/4

  // Get adjustment values
  double speedN = m_driveTrain->m_nte_DriveSpeedFilter.GetDouble(10.0);
  double rotationN = m_driveTrain->m_nte_DriveRotationFilter.GetDouble(8.0);
  if (speedN < 1.0) { speedN = 1.0; }
  if (rotationN < 1.0) { rotationN = 1.0; }
  double exponent = m_driveTrain->m_nte_InputExponent.GetDouble(1.0);
  if (exponent < 1.0) { exponent = 1.0; }
  if (exponent > 3.0) { exponent = 3.0; }

  // Adjust input speed with exponentiation
  double speed = m_speed();
  double adjustedSpeed = copysign(pow(fabs(speed), exponent), speed);

  // Adjust input speed and input rotation with filters
  speed = (((speedN - 1.0) * m_speedOut) + adjustedSpeed) / speedN;
  double rotation = (((rotationN - 1.0) * m_rotationOut) + m_rotation()) / rotationN;

  // Do it
  m_driveTrain->ArcadeDrive(speed, rotation);

  // Display the distance we've driven
  m_driveTrain->m_nte_Testing.SetDouble(m_driveTrain->GetLeftDistanceInches());

  m_speedOut = speed;
  m_rotationOut = rotation;

  // Options for more accurate time:
  //frc::RobotController::GetFPGATime()/1000
  // For Linear Filters:
  // From https://docs.wpilib.org/en/latest/docs/software/advanced-control/filters/linear-filter.html#creating-a-linearfilter
  //frc::LinearFilter<double> filter = frc::LinearFilter<double>::SinglePoleIIR(0.1_s, 0.02_s);
}

// Called once the command ends or is interrupted.
void TeleOpDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleOpDrive::IsFinished() { return false; }

#endif // ENABLE_DRIVETRAIN

 */