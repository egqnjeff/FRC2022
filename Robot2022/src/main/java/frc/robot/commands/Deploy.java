// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Deploy extends CommandBase {
  /** Creates a new Deploy. */
  public Deploy() {
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
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"


 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 
class Deploy
: public frc2::CommandHelper<frc2::CommandBase, Deploy> {
public:
Deploy(Intake *);

void Initialize() override;

void Execute() override;

void End(bool interrupted) override;

bool IsFinished() override;

private:
Intake *m_intake;

};

 */

/** Original CPP
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Deploy.h"

Deploy::Deploy(Intake *intake) : m_intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  //AddRequirements(intake);
}

// Called when the command is initially scheduled.
void Deploy::Initialize() {
  printf("Command Deploy::Initialize()\n");
  m_intake->Deploy();
}

// Called repeatedly when this Command is scheduled to run
void Deploy::Execute() {
}

// Called once the command ends or is interrupted.
void Deploy::End(bool interrupted) {
  printf("Command Deploy::End()\n");
}

// Returns true when the command should end.
bool Deploy::IsFinished() {
  return true;
}

 */