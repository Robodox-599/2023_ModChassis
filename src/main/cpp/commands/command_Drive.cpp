// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Drive.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_Drive::command_Drive(ExampleSubsystem* subsystem, std::function<double()> xSpeed, std::function<double()> zRot):
m_subsystem{subsystem},
                              m_xSpeed{xSpeed},
                              m_zRot{zRot} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_subsystem});
}

// Called when the command is initially scheduled.
void command_Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_Drive::Execute() {
  frc::SmartDashboard::PutNumber("X", m_xSpeed());
  frc::SmartDashboard::PutNumber("Z", m_zRot());
  // if(m_xSpeed() < 0.1){
  //   m_xSpeed = [this]{return 0.0;};
  // }
  // if(m_zRot() < 0.1){
  //   m_zRot = [this]{return 0.0;};
  // }
  m_subsystem->Drive(m_xSpeed() * 1_mps, m_zRot() * 1_rad_per_s);
}

// Called once the command ends or is interrupted.
void command_Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool command_Drive::IsFinished() {
  return false;
}
