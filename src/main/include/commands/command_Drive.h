// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ExampleSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_Drive
    : public frc2::CommandHelper<frc2::CommandBase, command_Drive> {
 public:
  command_Drive(ExampleSubsystem* subsystem, std::function<double()> xSpeed, std::function<double()> zRot);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ExampleSubsystem* m_subsystem;
  std::function<double()> m_xSpeed;
  std::function<double()> m_zRot;
};
