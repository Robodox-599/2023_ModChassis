// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>



RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // m_subsystem.SetDefaultCommand(command_Drive(&m_subsystem,
  //                                               [this]{return m_driverController.GetRawAxis(ControllerConstants::xboxLYAxis);},
  // [this]{return -m_driverController.GetRawAxis(ControllerConstants::xboxRXAxis);}));

    m_subsystem.SetDefaultCommand(command_Drive(&m_subsystem,
                                                [this]{return m_driverController.GetRawAxis(ControllerConstants::xboxLXAxis);},
                                                [this]{return  m_driverController.GetRightTriggerAxis() - m_driverController.GetLeftTriggerAxis();}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
