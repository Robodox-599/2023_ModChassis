// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>


class ExampleSubsystem : public frc2::SubsystemBase {
 public:
  ExampleSubsystem();

  /**
   * Example command factory method.
   */
  frc2::CommandPtr ExampleMethodCommand();

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  bool ExampleCondition();
  units::length::meter_t FalconToMeters(double Counts);
  units::degree_t FalconToDegrees(double Counts);
  void Drive(units::velocity::meters_per_second_t xSpeed, units::angular_velocity::radians_per_second_t zRot);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  ctre::phoenix::motorcontrol::can::TalonSRX m_LeftMotor{1};
  ctre::phoenix::motorcontrol::can::TalonSRX m_RightMotor{3};
  ctre::phoenix::motorcontrol::can::TalonSRX m_LeftMotor_2{2};
  ctre::phoenix::motorcontrol::can::TalonSRX m_RightMotor_2{4};
  

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
