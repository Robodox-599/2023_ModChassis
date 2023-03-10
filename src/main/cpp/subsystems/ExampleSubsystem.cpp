// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExampleSubsystem.h"

ExampleSubsystem::ExampleSubsystem() {
  // Implementation of subsystem constructor goes here.
  m_LeftMotor.Config_kP(0, 0.01);
  m_RightMotor.Config_kP(0, 0.01);
  m_LeftMotor_2.Follow(m_LeftMotor);
  m_RightMotor_2.Follow(m_RightMotor);
}

frc2::CommandPtr ExampleSubsystem::ExampleMethodCommand() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
  return RunOnce([this] {});
}

bool ExampleSubsystem::ExampleCondition() {
  // Query some boolean state, such as a digital sensor.
  return false;
}

void ExampleSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ExampleSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

units::length::meter_t ExampleSubsystem::FalconToMeters(double Counts){
  return units::length::meter_t{ (Counts * 18.84) / ( 2048 * 10.71)};
}

units::degree_t ExampleSubsystem::FalconToDegrees(double Counts){
  return units::degree_t(Counts * ( 360.0 / (10.71 * 2048.0)));
}

void ExampleSubsystem::Drive(units::velocity::meters_per_second_t xSpeed, units::angular_velocity::radians_per_second_t zRot){
  // frc::DifferentialDriveKinematics kinematics{27_in};
  // frc::ChassisSpeeds chassisSpeeds{xSpeed, 0_mps, zRot};
  // auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);

  m_LeftMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xSpeed.value() + zRot.value());
  m_RightMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xSpeed.value() - zRot.value());


  // m_LeftMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, xSpeed.value());
  // m_RightMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, zRot.value());

}