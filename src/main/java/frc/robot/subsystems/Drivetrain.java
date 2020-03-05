/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  CANSparkMax masterLeft = new CANSparkMax(Constants.masterLeftDrive, MotorType.kBrushless);
  CANSparkMax slaveLeft = new CANSparkMax(Constants.slaveLeftDrive, MotorType.kBrushless);
  CANSparkMax masterRight = new CANSparkMax(Constants.masterRightDrive, MotorType.kBrushless);
  CANSparkMax slaveRight = new CANSparkMax(Constants.slaveRightDrive, MotorType.kBrushless);
  SpeedControllerGroup right = new SpeedControllerGroup(masterRight, slaveRight);
  SpeedControllerGroup left = new SpeedControllerGroup(masterLeft, slaveLeft);
  DifferentialDrive robotDrive = new DifferentialDrive(left, right);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  CANEncoder leftEnc = masterLeft.getEncoder();
  CANEncoder rightEnc = masterRight.getEncoder();
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  public boolean isQuickTurn = false;
  final double DRIVE_WHEEL_DIAM_M = 0.127;
  final double DRIVE_GEARBOX_REDUCTION = 8.01;
  
  public Drivetrain() {
    slaveLeft.follow(masterLeft, false);
    slaveRight.follow(masterRight, false);

    masterLeft.setInverted(true);
    slaveLeft.setInverted(false);

    masterRight.setInverted(true);
    slaveRight.setInverted(false);

    masterLeft.setSmartCurrentLimit(40);
    masterRight.setSmartCurrentLimit(40);

    
    
    leftEnc.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION);
    rightEnc.setPositionConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / DRIVE_GEARBOX_REDUCTION); //Factor to convert RPM to m/s
    leftEnc.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / 60.0 / DRIVE_GEARBOX_REDUCTION);
    rightEnc.setVelocityConversionFactor(Math.PI * DRIVE_WHEEL_DIAM_M / 60.0 / DRIVE_GEARBOX_REDUCTION);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEnc.getPosition(), rightEnc.getPosition());
  }

  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    robotDrive.tankDrive(leftPower, rightPower, squareInputs);
  }

  public void arcadeDrive(double speed, double turn, boolean squareInputs) {
    robotDrive.arcadeDrive(speed, turn, squareInputs);
  }

  public void stopDrive() {
    robotDrive.tankDrive(0, 0);
  }

  public void curvatureDrive(double speed, double turn) {
    robotDrive.curvatureDrive(speed, turn, isQuickTurn);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
  }

  public double getHeading() {
    return gyro.getYaw();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(-rightVolts);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void toggleQuickTurn() {
    if(isQuickTurn)
      isQuickTurn=false;
    else
      isQuickTurn=true;
  }

  private void resetOdometry() {
    leftEnc.setPosition(0);
    rightEnc.setPosition(0);
    gyro.zeroYaw();
   
  }


}
