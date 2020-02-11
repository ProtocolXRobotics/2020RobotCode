/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int masterLeftDrive = 0;
    public static int slaveLeftDrive = 1;
    public static int masterRightDrive = 2;
    public static int slaveRightDrive = 3;
    public static int masterShooter = 4;
    public static int slaveShooter = 5;
    public static int turret = 6;
    public static int intakeMotor = 7;
    public static int leftIndexer = 8;
    public static int rightIndexer = 9;
    public static int climb = 10;
    // Use Drive Characterization to find these constants
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;

    public static final double trackwidth = 2;
    public static final double maxSpeed = 10;
    public static final double maxAcceleration = 4;
    public static final double ramseteB = 2;
    public static final double ramseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.trackwidth);

    public static final int retractorforward = 0;
    public static final int retractorreverse = 1;

    //Limelight
    public static final double kLimelightHeight = 24; // In inches
    public static final double kLimelightAngle = 30; // In degrees
    public static final double kPortHeight = 98.25; // In inches


   


}
