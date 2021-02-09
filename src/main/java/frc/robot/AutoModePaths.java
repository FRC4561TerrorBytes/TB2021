/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class AutoModePaths {

    public static Pose2d[] ShootDriveStraight = { new Pose2d(2, 0, Rotation2d.fromDegrees(0)) };

    public static Pose2d[] ShootDriveBack = { new Pose2d(3, 0, Rotation2d.fromDegrees(0)) };

    public static Pose2d[] TrenchSixBallPt1 = { new Pose2d(5, 0, Rotation2d.fromDegrees(0)) };

    public static Pose2d[] TrenchSixBallPt2 = { new Pose2d(5, -2, Rotation2d.fromDegrees(0)) };

    // UNTESTED
    public static Pose2d[] Shoot_Middle = { new Pose2d(-5, -2, Rotation2d.fromDegrees(0)) };

    //UNTESTED
    public static Pose2d[] Shoot_LoadingZone = { new Pose2d(-5, -4, Rotation2d.fromDegrees(0)) };

    
    // 2021 FaH autos
    
    //UNTESTED
    public static Pose2d[] PathA_RedAll = {
        new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.5, -0.76, Rotation2d.fromDegrees(-20)),
        new Pose2d(4.26, 0.526, Rotation2d.fromDegrees(90)),
        new Pose2d(7.26, 0.526, Rotation2d.fromDegrees(-70))
    };
    public static Pose2d[] PathA_RedOne = { new Pose2d(2, 0, Rotation2d.fromDegrees(0)) }; // 90
    public static Pose2d[] PathA_RedTwo = { new Pose2d(1.5, -0.76, Rotation2d.fromDegrees(-20)) }; // 1.5, -0.76, -25 //45
    public static Pose2d[] PathA_RedThree = { new Pose2d(0.76, 1.286, Rotation2d.fromDegrees(90)) }; //0.76, 2.286, 90
    public static Pose2d[] PathA_RedEnd = { new Pose2d(3, 0, Rotation2d.fromDegrees(-70)) }; //5, 0, -70
}


