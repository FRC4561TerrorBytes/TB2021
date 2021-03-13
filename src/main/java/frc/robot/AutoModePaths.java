/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;

/**
 * Add your docs here.
 */
public class AutoModePaths {

    public static Pose2d[] ShootDriveStraight = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(2, 0, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] ShootDriveBack = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] TrenchSixBallPt1 = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(5, 0, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] TrenchSixBallPt2 = { 
        new Pose2d(5, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(5, -2, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] ShowOffOne = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(2.3114, -1.4478, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] ShowOffTwo = { 
        new Pose2d(2.3114, -1.4478, Rotation2d.fromDegrees(0)),
        new Pose2d(4.6228, -1.4478, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] ShowOffThree = { 
        new Pose2d(4.6228, -1.4478, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0))
    };

    // UNTESTED
    public static Pose2d[] Shoot_Middle = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(-5, -2, Rotation2d.fromDegrees(0)) 
    };

    //UNTESTED
    public static Pose2d[] Shoot_LoadingZone = { 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(-5, -4, Rotation2d.fromDegrees(0)) 
    };

    public static Pose2d[] TestOne = {
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(90))
    };

    
    // 2021 FaH autos

    public static Pose2d[] BounceOne = { 
        new Pose2d(1.504, -2.283, Rotation2d.fromDegrees(0)),
        new Pose2d(2.3, -0.775, Rotation2d.fromDegrees(90)) 
    };
    public static Pose2d[] BounceTwo = { 
        new Pose2d(2.3, -0.775, Rotation2d.fromDegrees(-90)),
        new Pose2d(3.229, -3.328, Rotation2d.fromDegrees(-45)), //30
        new Pose2d(3.817, -3.842, Rotation2d.fromDegrees(0)), // 15
        new Pose2d(4.454, -3.285, Rotation2d.fromDegrees(45)),
        new Pose2d(4.585, -0.775, Rotation2d.fromDegrees(90)),
    };
    public static Pose2d[] BounceThree = { 
        new Pose2d(4.585, -0.775, Rotation2d.fromDegrees(0)),
        new Pose2d(5.147, -3.582, Rotation2d.fromDegrees(30)),
        new Pose2d(6.538, -3.458, Rotation2d.fromDegrees(45)),
        new Pose2d(6.865, -0.775, Rotation2d.fromDegrees(15)),
    };
    public static Pose2d[] BounceFour = { 
        new Pose2d(6.865, -0.775, Rotation2d.fromDegrees(0)),
        new Pose2d(7.386, -1.912, Rotation2d.fromDegrees(45)),
        new Pose2d(8.808, -2.066, Rotation2d.fromDegrees(0)),
    };



}


