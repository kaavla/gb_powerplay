package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class A5Parking implements MeepMeepPath{

    //blue start
    Pose2d A5StartPos =new Pose2d( 24 + (robotWidth/2),72 - (robotLength/2), Math.toRadians(270));

    //blue end
    Pose2d A5EndPos1 = new Pose2d(48 - (robotWidth/2), 48 + (robotLength/2), 90);
    Pose2d A5EndPos2 = new Pose2d(24 - (robotWidth/2), 48 + (robotLength/2), 90);
    Pose2d A5EndPos3 = new Pose2d( 0 - (robotWidth/2), 48 + (robotLength/2), 90);

    @Override
    public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
        return drive.trajectorySequenceBuilder(A5StartPos)

                //get to A5EndPos1 (zone 1) without bumping into anything
                .strafeLeft(30)
                .forward(30)
                .setTangent(Math.toRadians(90))
                .waitSeconds(2)

                //get to A5EndPos2 (zone 2) without bumping into anything
                .forward(30)
                .setTangent(Math.toRadians(90))
                .waitSeconds(2)

                //get to A5EndPos3 (zone 3) without bumping into anything
                .strafeRight(30)
                .forward(30)
                .setTangent(Math.toRadians(90))
                .waitSeconds(2)

                .build( );
    }
}
