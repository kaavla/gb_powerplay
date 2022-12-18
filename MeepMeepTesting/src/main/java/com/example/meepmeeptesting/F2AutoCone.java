package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class F2AutoCone implements MeepMeepPath{

    //red start
    Pose2d F2StartPos = new Pose2d( -24 - (robotWidth/2),-72 + (robotLength/2), Math.toRadians(90));

    //red end
    Pose2d F2EndPos1 = new Pose2d(-48 - (robotWidth/2), -48 + (robotLength/2), 90);
    Pose2d F2EndPos2 = new Pose2d(-24 - (robotWidth/2), -48 + (robotLength/2), 90);
    Pose2d F2EndPos3 = new Pose2d( 0 - (robotWidth/2), -48 + (robotLength/2), 90);

    @Override
    public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
        return drive.trajectorySequenceBuilder(F2StartPos)

                //

                .build( );
    }
}
