package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class F2Parking implements MeepMeepPath{

	//red start
	Pose2d F2StartPos = new Pose2d( -24 - (robotWidth/2),-72 + (robotLength/2), Math.toRadians(90));

	//red end
	Pose2d F2EndPos1 = new Pose2d(-48 - (robotWidth/2), -48 + (robotLength/2), 90);
	Pose2d F2EndPos2 = new Pose2d(-24 - (robotWidth/2), -48 + (robotLength/2), 90);
	Pose2d F2EndPos3 = new Pose2d( 0 - (robotWidth/2), -48 + (robotLength/2), 90);

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder(F2StartPos)

				//get to F2EndPos1 (zone 1) without bumping into anything
				.strafeLeft(30)
				.forward(30)
				.setTangent(Math.toRadians(90))
				.waitSeconds(2)

				//get to F2EndPos2 (zone 2) without bumping into anything
				.forward(30)
				.setTangent(Math.toRadians(90))
				.waitSeconds(2)

				//get to F2EndPos3 (zone 3) without bumping into anything
				.strafeRight(30)
				.forward(30)
				.setTangent(Math.toRadians(90))
				.waitSeconds(2)

				.build( );
	}
}
