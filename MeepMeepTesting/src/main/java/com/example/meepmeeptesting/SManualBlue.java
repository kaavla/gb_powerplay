package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class SManualBlue implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 18, 63, Math.toRadians( 0 ) ) )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.waitSeconds(1)
				//.lineToSplineHeading(new Pose2d(-9,42 , Math.toRadians(82)))
				.setTangent( Math.toRadians( 190) )
				.splineToLinearHeading( new Pose2d(-9,42 , Math.toRadians(82)), Math.toRadians(262 ) )

				.setTangent( Math.toRadians( 270 ) )
				//.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				//.splineToLinearHeading( new Pose2d( 63.9, 9.5,  Math.toRadians(30 ) ), Math.toRadians( 200 ) )

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				.setTangent( Math.toRadians( 90) )
				//.splineToSplineHeading( new Pose2d( 18, 63, Math.toRadians( 0 ) ), Math.toRadians( 10) )
				.lineToLinearHeading( new Pose2d( 0, 63, Math.toRadians( 0 ) ))

				.forward(10)
				//.splineToLinearHeading( MeepMeepPath.getHubPositionX( -45, 90, 8, true ), Math.toRadians( 0  ) )
				//.splineToLinearHeading( new Pose2d( 63.9, 9.5,  Math.toRadians(30 ) ), Math.toRadians( 200 ) )

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )



				.build( );
	}



}
