package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class ShankBlue1 implements MeepMeepPath{

	double wallPos = 63;

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 30, 0, Math.toRadians( 90 ) ) )
				.waitSeconds(2)
				.addTemporalMarker( ( ) -> {
					//Robot Arm to Capture Pos
				} )
				//Pos 1
				//.lineToSplineHeading(new Pose2d(-44, -47.5, Math.toRadians(90)))
				//Pos 2
				//.lineToSplineHeading(new Pose2d(-36, -47.5, Math.toRadians(90)))
				//Pos 3
				//.setTangent( Math.toRadians( 67.5 ) )
				//.splineToSplineHeading( new Pose2d( -21, -46 , Math.toRadians( 180 + 67.5 ) ), Math.toRadians( 67.5))
				.lineToSplineHeading(new Pose2d( -29.2, 33.75 , Math.toRadians( 180 - 30 )))

				.addTemporalMarker( ( ) -> {
					//Extend Robot Slides to right Height
				} )

				.addTemporalMarker( ( ) -> {
					//Draw Sides in
				} )
				.setTangent(Math.toRadians(90 ))
				.splineToLinearHeading(new Pose2d( -55, 50, Math.toRadians( 90 )), Math.toRadians(200))

				//.lineToLinearHeading(new Pose2d(-55, -50, Math.toRadians(270)))
				.addTemporalMarker( ( ) -> {
//					robot.dumpBucket( );
//					robot.lift.setDefaultHeightVel( 1200 );
				} )
				.waitSeconds( 1)
				.strafeLeft(5)
				//.waitSeconds( 1)
				//.forward(10)
				.addTemporalMarker( ( ) -> {
                  //start Carousel
				} )
				.lineToSplineHeading(new Pose2d( -29.2, 33.75 , Math.toRadians( 180 - 30 )))

				.setTangent(Math.toRadians(90 ))
				.splineToLinearHeading(new Pose2d( -65, 35, Math.toRadians( 90 )), Math.toRadians(270))

				//.lineToLinearHeading(new Pose2d(-65, -35, Math.toRadians(270)))


				.build( );




	}



}
