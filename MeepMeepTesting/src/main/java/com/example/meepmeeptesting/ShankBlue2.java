package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

//WareHouse Path
public class ShankBlue2 implements MeepMeepPath{

	double wallPos = 63;
	Pose2d dropPos =new Pose2d( -0.4,51.5 , Math.toRadians(67.5));

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( 6.75, 61, Math.toRadians( 90 ) ) )
				.setVelConstraint( new MinVelocityConstraint( Arrays.asList(new AngularVelocityConstraint(40 ), new MecanumVelocityConstraint( 60,14.1) ) ) )

				.addTemporalMarker(() -> {
					//moveSlideToPos(lvl, SlideDirection.OUT);
				})
				.setTangent(Math.toRadians(180 + 67.5))
				.splineToLinearHeading(dropPos, Math.toRadians(180+ 67.5))
				.back(12)
				.waitSeconds(0.5)
				.addTemporalMarker( ( ) -> {
					//slideDriver.dropGameElement();
				} )
				.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					//moveSlideToPos(lvl, SlideDirection.IN);
				} )

				//Go to Collect 1st Element
				.setTangent( Math.toRadians( 75) )
				.lineToLinearHeading( new Pose2d( 4, wallPos , Math.toRadians( 0 ) ))
				.lineToConstantHeading( new Vector2d( 42, 63 ) )
				.addTemporalMarker( ( ) -> {
					//inTakeDriver.intakeSet(true, true);
				} )
				.addTemporalMarker( ( ) -> {
					//inTakeDriver.intakeSet(true, false);
				} )

				.addTemporalMarker( ( ) -> {
					//stop intake
					//inTakeDriver.intakeSet(false, false);
				} )
				.waitSeconds(0.2)
				.addTemporalMarker( ( ) -> {
					//custommoveSlideToPos(lvl, SlideDirection.OUT, 5.0, 0.0);
					//moveSlideToPos(1, SlideDirection.OUT);

				} )

				.setTangent(Math.toRadians(180 ))
				.lineToConstantHeading( new Vector2d( 4, 63 ) )
				//.splineToLinearHeading(new Pose2d( -2.22, 42, Math.toRadians( 67.5 )), Math.toRadians(180+67.5))
				.lineToLinearHeading( new Pose2d( -2.22, 42 , Math.toRadians( 67.5 ) ))
				//.splineToLinearHeading(new Pose2d( 2.4, 51.5, Math.toRadians( 67.5 )), Math.toRadians(180+67.5))
				//.back(14)
				//.waitSeconds(0.5)
				.addTemporalMarker( ( ) -> {
					//slideDriver.dropGameElement();
				} )
				//.waitSeconds( 1 )
				.addTemporalMarker( ( ) -> {
					//moveSlideToPos(1, SlideDirection.IN);
				} )

				//Go to Collect 1st Element
				.setTangent( Math.toRadians( 75) )
				.lineToLinearHeading( new Pose2d( 4, wallPos , Math.toRadians( 0 ) ))


				.build( );




	}



}
