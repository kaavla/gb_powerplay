package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

public class BlueInAutoFreight implements MeepMeepPath{

	@Override
	public TrajectorySequence getTrajectorySequence( DriveShim drive ) {
		return drive.trajectorySequenceBuilder( new Pose2d( -6.375, 62.1875, Math.toRadians( 270 ) ) )
				//move element out of the way
				.lineToLinearHeading( new Pose2d( 10 , 48, Math.toRadians( 180 ) ) )
				.addTemporalMarker( () -> {
					//robot.liftToShippingHubHeight( height );
				} )
				.strafeLeft( 30 )
				//Drop block in shipping hub
				.lineToConstantHeading( new Vector2d( 10, 24 ) )
				.lineToConstantHeading( new Vector2d( -12 , 24 ))
				.addTemporalMarker( () -> {
					//robot.dumpBucket();
					//robot.lift.setDefaultHeightVel( 1000 );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )

				//Drive over bumps
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				.lineToConstantHeading( new Vector2d( 40, 40 ) )
				//Pick up freight
				.addTemporalMarker( () -> {
					//robot.intake.setPower( -0.6 );
				} )
				.lineToLinearHeading( new Pose2d( 48, 48, Math.toRadians( 225 ) ) )
				.waitSeconds( 2 )
				.addTemporalMarker( () -> {
					//robot.intake.setPower( 0 );
				} )
				//Drive over bumps
				.lineToLinearHeading( new Pose2d( 40, 40, Math.toRadians( 180 ) ) )
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				//Drop freight in shipping hub
				.addTemporalMarker( () -> {
					//robot.liftToShippingHubHeight( RRHexBot.ShippingHubHeight.LOW );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )
				.lineToConstantHeading( new Vector2d( -12 , 24 ))
				.addTemporalMarker( () -> {
					//robot.dumpBucket();
					//robot.lift.setDefaultHeightVel( 1000 );
				} )
				.lineToConstantHeading( new Vector2d( 10, 24 ) )

				//Drive over bumps
				.lineToConstantHeading( new Vector2d( 10, 40 ) )
				.lineToConstantHeading( new Vector2d( 40, 40 ) )
				//Park
				.splineTo( new Vector2d( 60, -44 ), 0 )
				.build();
	}
}
