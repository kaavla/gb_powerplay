package org.firstinspires.ftc.teamcode.whiteout.Red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;

@Autonomous(group = "robot")
public class storageredpark extends tataAutonomousBase {
    public Pose2d startPose = new Pose2d(-39.5, -66.25, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        Trajectory traj0;
        init(hardwareMap, startPose, opModeCalled.AUTO);
        robot.setPoseEstimate(startPose);

        waitForStart();


        if (isStopRequested()) {
            stopThreads();
            return;
        }
        sleep(1000);

        inTakeDriver.intakeSet(true, true);

        autoCollectElements(10);
        //go forward
        sleep(5000);
        inTakeDriver.intakeSet(false, true);


    }
}