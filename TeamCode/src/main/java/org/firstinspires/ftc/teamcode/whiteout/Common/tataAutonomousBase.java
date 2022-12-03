package org.firstinspires.ftc.teamcode.whiteout.Common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.whiteout.OpenCVDetector.OpenCVDetectorDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotCarousel.RC.RobotCaroselDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotFrontServo.RobotFrontServoDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotImu.RobotImuDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotImu.RobotImuParams;
import org.firstinspires.ftc.teamcode.whiteout.RobotIntake.RobotIntakeDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotLinearActuators.RobotLinearActuatorDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotSensors.RobotSensorDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotSensors.RobotSensorHW;
import org.firstinspires.ftc.teamcode.whiteout.RobotSensors.RobotSensorParams;
import org.firstinspires.ftc.teamcode.whiteout.RobotSideArm.RobotSideArmDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotSlide.RobotSlideDriver;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class tataAutonomousBase extends LinearOpMode {

    public enum SideColor {
        Red, Blue
    }

    public enum StartPos {
        Warehouse, //White Colored Box
        Storage    //Duck Side
    }

    public enum SlideDirection {
        IN,
        OUT
    }

    public enum opModeCalled {
        AUTO,
        MANUAL
    }


    public tataMecanumDrive robot;
    public RobotSensorDriver sensorDriver;
    public RobotSensorParams params;

    public RobotIntakeDriver inTakeDriver;
    public RobotSlideDriver slideDriver;
    //public RobotArmDriver armDriver;
    public RobotSideArmDriver sideArmDriver;
    public RobotCaroselDriver crDriver;
    public RobotFrontServoDriver frDriver;

    public RobotLinearActuatorDriver driver0;
    public RobotLinearActuatorDriver driver1;

    public RobotImuDriver imuDriver;
    public RobotImuParams imuParams;

    public OpenCVDetectorDriver openCVMainDriver;

    public SideColor currSide = SideColor.Blue;

    public ElapsedTime runtime = new ElapsedTime();
    public int barCodeLoc = 1;
    public RobotSensorParams dsParams = new RobotSensorParams();
    public opModeCalled op_mode_called = opModeCalled.AUTO;

    @Override
    public void runOpMode() throws InterruptedException {
        //Empty Function
    }

    public void stopMainCamera() {
        openCVMainDriver.stop();
    }

/*    public void startIntake(HardwareMap hwMap) {
        inTakeDriver = new RobotIntakeDriver(hwMap, 2000, telemetry);
        Thread intakeDriverThread = new Thread(inTakeDriver);
        intakeDriverThread.start();
    }
*/
    public void init(HardwareMap hwMap, Pose2d startPose, opModeCalled op) {
        PoseStorage.startPose = startPose;
        op_mode_called = op;

        if (startPose.getY() < 0) {
            currSide = SideColor.Red;
        } else {
            currSide = SideColor.Blue;
        }

        robot = new tataMecanumDrive(hardwareMap);

        if (op_mode_called == opModeCalled.AUTO) {
            openCVMainDriver = new OpenCVDetectorDriver(hardwareMap, 200, OpenCVDetectorDriver.RobotCamera.MAIN, currSide, telemetry);
            Thread openCVMainDriverThread = new Thread(openCVMainDriver);
            openCVMainDriverThread.start();

            sideArmDriver = new RobotSideArmDriver(hwMap, 50);
            Thread sideArmDriverThread = new Thread(sideArmDriver);
            sideArmDriverThread.start();

        }


        sensorDriver = new RobotSensorDriver(hwMap, 100, telemetry);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();


        inTakeDriver = new RobotIntakeDriver(hwMap, 2000, telemetry);
        Thread intakeDriverThread = new Thread(inTakeDriver);
        intakeDriverThread.start();

        slideDriver = new RobotSlideDriver(hwMap, 50, true);
        Thread slideDriverThread = new Thread(slideDriver);
        slideDriverThread.start();

        sideArmDriver = new RobotSideArmDriver(hwMap, 50);
        Thread sideArmDriverThread = new Thread(sideArmDriver);
        sideArmDriverThread.start();

        crDriver = new RobotCaroselDriver(hwMap, 200, currSide);
        Thread crDriverThread = new Thread(crDriver);
        crDriverThread.start();

        frDriver = new RobotFrontServoDriver(hwMap, 2000);
        Thread frDriverThread = new Thread(frDriver);
        frDriverThread.start();

        imuDriver = new RobotImuDriver(hardwareMap, 300, Math.toDegrees(startPose.getHeading()));
        Thread imuDriverThread = new Thread(imuDriver);
        imuDriverThread.start();

        driver0 = new RobotLinearActuatorDriver(hardwareMap, 1000, 0);
        Thread driverThread0 = new Thread(driver0);
        driverThread0.start();

        driver1 = new RobotLinearActuatorDriver(hardwareMap, 1000, 1);
        Thread driverThread1 = new Thread(driver1);
        driverThread1.start();


    }

    public int getMarkerPos() {
        return openCVMainDriver.getMarkerPos();

    }
    public void autoAlliancePark(double timeoutInSec){

        RobotSensorHW.DetectedColors colorTodetect = RobotSensorHW.DetectedColors.RED;
        if (currSide == SideColor.Blue) {
            colorTodetect = RobotSensorHW.DetectedColors.BLUE;
        }

        //Read Color
        dsParams = sensorDriver.getRobotSensorParams();

        //Timer
        ElapsedTime stopTimer = new ElapsedTime();

        int i = 0;

        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < timeoutInSec))
        {
            if (dsParams.c_LS == colorTodetect || dsParams.c_RS == colorTodetect) {
                RobotLog.ii("SHANK", "Red Detected ");
                break;
            }
            //Check if duck has been collected
            TrajectorySequence park = getTrajectorySequenceBuilder()
                    .forward(2)
                    .build();
            robot.followTrajectorySequence(park);

            dsParams = sensorDriver.getRobotSensorParams();
            i = i + 1;
        }

        //Correct Robot Orientation
        if (currSide == SideColor.Red) {
            imuParams = imuDriver.getRobotImuParams();
            robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 270));
        } else {
            //Blue
            imuParams = imuDriver.getRobotImuParams();
            robot.turn(-1 * Math.toRadians(imuParams.correctedHeading - 90));

        }

        //Check if duck has been collected
        TrajectorySequence park = getTrajectorySequenceBuilder()
                .forward(8.5)
                .build();
        robot.followTrajectorySequence(park);

    }

    public void autoCollectElements(double timeoutInSec){

        RobotSensorHW.DetectedColors colorTodetect = RobotSensorHW.DetectedColors.RED;
        if (currSide == SideColor.Blue) {
            colorTodetect = RobotSensorHW.DetectedColors.BLUE;
        }

        //Timer
        ElapsedTime stopTimer = new ElapsedTime();

        int i = 0;

        while(opModeIsActive() && !isStopRequested() && (stopTimer.seconds() < timeoutInSec))
        {
            if (inTakeDriver.isElementCollected() == true) {
                RobotLog.ii("SHANK", "Element  Detected ");
                break;
            }
            //Check if duck has been collected
            TrajectorySequence seek = getTrajectorySequenceBuilder()
                    .forward(2)
                    .build();
            robot.followTrajectorySequence(seek);

            //dsParams = sensorDriver.getRobotSensorParams();
            i = i + 1;
        }
        RobotLog.ii("SHANK", "i = %d ", i);

        //sleep(2000);
        //inTakeDriver.intakeSet(false, true);

    }

    public TrajectorySequenceBuilder getTrajectorySequenceBuilder() {
        return robot.trajectorySequenceBuilder(robot.getPoseEstimate());
    }
    public void lrMove(int lvl, boolean drop){
        double slideInclinePerLevel[]       = {0, 0.0, 0.0,  0.1};
        if (drop) {
            driver0.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]); //Pull up
            driver1.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]);
        }else{
            driver0.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            driver1.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
        }
    }
    public void moveSlideToPos(int lvl, SlideDirection slideDirection) {
        //0th element should be ignored as levels are 1, 2, 3
        double slideDistanceInIncPerLevel[] = {0, 4.5, 4.8, 8.5, 4.5, 5, 8.5};
        double slideInclinePerLevel[]       = {0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        if (slideDirection == SlideDirection.OUT) {
            slideDriver.moveRobotSlideBy(slideDistanceInIncPerLevel[lvl], 0);
            driver0.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]); //Pull up
            driver1.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]);
        } else {
            //IN
            driver0.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            driver1.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            slideDriver.moveRobotSlideBy(-1 * slideDistanceInIncPerLevel[lvl], 0);
        }
    }

    public void moveSlideToPosShank(int lvl, SlideDirection slideDirection) {
        //0th element should be ignored as levels are 1, 2, 3
        double slideDistanceInIncPerLevel[] = {0, 3.00, 4.75, 8.5}; //8.5
        double slideInclinePerLevel[]       = {0, 0.0, 0.0,  0.0};

        if (slideDirection == SlideDirection.OUT) {
            slideDriver.moveRobotSlideBy(slideDistanceInIncPerLevel[lvl], 0);
            driver0.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]); //Pull up
            driver1.pullLinearActuatorBy(-1 * slideInclinePerLevel[lvl]);
        } else {
            //IN
            driver0.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            driver1.pullLinearActuatorBy(slideInclinePerLevel[lvl]); //pull down
            slideDriver.moveRobotSlideBy(-1 * slideDistanceInIncPerLevel[lvl], 0);
        }
        RobotLog.ii("SHANK", "Slide moved to  lvl: %d, slide: %.2f", lvl, slideDistanceInIncPerLevel[lvl]);

    }

    public void custommoveSlideToPos(SlideDirection slideDirection, double slideDistance) {
        //0th element should be ignored as levels are 1, 2, 3
        double slideDistanceInIncPerLevel = slideDistance;

        if (slideDirection == SlideDirection.OUT) {
            slideDriver.moveRobotSlideBy(slideDistanceInIncPerLevel, 0);

        } else {
            slideDriver.moveRobotSlideBy(-1 * slideDistanceInIncPerLevel, 0);
        }
    }
/*
    target   Imu   direction to move
        0         10   CW 10    (target - imu = -10)
        0         350  CCW 10   (target - imu = 0 - (-10) = 10)

       270       280  CW 10    (target - imu = -10)
       270       260  CCW 10   (target - imu = 10)

       So whatever is returned by this functiom.. just turn in same direction
*/
    public double correctOrientationUsingImu(double targetHeading) {

        imuParams = imuDriver.getRobotImuParams();
        double correctedHeading =imuParams.correctedHeading;

        RobotLog.ii("SHANK", "targetHeading %2f, corrected heading %2f , raw %2f (%2f)",
                targetHeading, correctedHeading, imuParams.rawHeading, imuParams.correction);

        //Correction of 0 target heading because of 360 wrap around
        if (targetHeading == 0) {
            if (correctedHeading > 180) {
                correctedHeading = correctedHeading - 360;
            }
        }
        RobotLog.ii("SHANK", "(targetHeading - corrected heading )%2f", (targetHeading - correctedHeading));

        return (targetHeading - correctedHeading);
    }
    public double getDistanceFromWall(SideColor sc) {
        RobotSensorParams rsp = sensorDriver.getRobotSensorParams();
        double T;
        if (sc == SideColor.Blue) {
            T =  rsp.x_LS;
            RobotLog.ii("SHANK", "Blue LS =  %2f", T);

        } else {
            T = rsp.x_RS;
            RobotLog.ii("SHANK", "Red RS =  %2f", T);
        }

        return (T);
    }


    public double getSlideHeightByLvlInInch(int lvl) {
        switch (lvl) {
            case 1:
                return 8.0;
            case 2:
                return 10.0;
            case 3:
                return 21.0;
        }
        return 0.0;
    }

    public Pose2d getTeamMarkerCoord(SideColor sc, StartPos sp, int lvl) {
        Pose2d pose = new Pose2d(0, 0, 0);
        if (sc == SideColor.Red) {
            if (sp == StartPos.Storage) {
                switch (lvl) {
                    //red alliance side
                    case 1: {
                        pose = new Pose2d(-42, -52, Math.toRadians(90));//-45,-47.5
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, -51.5, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-29, -52, Math.toRadians(90));
                        break;
                    }
                }
            } else {
                //sp == Warehouse
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(-44.5, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36, -47.5, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(-27, -47.5, Math.toRadians(90));
                        break;
                    }
                }

            }
        } else {
            //Blue alliance side
            if (sp == StartPos.Storage) {
                switch (lvl) {
                    case 3: {
                        pose = new Pose2d(-43, 53, Math.toRadians(270));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(-36.5, 53, Math.toRadians(270));
                        break;
                    }
                    case 1: {
                        pose = new Pose2d(-28, 52, Math.toRadians(270));
                        break;
                    }
                }
            }
            else {
                //sp == Warehouse
                switch (lvl) {
                    case 1: {
                        pose = new Pose2d(4, 36, Math.toRadians(90));
                        break;
                    }
                    case 2: {
                        pose = new Pose2d(8, 36, Math.toRadians(90));
                        break;
                    }
                    case 3: {
                        pose = new Pose2d(12, 36, Math.toRadians(90));
                        break;
                    }
                }

            }

        }
        return pose;
    }


    public void stopThreads() {
    if (op_mode_called == opModeCalled.AUTO) {
        openCVMainDriver.stop();
        sideArmDriver.stop();
    }
        sensorDriver.stop();
        inTakeDriver.stop();
        slideDriver.stop();
        crDriver.stop();
        frDriver.stop();
        imuDriver.stop();
        driver0.stop();
        driver1.stop();

    }

}


