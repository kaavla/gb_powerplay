package org.firstinspires.ftc.teamcode.tata.Common;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autonomousBase extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        //Empty Function
    }


//    public void init(HardwareMap hwMap, Pose2d startPose, opModeCalled op) {
//        PoseStorage.startPose = startPose;
//        op_mode_called = op;
//
//        if (startPose.getY() < 0) {
//            currSide = SideColor.Red;
//        } else {
//            currSide = SideColor.Blue;
//        }
//
//        robot = new tataMecanumDrive(hardwareMap);
//
//        if (op_mode_called == opModeCalled.AUTO) {
//            openCVMainDriver = new OpenCVDetectorDriver(hardwareMap, 200, OpenCVDetectorDriver.RobotCamera.MAIN, currSide, telemetry);
//            Thread openCVMainDriverThread = new Thread(openCVMainDriver);
//            openCVMainDriverThread.start();
//
//            sideArmDriver = new RobotSideArmDriver(hwMap, 50);
//            Thread sideArmDriverThread = new Thread(sideArmDriver);
//            sideArmDriverThread.start();
//
//        }
//
//
//        sensorDriver = new RobotSensorDriver(hwMap, 100, telemetry);
//        Thread sensorDriverThread = new Thread(sensorDriver);
//        sensorDriverThread.start();
//
//
//        inTakeDriver = new RobotIntakeDriver(hwMap, 2000, telemetry);
//        Thread intakeDriverThread = new Thread(inTakeDriver);
//        intakeDriverThread.start();
//
//        slideDriver = new RobotSlideDriver(hwMap, 50, true);
//        Thread slideDriverThread = new Thread(slideDriver);
//        slideDriverThread.start();
//
//        sideArmDriver = new RobotSideArmDriver(hwMap, 50);
//        Thread sideArmDriverThread = new Thread(sideArmDriver);
//        sideArmDriverThread.start();
//
//        crDriver = new RobotCaroselDriver(hwMap, 200, currSide);
//        Thread crDriverThread = new Thread(crDriver);
//        crDriverThread.start();
//
//        frDriver = new RobotFrontServoDriver(hwMap, 2000);
//        Thread frDriverThread = new Thread(frDriver);
//        frDriverThread.start();
//
//        imuDriver = new RobotImuDriver(hardwareMap, 300, Math.toDegrees(startPose.getHeading()));
//        Thread imuDriverThread = new Thread(imuDriver);
//        imuDriverThread.start();
//
//        driver0 = new RobotLinearActuatorDriver(hardwareMap, 1000, 0);
//        Thread driverThread0 = new Thread(driver0);
//        driverThread0.start();
//
//        driver1 = new RobotLinearActuatorDriver(hardwareMap, 1000, 1);
//        Thread driverThread1 = new Thread(driver1);
//        driverThread1.start();


  //  }



    //public void stopThreads() {
//    if (op_mode_called == opModeCalled.AUTO) {
//        openCVMainDriver.stop();
//        sideArmDriver.stop();
    //}
//        sensorDriver.stop();
//        inTakeDriver.stop();
//        slideDriver.stop();
//        crDriver.stop();
//        frDriver.stop();
//        imuDriver.stop();
//        driver0.stop();
//        driver1.stop();

    //}

}


