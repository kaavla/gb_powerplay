package org.firstinspires.ftc.teamcode.whiteout;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import org.firstinspires.ftc.teamcode.whiteout.RobotCarousel.RC.RobotCaroselDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotDrivetrain.RobotDriveTrainDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotFrontServo.RobotFrontServoDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotIntake.RobotIntakeDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotSensors.RobotSensorDriver;
import org.firstinspires.ftc.teamcode.whiteout.RobotSlide.RobotSlideDriver;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class whiteoutBotHW {
    public RobotDriveTrainDriver dtDriver;
    public RobotSensorDriver sensorDriver;
    public RobotIntakeDriver inTakeDriver;
    public RobotSlideDriver  slideDriver;
    //public RobotArmDriver    armDriver;
    public RobotCaroselDriver crDriver;
    public RobotFrontServoDriver frDriver;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        RobotLog.ii("CAL", "Enter - init");

        dtDriver = new RobotDriveTrainDriver(hwMap, 50);
        Thread dtDriverThread = new Thread(dtDriver);
        dtDriverThread.start();

        sensorDriver = new RobotSensorDriver(hwMap, 100, telemetry);
        Thread sensorDriverThread = new Thread(sensorDriver);
        sensorDriverThread.start();

        inTakeDriver = new RobotIntakeDriver(hwMap, 2000, telemetry);
        Thread intakeDriverThread = new Thread(inTakeDriver);
        intakeDriverThread.start();

        slideDriver  = new RobotSlideDriver(hwMap, 50, false);
        Thread slideDriverThread = new Thread(slideDriver);
        slideDriverThread.start();

        //armDriver    = new RobotArmDriver(hwMap, 50);
        //Thread armDriverThread = new Thread(armDriver);
        //armDriverThread.start();

        crDriver     = new RobotCaroselDriver(hwMap, 200, tataAutonomousBase.SideColor.Blue);
        Thread crDriverThread = new Thread(crDriver);
        crDriverThread.start();

        frDriver     = new RobotFrontServoDriver(hwMap, 2000);
        Thread frDriverThread = new Thread(frDriver);
        frDriverThread.start();

    }
    public void stopThreads() {
        dtDriver.stop();
        sensorDriver.stop();
        inTakeDriver.stop();
        slideDriver.stop();
        //armDriver.stop();
        crDriver.stop();
        frDriver.stop();
    }



    }
