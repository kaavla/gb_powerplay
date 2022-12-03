package org.firstinspires.ftc.teamcode.whiteout.RobotImu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotImu", group = "Test - TATA")
//@Disabled
public class testRobotImu extends LinearOpMode {
    public RobotImuDriver driver;
    public RobotImuParams params;
    @Override
    public void runOpMode() {
        driver = new RobotImuDriver(hardwareMap, 300, 90);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            params = driver.getRobotImuParams();
            telemetry.addData("Raw:", "%2f deg ", params.rawHeading);
            telemetry.addData("Rel:", "%2f deg ", params.correctedHeading);
            telemetry.addData("Corr:", "%2f deg ", params.correction);
            telemetry.update();

            sleep(200);
            idle();
        }
        driver.stop();
    }
}

