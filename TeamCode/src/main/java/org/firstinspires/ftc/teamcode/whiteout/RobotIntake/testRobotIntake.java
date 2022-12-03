package org.firstinspires.ftc.teamcode.whiteout.RobotIntake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "testRobotIntake", group = "Test - TATA")
//@Disabled
public class testRobotIntake extends LinearOpMode {
    public RobotIntakeDriver driver;

    private double motor_power = 1.0;
    @Override
    public void runOpMode() {
        driver = new RobotIntakeDriver(hardwareMap, 200, telemetry);
        Thread driverThread = new Thread(driver);
        driverThread.start();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //driver.checkGamePad(gamepad1);
            driver.intakeSet(true, true);
            //driver.toggleIntake(true);
            sleep(6000);
            //driver.toggleIntake(true);
            driver.intakeSet(false, true);
            sleep(1000);

            idle();
        }
        driver.stop();
    }
}

