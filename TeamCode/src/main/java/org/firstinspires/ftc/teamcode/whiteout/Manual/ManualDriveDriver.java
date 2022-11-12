package org.firstinspires.ftc.teamcode.tata.Manual;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tata.RobotDrivetrain.RobotDrivetrainHW;
import org.firstinspires.ftc.teamcode.tata.RobotDrivetrain.RobotDrivetrainParams;

public class ManualDriveDriver implements Runnable{
    private RobotDrivetrainHW dtHW = new RobotDrivetrainHW();
    private RobotDrivetrainParams params = new RobotDrivetrainParams();

    private float leftX, leftY, rightZ;
    private double motor_power = 0.8;

    //Thead run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public ManualDriveDriver(HardwareMap ahwMap, int threadSleepDelay){
        dtHW.init(ahwMap);
        sleepTime = threadSleepDelay;
    }

    private void update(){
        //empty
    }

    public RobotDrivetrainParams getRobotDrivetrainParams(){
        params.enc_LF = dtHW.getEncoderPos(1);
        params.enc_RF = dtHW.getEncoderPos(2);
        params.enc_LR = dtHW.getEncoderPos(3);
        params.enc_RR = dtHW.getEncoderPos(4);
        return params;
    }

    public void setRawPowerAll(double power) {
        dtHW.setPowerAll(power,power,power,power);
    }

    public void checkGamePad(Gamepad gp) {
        if ((gp.left_stick_y != 0) || (gp.left_stick_x != 0) || (gp.right_stick_x != 0)) {
            leftY = gp.left_stick_y*-1;
            leftX = gp.left_stick_x * 1;
            rightZ = gp.right_stick_x * -1;
            dtHW.moveHolonomic(leftX, leftY, rightZ);
        } else if (gp.dpad_up) {
            //forward
            dtHW.moveHolonomic(0, motor_power * 1, 0);
        } else if (gp.dpad_down) {
            //backwards
            dtHW.moveHolonomic(0, motor_power * -1, 0);
        } else if (gp.dpad_left) {
            //rotate counter-clockwise
            dtHW.moveHolonomic(0, 0, motor_power * 1);
        } else if (gp.dpad_right) {
            //rotate clockwise
            dtHW.moveHolonomic(0, 0, motor_power * -1);
        }
        else {
            dtHW.stopAllMotors();
        }

    }

    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            update();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
