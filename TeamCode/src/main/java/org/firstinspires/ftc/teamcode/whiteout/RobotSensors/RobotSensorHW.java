package org.firstinspires.ftc.teamcode.whiteout.RobotSensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotSensorHW {
    public enum DetectedColors{
        RED,
        BLUE,
        WHITE,
        INVALID
    }
    public enum Side{
        LEFT,
        RIGHT
    }

    private DistanceSensor dsLF = null; //Left Front
    private DistanceSensor dsRF = null; //Right Front
    private DistanceSensor dsLR = null; //Left Rear
    private DistanceSensor dsRR = null; //Right Rear
    private DistanceSensor dsLS = null; //Left Side
    private DistanceSensor dsRS = null; //Right Side

    private NormalizedColorSensor csL = null;
    private NormalizedColorSensor csR = null;

    public RobotSensorParams rsp = new RobotSensorParams();
    Telemetry T;

    private boolean debugMode = false;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry t) {
        T = t;
        dsLF = ahwMap.get(DistanceSensor.class, "DSLF");
        dsRF = ahwMap.get(DistanceSensor.class, "DSRF");
        dsLR = ahwMap.get(DistanceSensor.class, "DSLR");
        dsRR = ahwMap.get(DistanceSensor.class, "DSRR");
        dsLS = ahwMap.get(DistanceSensor.class, "DSLS");
        dsRS = ahwMap.get(DistanceSensor.class, "DSRS");
        csL  = ahwMap.get(NormalizedColorSensor.class, "CSL");
        csR  = ahwMap.get(NormalizedColorSensor.class, "CSR");

    }

    public RobotSensorParams getDistances() {
        double correction = 0.0;
        rsp.x_LF =dsLF.getDistance(DistanceUnit.INCH);
        rsp.x_RF =dsRF.getDistance(DistanceUnit.INCH) + correction;
        rsp.x_LR =dsLR.getDistance(DistanceUnit.INCH);
        rsp.x_RR =dsRR.getDistance(DistanceUnit.INCH);
        rsp.x_LS =dsLS.getDistance(DistanceUnit.INCH);
        rsp.x_RS =dsRS.getDistance(DistanceUnit.INCH);
        rsp.c_LS = detectColor(Side.LEFT);
        rsp.c_RS = detectColor(Side.RIGHT);

        return rsp;
    }


    public DetectedColors detectColor(Side sc) {
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        NormalizedRGBA colors = new NormalizedRGBA();
        if (sc == Side.LEFT) {
            colors = csL.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            if (debugMode) T.addLine("LEFT");
        } else {
            colors = csR.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            if (debugMode) T.addLine("RIGHT");
        }
        if (debugMode) {
            T.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        T.addLine()
                .addData("a", "%.3f", colors.alpha)
                .addData("r", "%.3f", colors.red)
                .addData("g", "%.3f", colors.green)
                .addData("g", "%.3f", colors.blue);

            T.update();
        }

        //RED
        if ((hsvValues[0] > 0 && hsvValues[0] < 30) || (hsvValues[0] > 340 && hsvValues[0] < 360)) {
            return DetectedColors.RED;
        }
        if (hsvValues[0] > 200 && hsvValues[0] < 260) {
            return DetectedColors.BLUE;
        }
        /*
        if ((hsvValues[0] >= 0 && hsvValues[0] < 170) &&
           (hsvValues[1] >= 0 && hsvValues[1] < 110) &&
           (hsvValues[2] > 170 && hsvValues[2] < 255)){
            return DetectedColors.WHITE;
        }

         */
        return DetectedColors.INVALID;

    }


}

