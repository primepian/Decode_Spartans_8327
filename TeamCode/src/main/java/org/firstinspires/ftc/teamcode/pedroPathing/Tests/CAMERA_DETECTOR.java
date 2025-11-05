package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

//Blob locator Processor part:
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;

import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

@TeleOp (name = "Center objects", group = "Camera")
public class CAMERA_DETECTOR extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        //PredominantColorProcessor.Swatch purple;

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                //.setTargetColorRange(ColorRange.ARTIFACT_PURPLE)         // use a predefined color match
                .setTargetColorRange(ColorRange.GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        /*PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setRoi(ImageRegion.entireFrame())
                .setSwatches(
                        //PredominantColorProcessor.Swatch.PURPLE,
                        //purple = PredominantColorProcessor.Swatch.PURPLE,
                        PredominantColorProcessor.Swatch.GREEN)
                .build();*/


        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                //.addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            //Blob Locator Prossesor Part:
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    8000, 20000, blobs);  // filter out very small blobs.

            telemetry.addLine("Area Density Aspect Arc Circle Center");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f  %5.2f %3d %5.3f (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) b.getArcLength(), b.getCircularity(), (int) boxFit.center.x, (int) boxFit.center.y));

                if(boxFit.center.x > 180) {
                    telemetry.addLine("MOVER ROBOT A LA DERECHA");
                }
                else if(boxFit.center.x < 140) {
                    telemetry.addLine("MOVER ROBOT A LA IZQUIERDA");
                }

                if(b.getContourArea() > 17000) {
                    telemetry.addLine("SUCCIONAR PELOTA");
                }
                else if(b.getContourArea() < 16000) {
                    telemetry.addLine("ROBOT FORWARD");
                }
            }


            telemetry.update();
            sleep(50);
        }
    }
}