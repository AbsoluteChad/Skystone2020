/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.lib.vision.tensorflow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotMain;

import java.util.List;

@TeleOp(name = "TensorFlowTest", group = "Concept")
public class TensorFlowTest extends LinearOpMode {

    //Labels and stuff
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    //Declare engines
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    //Stone pixel boundaries
    private static final double BOUNDARY1 = 350;
    private static final double BOUNDARY2 = 700;
    private static final double LEFT_FOR_LEFT_STONE = 150;
    private static final double MINIMUM_CONFIDENCE = 0.60;
    @Override
    public void runOpMode() {
        //Initialize engines
        initVuforia();
        initTfod();

        //Activate tfod
        if (tfod != null) {
            tfod.activate();
 //           tfod.setClippingMargins(100,200,200,100);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        String skystonePosition = "nope";

        while (opModeIsActive()) {
            if (tfod != null) {

                //Get all recognitions & filter out unwanted
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (int i = updatedRecognitions.size() - 1; i >= 0; i--) {
                        if (!updatedRecognitions.get(i).getLabel().equals(LABEL_SKYSTONE)) {
                            updatedRecognitions.remove(i);
                        }
                    }

                    //Skystone processing
                    telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                    for (int i = 0; i < updatedRecognitions.size(); i++) {
                        Recognition recognition = updatedRecognitions.get(i);
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData("left", recognition.getLeft());
                        telemetry.addData("top", recognition.getTop());
                        telemetry.addData("right", recognition.getRight());
                        telemetry.addData("width", recognition.getWidth());
                        telemetry.addData("center of obj", (recognition.getWidth() / 3) + recognition.getLeft());
                        double left = recognition.getLeft();
                        double width = recognition.getWidth();
                        double objCenter = (width / 3) + left;

                        if(recognition.getTop()  < 50) {
                            continue;
                        }
                        if ((objCenter < BOUNDARY1) || (recognition.getLeft() < LEFT_FOR_LEFT_STONE )) {
                            skystonePosition = "left " + recognition.getConfidence();
                        } else if (objCenter >= BOUNDARY1 && objCenter < BOUNDARY2) {
                            skystonePosition = "middle " + recognition.getConfidence();
                        } else if (objCenter >= BOUNDARY2) {
                            skystonePosition= "right " + recognition.getConfidence();
                        }
                        telemetry.addData("skystonePosition", skystonePosition);
                        telemetry.addData("recog conf", recognition.getConfidence());

                        if(recognition.getLabel().equals("Skystone")){
                            double boxX = recognition.getWidth() / 2;
                            double boxMid = recognition.getLeft() + boxX ;
                            if(boxMid >= 512 && boxMid < 768) {
                                skystonePosition= "new center : " + recognition.getConfidence();
                            } else if (boxMid > (recognition.getImageWidth()/2)){
                                    skystonePosition= "new right : " + recognition.getConfidence();
                            } else {
                                skystonePosition= "new left : " + recognition.getConfidence();
                            }
                            telemetry.addData("boxX" , boxX);
                            telemetry.addData("boxMid :" , boxMid);
                            telemetry.addData("new position :" , skystonePosition);
                        }
                    }

                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

        //  @Override
    public void runOpModeold() {
        //Initialize engines
        initVuforia();
        initTfod();

        //Activate tfod
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (tfod != null) {
                //Get all recognitions & filter out unwanted
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (int i = updatedRecognitions.size() - 1; i >= 0; i--) {
                        if (!updatedRecognitions.get(i).getLabel().equals(LABEL_SKYSTONE)) {
                            updatedRecognitions.remove(i);
                        }
                    }

                    //Skystone processing
                    telemetry.addData("# of Objects Detected", updatedRecognitions.size());
                    for (int i = 0; i < updatedRecognitions.size(); i++) {
                        Recognition recognition = updatedRecognitions.get(i);
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData("left", recognition.getLeft());
                        telemetry.addData("width", recognition.getWidth());
                        telemetry.addData("center of obj", (recognition.getWidth() / 4) + recognition.getLeft());
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //Initialize the Vuforia localization engine
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RobotMain.VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.65;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }
}
