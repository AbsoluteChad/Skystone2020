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

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotMain;

import java.util.List;

public class TensorFlow {

    //Labels and stuff
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    //Declare engines
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Stone pixel boundaries
    private static final double BOUNDARY1 = 200;
    private static final double BOUNDARY2 = 700;

    public TensorFlow() {
        //Initialize engines
        initVuforia();
        initTfod();

        //Activate tfod
        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
     * The starting skystone configuration at the beginning of autonomous can be in one of three states:
     *   1. SS -- -- SS -- -- (left)
     *   2. -- SS -- -- SS -- (middle)
     *   3. -- -- SS -- -- SS (right)
     * SS represents a skystone, while -- represents a regular stone. Keep in mind that left is on the
     * outside on blue alliance and that right is on the blue side on
     *
     * @param enableTimer whether or not to use a timer
     * @param timeout max amount of time (ms) to sense for skystone if <i>enableTimer</i> is true before
     *                giving up and returning an empty string
     * @return Starting skystone config. Will return an empty string if cannot be sensed.
     */
    public String getSkystonePosition(boolean enableTimer, int timeout) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        String skystonePosition = "nope";
        while (skystonePosition.equals("nope")) {
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
                    for (int i = 0; i < updatedRecognitions.size(); i++) {
                        Recognition recognition = updatedRecognitions.get(i);
                        double left = recognition.getLeft();
                        double width = recognition.getWidth();
                        double objCenter = (width / 2) + left;
                        if (objCenter < BOUNDARY1) {
                            skystonePosition = "left";
                        } else if (objCenter >= BOUNDARY1 && objCenter < BOUNDARY2) {
                            skystonePosition = "middle";
                        } else if (objCenter >= BOUNDARY2) {
                            skystonePosition = "right";
                        }
                    }
                }
            }
            if (enableTimer) {
                if (timer.milliseconds() > timeout) {
                    return "nope";
                }
            }
        }

        //Disable tensor flow and return
        if (tfod != null) {
            tfod.shutdown();
        }
        return skystonePosition;
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
        int tfodMonitorViewId = RobotMain.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId",
                "id", RobotMain.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }
}
