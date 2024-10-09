import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;
    private Motor leftMotor, rightMotor;
    private Camera camera;

    public AutonomyTwo() {
        timeStep = 128; // Control time step

        // Initialize distance sensors
        distanceSensor = new DistanceSensor[8];
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = getDistanceSensor("ps" + i);
            distanceSensor[i].enable(timeStep);
        }

        // Initialize motors
        leftMotor = getMotor("left wheel motor");
        rightMotor = getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);

        // Initialize the camera
        camera = getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep); // Enable object recognition in the camera
    }

    public void run() {
        while (step(timeStep) != -1) {
            // Check sensor values
            double[] psValues = new double[8];
            for (int i = 0; i < 8; i++) {
                psValues[i] = distanceSensor[i].getValue(); // Get values from the enabled distance sensors
            }

            // Sensor values for better understanding:
            double frontLeft = psValues[7];   // Front-left
            double frontRight = psValues[0];  // Front-right
            double left = psValues[5];        // Left
            double right = psValues[2];       // Right
            double topLeft = psValues[6];     // Top-left
            double topRight = psValues[1];    // Top-right

            // Obstacle avoidance logic
            if (frontLeft > 80.0 || frontRight > 80.0 || topLeft > 80.0 || topRight > 80.0) {
                // If there's an obstacle in the front, top-left, or top-right, turn in the direction opposite to the highest sensor value
                if (Math.max(frontLeft, topLeft) > Math.max(frontRight, topRight)) {
                    // Turn right if the left sensors have higher values
                    turnRight();
                } else {
                    // Turn left if the right sensors have higher values
                    turnLeft();
                }
            } else if (left > 80.0) {
                // If there's an obstacle on the left, turn right
                turnRight();
            } else if (right > 80.0) {
                // If there's an obstacle on the right, turn left
                turnLeft();
            } else {
                // If no obstacles, move forward
                leftMotor.setVelocity(6.0);
                rightMotor.setVelocity(6.0);
            }

            // Check for the green square (target) using the camera
            if (camera.getRecognitionNumberOfObjects() > 0) {
                CameraRecognitionObject[] objects = camera.getRecognitionObjects();
                for (CameraRecognitionObject object : objects) {
                    // Check if the object is the green square (target)
                    if (object.getColors()[0] == 0.0 && object.getColors()[1] == 1.0 && object.getColors()[2] == 0.0) { // RGB for green
                        // Display target position
                        double[] position = object.getPosition(); // [x, y, z] coordinates
                        double distanceToTarget = position[1]; // z-axis indicates forward distance
                        double targetX = position[0]; // x-axis indicates left or right

                        System.out.println("Cible détectée à la position : x = " + position[0] + ", y = " + position[1] + ", z = " + position[2]);

                        // Move towards the target
                        while (distanceToTarget >= 0.1) {
                            // Update target information
                            distanceToTarget = object.getPosition()[1];
                            targetX = object.getPosition()[0];

                            // Adjust direction towards the target
                            if (targetX < -0.1) { // Target is to the left
                                leftMotor.setVelocity(4.0);  // Slow down left motor to turn left
                                rightMotor.setVelocity(6.0); // Full speed on right motor
                            } else if (targetX > 0.1) { // Target is to the right
                                leftMotor.setVelocity(6.0);  // Full speed on left motor
                                rightMotor.setVelocity(4.0); // Slow down right motor to turn right
                            } else {
                                // Move straight forward towards the target
                                leftMotor.setVelocity(6.0);
                                rightMotor.setVelocity(6.0);
                            }

                            step(timeStep); // Continue approaching the target

                            // Update distance to target at each step
                            distanceToTarget = object.getPosition()[1];
                        }

                        // Once close enough, stop the robot
                        leftMotor.setVelocity(0.0);
                        rightMotor.setVelocity(0.0);
                        System.out.println("Le robot a atteint la cible.");
                    }
                }
            }
        }
    }

    private void turnRight() {
        leftMotor.setVelocity(6.0); // Move left wheel forward
        rightMotor.setVelocity(-6.0); // Move right wheel backward
        pauseTurning(0.3);
    }

    private void turnLeft() {
        leftMotor.setVelocity(-6.0); // Move left wheel backward
        rightMotor.setVelocity(6.0); // Move right wheel forward
        pauseTurning(0.3);
    }

    private void pauseTurning(double seconds) {
        int targetTime = (int) (seconds * 1000 / timeStep); // Convert seconds to time steps
        for (int i = 0; i < targetTime; i++) {
            step(timeStep); // Keep turning
        }
        // Stop turning after the pause
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);
        step(200); // Pause for a moment after turning
    }

    public static void main(String[] args) {
        AutonomyTwo robot = new AutonomyTwo();
        robot.run();
    }
}
