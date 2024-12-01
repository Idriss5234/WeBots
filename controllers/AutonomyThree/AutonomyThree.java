import java.util.List;
import com.cyberbotics.webots.controller.*;

public class AutonomyThree extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;
    private Motor leftMotor, rightMotor;
    private PositionSensor leftMotorSensor, rightMotorSensor;
    private Camera camera;
    private Emitter emitter;
    private Receiver receiver;
    private String robotName;
    private boolean isEmitter = false; // Si ce robot est le premier à détecter la cible
    private boolean hasReachedTarget = false; // Si ce robot a atteint la cible
    private static boolean emitterAssigned = false; // Indique si un émetteur a déjà été désigné
    private double[] targetPosition = null; // Position de la cible
    private Odometry odometry;

    public AutonomyThree() {
        timeStep = 128;

        // Initialisation des capteurs
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
            distanceSensor[i].enable(timeStep);
        }

        // Caméra
        camera = this.getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);

        // Communication
        emitter = getEmitter("emitter");
        receiver = getReceiver("receiver");
        receiver.enable(timeStep);

        // Moteurs
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // Capteurs de position des moteurs
        leftMotorSensor = this.getPositionSensor("left wheel sensor");
        rightMotorSensor = this.getPositionSensor("right wheel sensor");
        leftMotorSensor.enable(timeStep);
        rightMotorSensor.enable(timeStep);

        // Odometry pour localisation
        odometry = new Odometry();
        initLocalisation();

        // Nom du robot pour identification
        robotName = getName();
    }

    private void initLocalisation() {
        odometry.track_start_pos(leftMotorSensor.getValue(), rightMotorSensor.getValue());
    }

    private void moveTowards(Double xObj, Double yObj) {
        odometry.track_step_pos(leftMotorSensor.getValue(), rightMotorSensor.getValue());
        Double[] currentPosition = getPosition();
        double distance = Math.sqrt(Math.pow(xObj - currentPosition[0], 2) + Math.pow(yObj - currentPosition[1], 2));

        if (distance > 0.2) { // Distance de seuil proche de la cible
            double theta_goal = Math.atan2(yObj - currentPosition[1], xObj - currentPosition[0]);
            double deltaTheta = theta_goal - odometry.getTheta();
            if (Math.abs(deltaTheta) > 0.1) {
                move(50, -50); // Ajustement de direction
            } else {
                move(100, 100); // Avancer directement vers la cible
            }
        } else {
            move(0, 0); // Arrêt près de la cible
            hasReachedTarget = true;
            System.out.println(robotName + " a atteint la cible !");
        }
    }

    private Double[] getPosition() {
        return new Double[]{odometry.getX(), odometry.getY()};
    }

    protected void move(double left, double right) {
        double max = 6.28; // Vitesse maximale
        leftMotor.setVelocity(left * max / 100);
        rightMotor.setVelocity(right * max / 100);
    }

    protected void broadcastMessage(String message) {
        emitter.send(message.getBytes());
        System.out.println(robotName + " a envoyé : " + message);
    }

    protected String checkMailBox() {
        while (receiver.getQueueLength() > 0) {
            byte[] message = receiver.getData();
            receiver.nextPacket();
            if (message != null) {
                String receivedMessage = new String(message);
                System.out.println(robotName + " a reçu : " + receivedMessage);
                return receivedMessage;
            }
        }
        return null;
    }

    protected double[] readDistanceSensorValues() {
        double[] psValues = new double[8];
        for (int i = 0; i < 8; i++) {
            psValues[i] = distanceSensor[i].getValue();
        }
        return psValues;
    }

    private void avoidObstacles() {
        double[] psValues = readDistanceSensorValues();
        boolean obstacleFront = psValues[0] > 80.0 || psValues[7] > 80.0;
        boolean obstacleLeft = psValues[5] > 80.0 || psValues[6] > 80.0;
        boolean obstacleRight = psValues[1] > 80.0 || psValues[2] > 80.0;

        if (obstacleFront) {
            if (obstacleLeft && !obstacleRight) {
                move(100.0, -100.0); // Tourner à droite
            } else if (obstacleRight && !obstacleLeft) {
                move(-100.0, 100.0); // Tourner à gauche
            } else {
                move(-100.0, -100.0); // Reculer
            }
        } else {
            move(100.0, 100.0); // Avancer
        }
    }

    public void run() {
        while (step(timeStep) != -1) {
            if (hasReachedTarget) continue; // Ignorer les robots qui ont atteint la cible

            if (!emitterAssigned) {
                // Tous les robots commencent à chercher la cible
                avoidObstacles();
                List<CameraRecognitionObject> detectedObjects = List.of(camera.getRecognitionObjects());
                CameraRecognitionObject target = detectedObjects.stream()
                        .filter(obj -> "cible".equals(obj.getModel()))
                        .findFirst()
                        .orElse(null);

                if (target != null) {
                    // Le premier robot qui détecte la cible devient l'émetteur
                    emitterAssigned = true;
                    isEmitter = true;
                    targetPosition = new double[]{target.getPosition()[0], target.getPosition()[2]};
                    broadcastMessage("TARGET_FOUND:" + targetPosition[0] + ":" + targetPosition[1]);
                    System.out.println(robotName + " a détecté la cible à la position : X = " + targetPosition[0] + ", Z = " + targetPosition[1]);
                }
            }

            if (isEmitter && targetPosition != null) {
                // L'émetteur se déplace vers la cible
                moveTowards(targetPosition[0], targetPosition[1]);
            }

            if (!isEmitter && targetPosition == null) {
                // Récepteurs attendent le message
                String message = checkMailBox();
                if (message != null && message.startsWith("TARGET_FOUND")) {
                    String[] parts = message.split(":");
                    targetPosition = new double[]{Double.parseDouble(parts[1]), Double.parseDouble(parts[2])};
                    System.out.println(robotName + " se dirige vers la cible...");
                }
            }

            if (!isEmitter && targetPosition != null && !hasReachedTarget) {
                // Récepteurs se dirigent vers la cible
                moveTowards(targetPosition[0], targetPosition[1]);
            }
        }
    }

    public static void main(String[] args) {
        AutonomyThree controller = new AutonomyThree();
        controller.run();
    }

    private class Odometry {
        private double x, y, theta;
        private double leftPrev, rightPrev;

        public void track_start_pos(double left, double right) {
            leftPrev = left;
            rightPrev = right;
            x = 0;
            y = 0;
            theta = 0;
        }

        public void track_step_pos(double left, double right) {
            double dLeft = left - leftPrev;
            double dRight = right - rightPrev;
            double distance = (dLeft + dRight) / 2;
            theta += (dRight - dLeft) / 0.057;
            x += distance * Math.cos(theta);
            y += distance * Math.sin(theta);
            leftPrev = left;
            rightPrev = right;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getTheta() {
            return theta;
        }
    }
}
