import java.util.List;
import com.cyberbotics.webots.controller.*;

public class AutonomyThree extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;  // Tableau des capteurs de distance
    private Motor leftMotor, rightMotor;  // Moteurs gauche et droit
    private PositionSensor leftMotorSensor, rightMotorSensor;  // Capteurs de position des moteurs
    private Camera camera;  // Caméra pour détecter la cible
    private Emitter emitter;  // Émetteur pour envoyer des messages
    private Receiver receiver;  // Récepteur pour recevoir des messages
    private String robotName;  // Nom du robot
    private boolean isEmitter = false;  // Si ce robot est l'émetteur (le premier à détecter la cible)
    private boolean hasReachedTarget = false;  // Si ce robot a atteint la cible
    private static boolean emitterAssigned = false;  // Indique si un émetteur a déjà été assigné
    private double[] targetPosition = null;  // Position de la cible
    private Odometry odometry;  // Classe d'odométrie pour la localisation

    // Constructeur pour initialiser tous les composants du robot
    public AutonomyThree() {
        timeStep = 128;

        // Initialisation des capteurs de distance
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
            distanceSensor[i].enable(timeStep);  // Activation de chaque capteur
        }

        // Initialisation de la caméra
        camera = this.getCamera("camera");
        camera.enable(timeStep);  // Activation de la caméra
        camera.recognitionEnable(timeStep);  // Activation de la reconnaissance des objets

        // Initialisation des communications
        emitter = getEmitter("emitter");
        receiver = getReceiver("receiver");
        receiver.enable(timeStep);  // Activation du récepteur

        // Initialisation des moteurs
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);  // Contrôle des moteurs en position infinie
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);  // Initialisation des vitesses des moteurs
        rightMotor.setVelocity(0.0);

        // Initialisation des capteurs de position des moteurs
        leftMotorSensor = this.getPositionSensor("left wheel sensor");
        rightMotorSensor = this.getPositionSensor("right wheel sensor");
        leftMotorSensor.enable(timeStep);  // Activation des capteurs de position
        rightMotorSensor.enable(timeStep);

        // Initialisation de l'odométrie pour la localisation
        odometry = new Odometry();
        initLocalisation();

        // Initialisation du nom du robot
        robotName = getName();
    }

    // Initialisation de la localisation
    private void initLocalisation() {
        odometry.track_start_pos(leftMotorSensor.getValue(), rightMotorSensor.getValue());
    }

    // Méthode pour déplacer le robot vers la position de la cible
    private void moveTowards(Double xObj, Double yObj) {
        odometry.track_step_pos(leftMotorSensor.getValue(), rightMotorSensor.getValue());
        Double[] currentPosition = getPosition();
        double distance = Math.sqrt(Math.pow(xObj - currentPosition[0], 2) + Math.pow(yObj - currentPosition[1], 2));

        // Si la distance est supérieure à un seuil, avancer vers la cible
        if (distance > 0.2) {
            double theta_goal = Math.atan2(yObj - currentPosition[1], xObj - currentPosition[0]);
            double deltaTheta = theta_goal - odometry.getTheta();
            if (Math.abs(deltaTheta) > 0.1) {
                move(50, -50);  // Ajustement de direction (rotation)
            } else {
                move(100, 100);  // Avancer directement vers la cible
            }
        } else {
            move(0, 0);  // Arrêt du robot lorsqu'il atteint la cible
            hasReachedTarget = true;  // Marque que le robot a atteint la cible
            System.out.println(robotName + " a atteint la cible !");
        }
    }

    // Méthode pour obtenir la position actuelle du robot
    private Double[] getPosition() {
        return new Double[]{odometry.getX(), odometry.getY()};
    }

    // Méthode pour définir la vitesse des moteurs
    protected void move(double left, double right) {
        double max = 6.28;  // Vitesse maximale
        leftMotor.setVelocity(left * max / 100);  // Calcul de la vitesse du moteur gauche
        rightMotor.setVelocity(right * max / 100);  // Calcul de la vitesse du moteur droit
    }

    // Méthode pour envoyer un message avec l'émetteur
    protected void broadcastMessage(String message) {
        emitter.send(message.getBytes());  // Envoi du message sous forme de bytes
        System.out.println(robotName + " a envoyé : " + message);
    }

    // Méthode pour vérifier si un message a été reçu par le récepteur
    protected String checkMailBox() {
        while (receiver.getQueueLength() > 0) {
            byte[] message = receiver.getData();
            receiver.nextPacket();  // Passer au message suivant
            if (message != null) {
                String receivedMessage = new String(message);
                System.out.println(robotName + " a reçu : " + receivedMessage);
                return receivedMessage;
            }
        }
        return null;
    }

    // Méthode pour lire les valeurs des capteurs de distance
    protected double[] readDistanceSensorValues() {
        double[] psValues = new double[8];
        for (int i = 0; i < 8; i++) {
            psValues[i] = distanceSensor[i].getValue();  // Lecture de chaque capteur de distance
        }
        return psValues;
    }

    // Méthode pour éviter les obstacles
    private void avoidObstacles() {
        double[] psValues = readDistanceSensorValues();
        boolean obstacleFront = psValues[0] > 80.0 || psValues[7] > 80.0;  // Détection d'un obstacle devant
        boolean obstacleLeft = psValues[5] > 80.0 || psValues[6] > 80.0;   // Détection d'un obstacle à gauche
        boolean obstacleRight = psValues[1] > 80.0 || psValues[2] > 80.0;  // Détection d'un obstacle à droite

        // Si un obstacle est détecté, tourner ou reculer en fonction de sa position
        if (obstacleFront) {
            if (obstacleLeft && !obstacleRight) {
                move(100.0, -100.0);  // Tourner à droite
            } else if (obstacleRight && !obstacleLeft) {
                move(-100.0, 100.0);  // Tourner à gauche
            } else {
                move(-100.0, -100.0);  // Reculer
            }
        } else {
            move(100.0, 100.0);  // Avancer si aucun obstacle
        }
    }

    // Méthode principale d'exécution
    public void run() {
        while (step(timeStep) != -1) {
            if (hasReachedTarget) continue;  // Ignorer les robots qui ont atteint la cible

            if (!emitterAssigned) {
                // Si l'émetteur n'est pas encore assigné, chercher la cible
                avoidObstacles();
                List<CameraRecognitionObject> detectedObjects = List.of(camera.getRecognitionObjects());
                CameraRecognitionObject target = detectedObjects.stream()
                        .filter(obj -> "cible".equals(obj.getModel()))
                        .findFirst()
                        .orElse(null);

                if (target != null) {
                    // Le premier robot à détecter la cible devient l'émetteur
                    emitterAssigned = true;
                    isEmitter = true;
                    targetPosition = new double[]{target.getPosition()[0], target.getPosition()[2]};
                    broadcastMessage("TARGET_FOUND:" + targetPosition[0] + ":" + targetPosition[1]);
                    System.out.println(robotName + " a détecté la cible à la position : X = " + targetPosition[0] + ", Z = " + targetPosition[1]);
                }
            }

            if (isEmitter && targetPosition != null) {
                // L'émetteur se dirige vers la cible
                moveTowards(targetPosition[0], targetPosition[1]);
            }

            if (!isEmitter && targetPosition == null) {
                // Les récepteurs attendent un message
                String message = checkMailBox();
                if (message != null && message.startsWith("TARGET_FOUND")) {
                    String[] parts = message.split(":");
                    targetPosition = new double[]{Double.parseDouble(parts[1]), Double.parseDouble(parts[2])};
                    System.out.println(robotName + " se dirige vers la cible...");
                }
            }

            if (!isEmitter && targetPosition != null && !hasReachedTarget) {
                // Les récepteurs se dirigent vers la cible
                moveTowards(targetPosition[0], targetPosition[1]);
            }
        }
    }

    // Méthode main pour lancer le robot
    public static void main(String[] args) {
        AutonomyThree controller = new AutonomyThree();
        controller.run();
    }

    // Classe d'odométrie pour la localisation du robot
    private class Odometry {
        private double x, y, theta;
        private double leftPrev, rightPrev;

        // Initialisation de la position du robot
        public void track_start_pos(double left, double right) {
            leftPrev = left;
            rightPrev = right;
            x = 0;
            y = 0;
            theta = 0;
        }

        // Mise à jour de la position du robot
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

        // Getter pour X
        public double getX() {
            return x;
        }

        // Getter pour Y
        public double getY() {
            return y;
        }

        // Getter pour l'angle (orientation)
        public double getTheta() {
            return theta;
        }
    }
}
