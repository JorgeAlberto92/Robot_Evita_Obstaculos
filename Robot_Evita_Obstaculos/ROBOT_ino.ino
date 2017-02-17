#include <AFMotor.h>
#include <Servo.h> 
#include <NewPing.h>

#define TRIG_PIN A4 //Pina A4 pone el trig del sensor ultrasonico sobre la shield que controla los motores
#define ECHO_PIN A5 //Pina A5 pone el echo del sensor ultrasonico sobre la shield que controla los motores
#define MAX_DISTANCE 200 // coloca el valor máximo de reconocimiento del sensor aproximadamente 200 cm.
#define MAX_SPEED 180 //coloca la velocidad de los motores a un intervalo de 180 rpm aunque lo máximo son 256rpm
#define MAX_SPEED_OFFSET 10 //esto permite que el intervalo de tiempo entre que pare un motor y otro sea de 10milisegundos
#define COLL_DIST 10 // coloca la distancia minima a la cual el robot debe de girar
#define TURN_DIST COLL_DIST+10 // coloca la distancia minima a la cual el robot debe de parar y tomar una decisión
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); //coloca los sensores en el metodo sonar de la libreria sonar la cual recibira de parametro los estatus anteriores

AF_DCMotor motor1(1, MOTOR12_1KHZ); // crea el motor #1, utilizando la salida M1 de la shield controladora, y le da un valor de 1khz a la frecuencia del pwm
AF_DCMotor motor2(4, MOTOR12_1KHZ); // crea el motor #2, utilizando la salida M4 de la shield controladora, y le da un valor de 1khz a la frecuencia del pwm
Servo myservo;  // crea un objeto servo de la clase instanciada

int pos = 0; // variable posición a continuación se colocan una serie de variables que seran utiliadas en el resto del programa
  int maxDist = 0;
  int maxAngle = 0;
  int maxRight = 0;
  int maxLeft = 0;
  int maxFront = 0;
int course = 0;
int curDist = 0;
String motorSet = "";
int speedSet = 0;

//-------------------------------------------- SETUP LOOP ----------------------------------------------------------------------------
void setup() {
  myservo.attach(9);  // colocal el servo sobre el pin 9 del arduino (Servo_2 del motorshield)
  myservo.write(90); // le dice al servo la posición de 90 grados 
  delay(2000); // espera alrededor 200 milisegudnos 
  checkPath(); // rutina o método necesario para que pueda empezar a recorrer el camino
  motorSet = "FORWARD"; // coloca el motor con la variable forward la cual nos sirve para que avanze
  myservo.write(90); // se asegura de que el servo aun siga panenando
  moveForward(); // hace que el robot siga moviendose adelante
}
//------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------MAIN LOOP ------------------------------------------------------------------------------
//el main loop es el ciclo infinito el cual se repetira n mil veces dentro de la ejecución del programa
void loop() {
  checkForward(); // revisa si el robot esta avanzando constantemente hacia adelante, esto ocaciona que el consumo energetico en los motores sea mayor
  checkPath(); //pone al sensor ultrasonico a escanear de posibles obstaculos
}
//-------------------------------------------------------------------------------------------------------------------------------------
//------------método checkpath para el servomotor.
void checkPath() {
  int curLeft = 0;
  int curFront = 0;
  int curRight = 0;
  int curDist = 0;
  myservo.write(144); //coloca el servo a 54 grados
  delay(120); //espera 120 milisegundos para que el servo busque su posición
  for(pos = 144; pos >= 36; pos-=18)     // ciclo importante del programa el cual realiza el paneo desde 144 grados a 36 
  {
    myservo.write(pos);  // le dice al servo que vaya a la posición almacenado en la variables pos
    delay(90); // espera 90 segundos para obtener la posición
    checkForward(); // checa si el motor si aun se esta moviendo adelante
    curDist = readPing(); // obtene la distancia de algun objeto si es que lo hubiera
    if (curDist < COLL_DIST) { //si la distancia es menor antes de que colisione
      checkCourse(); // funcion que checa el curo y lo recorre
      break; //salta de este ciclo
    }
    if (curDist < TURN_DIST) { // si la distancia es menor gira
      changePath(); // corre la rutina o funcion chancepath
    }
    if (curDist > curDist) {maxAngle = pos;}
    if (pos > 90 && curDist > curLeft) { curLeft = curDist;}
    if (pos == 90 && curDist > curFront) {curFront = curDist;}
    if (pos < 90 && curDist > curRight) {curRight = curDist;}
  }
  maxLeft = curLeft;
  maxRight = curRight;
  maxFront = curFront;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void setCourse() { // coloca la dirección de viaje basada en el mapeo
    if (maxAngle < 90) {turnRight();}
    if (maxAngle > 90) {turnLeft();}
    maxLeft = 0;
    maxRight = 0;
    maxFront = 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void checkCourse() {
  moveBackward();
  delay(500);
  moveStop();
  setCourse();
}
//-------------------------------------------------------------------------------------------------------------------------------------
void changePath() {
  if (pos < 90) {veerLeft();} 
  if (pos > 90) {veerRight();} 
}
//-------------------------------------------------------------------------------------------------------------------------------------

int readPing() { //lee la distancia del sensor ultrasonico.
  delay(70);
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}
//-------------------------------------------------------------------------------------------------------------------------------------
void checkForward() { if (motorSet=="FORWARD") {motor1.run(FORWARD); motor2.run(FORWARD); } }     // se asegura que los motores vayan adelante
//-------------------------------------------------------------------------------------------------------------------------------------
void checkBackward() { if (motorSet=="BACKWARD") {motor1.run(BACKWARD); motor2.run(BACKWARD); } } // se asegura que los  motores vayan hacia atras
//-------------------------------------------------------------------------------------------------------------------------------------

// In some cases, the Motor Drive Shield may just stop if the supply voltage is too low (due to using only four NiMH AA cells).
// The above functions simply remind the Shield that if it's supposed to go forward, then make sure it is going forward and vice versa.

//-------------------------------------------------------------------------------------------------------------------------------------
void moveStop() {motor1.run(RELEASE); motor2.run(RELEASE);}  // para los motores
//-------------------------------------------------------------------------------------------------------------------------------------
void moveForward() {
    motorSet = "FORWARD";
    motor1.run(FORWARD);      // hace que se mueva adelante el motor 1
    motor2.run(FORWARD);      // hace que se mueva adelante el motor 2
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2)
  {
    motor1.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor2.setSpeed(speedSet);
    delay(5);
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveBackward() {
    motorSet = "BACKWARD";
    motor1.run(BACKWARD);      // turn it on going forward
    motor2.run(BACKWARD);     // turn it on going forward
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet+MAX_SPEED_OFFSET);
    motor2.setSpeed(speedSet);
    delay(5);
  }
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() {
  motorSet = "RIGHT";
  motor1.run(FORWARD);      // hace que el motor uno vaya hacia adelante
  motor2.run(BACKWARD);     // hace que el motor dos vaya hacia atras
  delay(400); // corre los motores alrededor de 400 milisegundos
  motorSet = "FORWARD";
  motor1.run(FORWARD);      // hace que ambos motores vayan hacia adelante
  motor2.run(FORWARD);      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  motor1.run(BACKWARD);     // 
  motor2.run(FORWARD);      // prende el motor 2 hace que vaya hacia adelante
  delay(400); // corre el motor y le da un delay de 400 milisegundos
  motorSet = "FORWARD";
  motor1.run(FORWARD);      
  motor2.run(FORWARD);      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void veerRight() {motor2.run(BACKWARD); delay(400); motor2.run(FORWARD);} //verifica  el motor derecho vaya hacia adelante alrededor de 400 ms
//-------------------------------------------------------------------------------------------------------------------------------------
void veerLeft() {motor1.run(BACKWARD); delay(400); motor1.run(FORWARD);} // veering el motor izquier vaya hacia atras alrededor de 400 ms
//-------------------------------------------------------------------------------------------------------------------------------------
