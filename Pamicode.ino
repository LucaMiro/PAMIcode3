#include <Wire.h>
#include <math.h>

#define BACKWARD 0x1
#define FORWARD 0x0


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Codeurs {
public:
  Codeurs(int address = 0x10);
  void begin();
  bool test();
  void reset();
  void read16(int16_t &gauche, int16_t &droit);
  void read(int32_t &gauche, int32_t &droit);
private:
  int _address;
  int32_t _gauche, _droit;
  int16_t _g16, _d16;
};

static const int16_t MAX = 16384;
Codeurs::Codeurs(int address)
  : _address(address) {
  _gauche = 0;
  _droit = 0;
  _g16 = 0;
  _d16 = 0;
}

void Codeurs::begin() {
  Wire.begin();
}

bool Codeurs::test() {
  byte data = 5, error;
  Wire.beginTransmission(_address);
  Wire.write(data);
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("I2C device found at address 0x");
    Serial.println(_address, HEX);
    Wire.requestFrom(_address, 1);
    data = 0;
    while (Wire.available()) {  // slave may send less than requested
      data = Wire.read();       // receive a byte
    }
    Serial.print("Data = 0x");
    Serial.println(data, HEX);
    return (data == 0x1F);
  }
  return false;
}

void Codeurs::read16(int16_t &gauche, int16_t &droit) {
  byte data[4];
  data[0] = 1;
  Wire.beginTransmission(_address);
  Wire.write(data[0]);
  Wire.endTransmission();
  Wire.requestFrom(_address, 4);
  int i = 0;
  while (Wire.available()) {  // slave may send less than requested
    data[i++] = Wire.read();  // receive a byte
  }
  if (i == 4) {
    gauche = data[0] << 8 | (data[1] & 0xFF);
    droit = data[2] << 8 | (data[3] & 0xFF);
  }
}

void Codeurs::reset() {
  byte data[2] = { 0, 1 };
  Wire.beginTransmission(_address);
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.endTransmission();
  _gauche = 0;
  _droit = 0;
  _g16 = 0;
  _d16 = 0;
}

void Codeurs::read(int32_t &gauche, int32_t &droit) {
  int16_t ng, nd;
  read16(ng, nd);
  if ((ng > MAX) && (_g16 < -MAX)) {
    _gauche = _gauche - _g16 + ng - 65536;
  } else if ((ng < -MAX) && (_g16 > MAX)) {
    _gauche = _gauche - _g16 + ng + 65536;
  } else {
    _gauche = _gauche - _g16 + ng;
  }
  _g16 = ng;
  if ((nd > MAX) && (_d16 < -MAX)) {
    _droit = _droit - _d16 + nd - 65536;
  } else if ((nd < -MAX) && (_d16 > MAX)) {
    _droit = _droit - _d16 + nd + 65536;
  } else {
    _droit = _droit - _d16 + nd;
  }
  _d16 = nd;
  gauche = _gauche;
  droit = _droit;
}


Codeurs codeurs;//faut déclarer les codeurs avant la classe robot car cette dernière en a besoin
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class Moteur {
public:
  Moteur(int pinPwm, int pinRotation);

  //contrôle des moteurs
  void rawForward(int dutyCycle);   // méthode pour avancer en fonction du dutyCycle
  void rawBackward(int dutyCycle);  // méthode pour reculer en fonction du dutyCycle
  void stop(void);                  // arrête le moteur

  //informations/mesures
  float distance(void);  // distance parcourue en mm

private:
  int m_pinPWM, m_pinRotation;  //sert à utiliser les pins passés en paramètres du constructeur dans l'ensemble de la classe
};

Moteur::Moteur(int pinPWM, int pinRotation) {
  m_pinPWM = pinPWM;
  m_pinRotation = pinRotation;

  pinMode(m_pinPWM, OUTPUT);
  pinMode(m_pinRotation, OUTPUT);
}

//fonctionne
void Moteur::rawForward(int dutyCycle) {
  digitalWrite(m_pinRotation, FORWARD);
  analogWrite(m_pinPWM, dutyCycle);
}

//fonctionne
void Moteur::rawBackward(int dutyCycle) {
  digitalWrite(m_pinRotation, BACKWARD);
  analogWrite(m_pinPWM, dutyCycle);
}

//fonctionne
void Moteur::stop(void) {
  analogWrite(m_pinPWM, 0);
  Serial.println("moteur arrêté\n");
}

Moteur moteurGauche(PA13, PA15);
Moteur moteurDroit(PA12, PA14);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class robot {
public:
  int valD, valG;

  float m_perimetre = PI * 10.60 * 47.74648;  //PI*DIAMETRE_ROBOT*(TICKS_PAR_TOUR*PERIMETRE_ROUE)

  float m_distance;     //distance à parcourir en cm
  float m_ticks;        //nombre de ticks à parcourir
  float m_vBase = 127;  // vitesse de base

  float m_erreurPosition,        //erreur de position moyenne des deux roues (asservissement de distance)
    m_erreurRotation,            //erreur de l'angle de rotation moyenne des deux roues (asservissement de rotation)
    m_correctionPosition,        //correction de position (asservissement de distance proportionnel)
    m_correctionRotation,        //correction de rotation (asservissement de rotation proportionnel)

    m_kp = 0.47,                 //coefficient proportionnel asservissement distance et rotation
    m_kd = 0;                    //coefficient dérivé asservissement distance et rotation

  int i;                    //compteur
  int m_commandeMotGauche,  //commande moteur gauche
    m_commandeMotDroit;     //commande moteur droit

  int32_t codeurGauche, codeurDroit;

  robot(Moteur &moteurDroit, Moteur &moteurGauche);  // on met un & pour faire référence à des objets déjà existants

//voir si dutyCycle est vraiment utile (pour limiter la vitesse max du robot)
  void forward(int dutyCycle, float distance);
  void turn(int dutyCycle, float angle);
  void goTo(float x, float y, float theta);


//voir si c'est utile (petit doute)
  int compensationFsecMotDroit(int commande);
  int compensationFsecMotGauche(int commande);
  int saturationNumerique(int commande);

private:
  /*
    déclatation de membres de la classe robot à partir d'une réference de la classe Moteur
    "m_moteurDroit est une référence à moteurDroit, je peux alors contrôleur moteurDroit par le biais de m_moteurDroit, qui appartient à la classe robot"
    */
  Moteur &m_moteurDroit;
  Moteur &m_moteurGauche;
};

robot::robot(Moteur &moteurDroit, Moteur &moteurGauche)
  : m_moteurDroit(moteurDroit), m_moteurGauche(moteurGauche)  //en vrai on peut mettre un constructeur par défaut mais au cas où je laisse ça là
{
}

//calcul de la distance à parcourir à vérifier mais ça "marche"
void robot::forward(int dutyCycle, float distance) {
  m_ticks = distance * 47.74648;
  m_erreurPosition = 0;
  codeurs.reset();

  do {
    codeurs.read(codeurGauche, codeurDroit);

    // calcul des erreurs de position moyenne :
    m_erreurPosition = m_ticks - (-codeurGauche + codeurDroit) / 2;  // erreur de position moyenne des deux roues
    // debug :
    Serial.println(m_erreurPosition);
    // correction proportionnelle
    m_correctionPosition = m_kp * m_erreurPosition;

    // commande des moteurs :
    m_commandeMotDroit = m_correctionPosition;
    m_commandeMotGauche = m_correctionPosition;

    // compensation des frottements secs :
    m_commandeMotGauche = compensationFsecMotGauche(m_commandeMotGauche);
    m_commandeMotDroit = compensationFsecMotDroit(m_commandeMotDroit);

    // saturation numérique :
    m_commandeMotDroit = saturationNumerique(m_commandeMotDroit);
    m_commandeMotGauche = saturationNumerique(m_commandeMotGauche);

    // rampe d'accélération
    if (i < 120) {
      m_vBase = i;
      i++;
      delay(2.5);
    }

    // contrôle des moteurs
    if ((m_vBase + m_commandeMotDroit) < dutyCycle) {
      m_moteurDroit.rawForward(m_vBase + m_commandeMotDroit);
      m_moteurGauche.rawForward(m_vBase + m_commandeMotGauche);
    } else {
      m_moteurDroit.rawForward(dutyCycle);
      m_moteurGauche.rawForward(dutyCycle);
    }




  } while (m_erreurPosition > 0);

  m_moteurDroit.stop();
  m_moteurGauche.stop();
}

//la fonction marche pas, je sais pas bien d'où le problème vient
void robot::turn(int dutyCycle, float angle) {
  //réinitialisation des variables
  i = 0;
  m_erreurPosition =0;
  codeurs.reset();

  // if (angle > 180)  //le robot devra tourner par la gauche
  // {
  //   angle = 360 - angle;  //angle à parcourir
  //   m_ticks = (int)((angle / 360.0) * m_perimetre);

  //   do {
  //     // compte des ticks de chaque moteur
  //     codeurs.read(codeurGauche, codeurDroit);

  //     // calcul de l'erreur de position moyenne des roues
  //     m_erreurPosition = m_ticks - (-codeurGauche + codeurDroit) / 2;

  //     // correction proportionnelle
  //     m_correctionPosition = 0.1 * m_erreurPosition;

  //     m_commandeMotDroit = m_correctionPosition;
  //     m_commandeMotGauche = m_correctionPosition;

  //     // compensation des frottements secs
  //     m_commandeMotGauche = compensationFsecMotGauche(m_commandeMotGauche);
  //     m_commandeMotDroit = compensationFsecMotDroit(m_commandeMotDroit);

  //     // saturation numérique
  //     m_commandeMotDroit = saturationNumerique(m_commandeMotDroit);
  //     m_commandeMotGauche = saturationNumerique(m_commandeMotGauche);

  //     //rampe d'accélération
  //     if (i < 120) {
  //       m_vBase = i;
  //       i++;
  //       delay(2.5);
  //     }

  //     //contrôle des moteurs
  //       m_moteurDroit.rawForward(m_vBase + m_commandeMotDroit);
  //       m_moteurGauche.rawBackward(m_vBase + m_commandeMotGauche);
      


  //     // debug
  //   Serial.print(m_erreurPosition);
  //   } while (m_erreurPosition > 0);
  // }
  if (angle < 180) {
    //on n'a pas besoin de recalculer l'angle, on tournera vers la droite
    m_ticks = (int)((angle / 360.0) * m_perimetre);

    do {
      codeurs.read(codeurGauche,codeurDroit);
      // calcul de l'erreur de position moyenne des roues
      m_erreurPosition = m_ticks - (codeurGauche - codeurDroit);
      Serial.println(m_erreurPosition);
      // correction proportionnelle
      m_correctionPosition = m_kp * m_erreurPosition;

      m_commandeMotDroit = m_correctionPosition;
      m_commandeMotGauche = m_correctionPosition;

      // compensation des frottements secs
      m_commandeMotGauche = compensationFsecMotGauche(m_commandeMotGauche);
      m_commandeMotDroit = compensationFsecMotDroit(m_commandeMotDroit);

      // saturation numérique
      m_commandeMotDroit = saturationNumerique(m_commandeMotDroit);
      m_commandeMotGauche = saturationNumerique(m_commandeMotGauche);
      if (i < 120) {
        m_vBase = i;
        i++;
        delay(2.5);
      }

      //contrôle des moteurs
      m_moteurDroit.rawBackward(m_vBase + m_commandeMotDroit);
      m_moteurGauche.rawForward(m_vBase + m_commandeMotGauche);

      // debug

    } while (m_erreurPosition > 0);
  }

  m_moteurDroit.stop();
  m_moteurGauche.stop();
}

//a revérifier et tester
void robot::goTo(float x, float y, float theta) {  //se référer à la fonction aller_a de Quentin disponible dans son rapport
  float m_newX = x;
  float m_newY = y;
  float m_newTheta = theta;

  m_distance = sqrt(pow(m_newX - x, 2) + pow(m_newY - y, 2));
  m_distance = m_distance * 47.74648;

  //calcul de l'angle en radians
  float angle = atan2(m_newY - y, m_newX - x) - theta;
  if (angle > 2 * PI) {
    angle -= 2 * PI;
  } else if (angle < 0) {
    angle += 2 * PI;
  }
  //conversion en degrés
  //  1 rad = 57,2958 deg
  //  x rad = x*57,2958 deg
  angle = angle * 57,2958;
  robot::turn(150,angle);
  robot::forward(150,m_distance);
  robot::turn(150,theta);
}

int robot::compensationFsecMotDroit(int commande) {
  if (commande > 0) {
    commande = commande + 50;
  }

  if (commande < 0) {
    commande = commande - 50;
  }

  return commande;
}

int robot::compensationFsecMotGauche(int commande) {
  if (commande > 0) {
    commande = commande + 50;
  }

  if (commande < 0) {
    commande = commande - 50;
  }

  return commande;
}

int robot::saturationNumerique(int commande) {
  if (commande >= 120) {
    commande = 120;
  }

  if (commande <= -120) {
    commande = -120;
  }
  return commande;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




robot pami(moteurGauche, moteurDroit);
int32_t valGauche, valDroite;

void setup() {
  codeurs.begin();
  Serial.begin(115200);
}

void loop() {
codeurs.read(valGauche,valDroite);
Serial.println(valGauche);

}










/*



void setup() {
  codeurs.begin();

  Serial.begin(115200);
  Serial.println("\nI2C Encoders");

  pinMode(PA15, OUTPUT);
  digitalWrite(PA15, OUTPUT);
  pinMode(PA12, OUTPUT);
  pinMode(PA13, OUTPUT);
}

void loop() {
  Serial.println("test");
  int32_t g, d;
  Serial.println("CRAC encoders demo");

  Serial.println("Test");
  while (!codeurs.test()) {
    Serial.println("Codeurs non connectes");
    delay(1000);
  }
  Serial.println("Codeurs ok");
  codeurs.reset();
  while (1) {
    codeurs.read(g, d);
    Serial.print(g);
    Serial.print(" ");
    Serial.println(d);

    analogWrite(PA13, 250);
    analogWrite(PA12, 250);
    delay(100);
  }
}

*/