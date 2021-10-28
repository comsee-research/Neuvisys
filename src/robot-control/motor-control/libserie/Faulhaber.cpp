#include "Faulhaber.hpp"
#include <iostream>
#include <string>     // std::string, std::to_string

Faulhaber::Faulhaber(int Adresse_moteur, string port) : ComSerie() {
    std::cout << " Ouverture de " << port << std::endl;
    ouvrir(port, B9600 | CS8 | CLOCAL | CREAD, 10);
    Adresse_moteur_ = std::to_string(Adresse_moteur);
}

Faulhaber::~Faulhaber() {
    fermer(); //Fermeture du lien serie
}

void Faulhaber::StartDrive(void) {
    std::string chaine;
    chaine = Adresse_moteur_ + "GSER\r\n"; //Get Serial 0
    chaine += Adresse_moteur_ + "EN\r\n";  // Enable drive 0
    chaine += Adresse_moteur_ + "CST\r\n";  //Set operating mode
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();

    std::cout << "Drive " << Adresse_moteur_ << " actif." << std::endl;
//  usleep(5000);
}

void Faulhaber::StopDrive(void) {
    string chaine;
    chaine = Adresse_moteur_ + "DI\r\n"; //Disable drive
    chaine += Adresse_moteur_ + "CST\r\n";
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();

    std::cout << "Drive " << Adresse_moteur_ << " inactif." << std::endl;
}


void Faulhaber::GetPosition(string &position) {
    uint8_t octet = 0;
    while (recevoir(&octet, 1));  //vide le buffer de reception
    string chaine;
    chaine = Adresse_moteur_ + "POS\r\n"; //Get actual position*/
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();

    std::cout << "Position moteur " << Adresse_moteur_ << " : ";
    // usleep(10000);
    int nbTrames = 0;
//    while (nbTrames <= 10)
    {
        while (recevoir(&octet, 1)) {
            if (octet == '\n') ++nbTrames;
            std::cout << octet;
//            usleep(1000);
        }
//        else cout << "timeout" << endl;
    }

}


void Faulhaber::SetAbsolutePosition(int position) {
    string chaine;
    chaine = Adresse_moteur_ + "LA" + std::to_string(position) + "\r\n"; //Set absolute position*/
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();
    chaine = "";
    chaine = Adresse_moteur_ + "NP\r\n";  // Stop on the command M until the position is reached
    chaine += Adresse_moteur_ + "M\r\n";   //Move
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();

//  usleep(1000);
}


void Faulhaber::SetRelativePosition(int position) {
    string chaine;
    chaine = Adresse_moteur_ + "LR" + std::to_string(position) + "\r\n"; //Set absolute position*/
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();
    chaine = "";
    chaine = Adresse_moteur_ + "NP\r\n";  // Stop on the command M until the position is reached
    chaine += Adresse_moteur_ + "M\r\n";   //Move
    envoyer((const uint8_t *) (chaine.c_str()), uint32_t(chaine.length()));
    flush();

//    usleep(200000);
}
