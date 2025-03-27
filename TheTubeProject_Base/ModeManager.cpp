#include "ModeManager.h"

// Constructeur
ModeManager::ModeManager() : currentMode(IDLE) {}

// Définition de la fonction pour définir le mode
void ModeManager::setMode(Mode newMode) {
  currentMode = newMode;
}

// Définition de la fonction pour obtenir le mode actuel
Mode ModeManager::getMode() const {
  return currentMode;
}

// Définition de la fonction pour vérifier si un mode est actif
bool ModeManager::isMode(Mode mode) const {
  return currentMode == mode;
}

