#ifndef MODEMANAGER_H
#define MODEMANAGER_H

// Définition de l'enum pour les modes
enum Mode {
  MANUAL,
  AUTOMATIC,
  IDLE,
  CONTEST  // Ajout de CONTEST pour l'exemple
};

class ModeManager {
public:
  ModeManager(); // Constructeur
  void setMode(Mode newMode);
  Mode getMode() const;
  bool isMode(Mode mode) const;

private:
  Mode currentMode;
};

#endif // MODEMANAGER_H