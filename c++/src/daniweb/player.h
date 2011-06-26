#ifndef player_h
#define player_h

//player
class PLAYER
{
     private:
          char name[32];
          int strength;
          int intelligence;
          int gold;
          int experience;
          int health;
          int maxHealth;
          int mana;
          int maxMana;
          int level;
     public:
          PLAYER();
          bool setName (char* newName);
          char* getName();
          void setStrength (int newStr);
          int getStrength();
          void setIntelligence (int newInt);
          int getIntelligence();
          void setGold (int newGold);
          void addGold (int amount);
          void spendGold (int amount);
          int getGold();
          void setExperiance (int newExperiance);
          void addExperiance (int amount);
          int getExperiance();
          void setHealth (int newHealth);
          void addHealth (int value);
          void loseHealth (int value);
          int getHealth();
          void setMaxHealth (int newMaxHealth);
          int getMaxHealth();
          void setMana (int newMana);
          void addMana (int amount);
          void loseMana(int amount);
          int getMana();
          void setMaxMana (int newMaxMana);
          int getMaxMana();
};

#endif
