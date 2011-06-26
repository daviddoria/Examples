#include "library.h"

#include <cstring>

PLAYER::PLAYER()
{
     strength = 1;
     intelligence = 1;
     gold = 0;
     experience = 0;
     health = 25;
     maxHealth = 25;
     mana = 0;
     maxMana = 0;
}
void PLAYER::addExperiance(int amount)
{
     setExperiance(experience + amount);
}

void PLAYER::addGold(int amount)
{
     setGold(gold + amount);
}

void PLAYER::addHealth(int value)
{
     setHealth(health + value);
}

void PLAYER::addMana(int amount)
{
     setMana(mana + amount);
}

int PLAYER::getExperiance()
{
     return experience;
}

int PLAYER::getGold()
{
     return gold;
}

int PLAYER::getHealth()
{
     return health;
}

int PLAYER::getMana()
{
     return mana;
}

int PLAYER::getIntelligence()
{
     return  intelligence;
}

int PLAYER::getMaxHealth()
{
     return maxHealth;
}

int PLAYER::getMaxMana()
{
     return maxMana;
}

char* PLAYER::getName()
{
     return name;
}

int PLAYER::getStrength()
{
     return strength;
}

void PLAYER::setExperiance(int newExperiance)
{
     experience = newExperiance;
     
     //check for level ups
}

void PLAYER::setGold(int newGold)
{
     gold = newGold;
}

void PLAYER::setHealth(int newHealth)
{
     health = newHealth;
     
     if (health > maxHealth)
          health = maxHealth;
     if (health < 0)
          health = 0;
}

void PLAYER::setIntelligence(int newInt)
{
     intelligence = newInt;
}

void PLAYER::setMana(int newMana)
{
     mana = newMana;
     
     if (mana > maxMana)
          mana = maxMana;
     if (mana < 0)
          mana = 0;
}

void PLAYER::setMaxHealth(int newMaxHealth)
{
     maxHealth = newMaxHealth;
}

void PLAYER::setMaxMana(int newMaxMana)
{
     maxMana = newMaxMana;
}

bool PLAYER::setName(char* newName)
{
     if (strlen(newName) == 0)
          return false;
     
     if (strlen(newName) > 32)
          return false;

     strcpy(name, newName);
     
     return true;
}

void PLAYER::setStrength(int newStr)
{
     strength = newStr;
}

void PLAYER::loseHealth(int amount)
{
     setHealth(health - amount);
}

void PLAYER::loseMana(int amount)
{
     setMana(mana - amount);
}

void PLAYER::spendGold(int amount)
{
     setGold(gold - amount);
}
