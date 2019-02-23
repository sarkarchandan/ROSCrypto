#ifndef MESSAGEARCHIVE_H
#define MESSAGEARCHIVE_H

#include <string>
#include <vector>
#include <cstdlib>

const std::vector<std::string> messages_for_bob = {
  "Believe you can and you are halfway there.",
  "You have to expect things of yourself before you can do them.",
  "It always seems impossible until it is done.",
  "Good things come to people who wait but better things come to those who go out and get them.",
  "Successful and unsuccessful people do not vary greatly in their abilities, they vary in their desires to reach their potential.",
  "Success is the sum of small efforts, repeated day in and day out.",
  "There are two kinds of people in this world, those who want to get things done and those who do not want to make mistakes.",
  "Push yourself, because no one else is going to do it for you.",
  "Some people dream of accomplishing great things, others stay awake and make it happen.",
  "You do not always get what you wish for, you get what you work for.",
  "The only place where success comes before work is in the dictionary.",
  "Challenges are what make life interesting, overcoming them is what makes life meaningful.",
  "If you are going through hell, keep going."
};


const std::vector<std::string> messages_for_alice = {
  "If you want to shine like a sun, first burn like a sun.",
  "You have to dream before your dreams can come true.",
  "We should not give up and we should not allow the problem to defeat us.",
  "To succeed in your mission, you must have single minded devotion to your goal.",
  "Small aim is a crime, have great aim.",
  "Man needs his difficulties because they are necessary to enjoy success.",
  "Great dreams of great dreamers are always transcended.",
  "There has to be a global mission of human progress.",
  "All of us do not have equal talent but all of us have an equal opportunity to develop our talent.",
  "Dream is not that you see in sleep, dream is something that does not let you sleep.",
  "Do not take rest after your first victory because if you fail in second, more lips are waiting to say that your first victory was just luck."
 };


std::string RandomMessage(const char& _ch)
{
  std::time_t _time;
  std::srand((unsigned) std::time(&_time));
  return (_ch == 'A') ? messages_for_alice[std::rand() % messages_for_alice.size()] : messages_for_bob[std::rand() % messages_for_bob.size()];
}



#endif //MESSAGEARCHIVE_H
