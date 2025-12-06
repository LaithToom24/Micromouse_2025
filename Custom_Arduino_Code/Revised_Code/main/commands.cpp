#include "commands.hpp"
#include <Arduino.h>

int command_queue_size = 0;
Command command_queue[TOTAL_COMMANDS];

void add_command(bool type, float value){
  if (command_queue_size >= TOTAL_COMMANDS)
    return;

  Command command;
  command.type = type;
  command.value = value;

  command_queue[command_queue_size] = command;

  command_queue_size++;
}

void remove_command(){
  for (int i = 1; i < command_queue_size; i++)
    command_queue[i-1] = command_queue[i]; 
  
  command_queue_size--;
}

void print_commands(){
  for (int i = 0; i < command_queue_size+1; i++){
    Serial.print(i);
    Serial.print(" ");
    Serial.print(command_queue[i].type);
    Serial.print(" ");
    Serial.println(command_queue[i].value);
  }
}

void add_360turn(){
  add_command(1, 90);
  add_command(1, 90);
  add_command(1, 90);
  add_command(1, 90);
}

void add_180turn(){
  add_command(1, 90);
  add_command(1, 90);
}