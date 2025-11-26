#ifndef COMMANDS_HPP
#define COMMANDS_HPP

typedef struct Command{
  bool type;   // 0 means straight and 1 means turn
  float value; // distance in cm for straight and angle in degrees for turn  
};

// queue up to 10 commands at a time
#define TOTAL_COMMANDS 20
extern int command_queue_size;
extern Command command_queue[TOTAL_COMMANDS];

void add_command(bool, float);
void remove_command();
void add_360turn();
void add_180turn();

#endif
