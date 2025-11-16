typedef struct Command{
  bool type;   // 0 means straight and 1 means turn
  float value; // distance in cm for straight and angle in degrees for turn  
};

// queue up to 10 commands at a time
#define TOTAL_COMMANDS 10
int queue_size = 0;
Command command_queue[TOTAL_COMMANDS];

void add_command(bool type, float value){
  if (queue_size >= TOTAL_COMMANDS)
    return;

  Command command;
  command.type = type;
  command.value = value;

  command_queue[queue_size] = command;

  queue_size++;
}

void remove_command(){
  for (int i = 1; i < queue_size; i++)
    command_queue[i-1] = command_queue[i]; 
  
  queue_size--;
}
