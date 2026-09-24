# Event -> reports when an action or method ran

int32 id
string name

uint8 ACTION = 0
uint8 METHOD = 1
uint8 kind

builtin_interfaces/Time start_time
builtin_interfaces/Time end_time

uint8 RUNNING = 0
uint8 SUCCESS = 1
uint8 FAILURE = 2
uint8 status