# Action msg -> Contains basic information about the action
string name
int32 robotid
string[] parents
int32 level # used to differentiate two different actions with the same name and parameters that appear in a different moment in the plan
