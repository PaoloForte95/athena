# Action msg -> Contains basic information about the action
string name
int32 action_id
int32 robotid
int32[] parents
string[] waypoints
int32 level # used to differentiate two different actions with the same name and parameters that appear in a different moment in the plan
