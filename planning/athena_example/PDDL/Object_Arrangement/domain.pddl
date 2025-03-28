(define (domain object_arrangement)

(:requirements :strips :typing :fluents :negative-preconditions)

(:types
    location - object
    locatable - object
    cutlery - locatable
    container - locatable
    plate - container
    basket - container
    food - locatable 
    robot  - locatable
    tool - locatable
    screwdriver - locatable
    glass - locatable
    mug - locatable
)

(:predicates
    (free ?rb - robot)
    (at ?obj - object ?loc - location) 
    (on ?obj1 - food ?obj2 - container) 
    (loaded ?obj1 - locatable ?rb - robot)

)

(:action move
    :parameters (?robot - robot ?loc1 - location ?loc2 - location)
    :precondition(
    and
        (at ?robot ?loc1)
    )
    :effect(
    and
        (not (at ?robot ?loc1))
        (at ?robot ?loc2)
    )
)

(:action pick
    :parameters (?robot - robot ?obj - locatable ?loc - location)
    :precondition(
    and
        (free ?robot)
        (at ?obj ?loc)
        (at ?robot ?loc)    
    )
    :effect(
    and
        (not (free ?robot))
        (not (at ?obj ?loc)) 
         (loaded ?obj ?robot)
    )
)

(:action place
    :parameters (?robot - robot ?obj - locatable ?loc - location)
    :precondition(
    and
        (at ?robot ?loc)
        (loaded ?obj ?robot)
    )
    :effect(
    and
        (free ?robot)
        (at ?obj ?loc) 
        (not (loaded ?obj ?robot))
    )
)

(:action place_on_plate
    :parameters (?robot - robot ?obj1 - food ?obj2 - container ?loc - location)
    :precondition(
    and
        (at ?robot ?loc)
        (loaded ?obj1 ?robot)
        (at ?obj2 ?loc)
    )
    :effect(
    and
        (free ?robot)
        (on ?obj1 ?obj2) 
        (not (loaded ?obj1 ?robot))
    )
)
)
