(define (problem object_arrangement)
    (:domain object_arrangement)
    (:objects
        red_plate - plate
        green_apple - food
        blue_mug - mug
        clear_glass - glass
        robot1 - robot
        home wp1s wp2s wp3s wp4s wp5s wp6s - location
        wp1f wp2f wp3f wp4f - location
        out_location - location 
    )
    (:init
        (at red_plate wp1s)
        (at green_apple wp2s)
        (at blue_mug wp3s)
        (at clear_glass wp4s)
        (at robot1 home)
        (free robot1)
    )
    (:goal
        (and
            (at red_plate wp1f)
            (at blue_mug wp2f)
            (at clear_glass wp3f)
            (on green_apple red_plate)
        )
    )
)