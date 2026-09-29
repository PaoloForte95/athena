(define (problem block_world)
    (:domain block_world)
    (:objects
        green red white yellow - block
        robot1 robot2 - robot
    )
    (:init
        (ontable green)
        (ontable red)
        (ontable white) 
        (ontable yellow)
        (clear green)
        (clear red)
        (clear white) 
        (clear yellow)
        (handempty robot1)
        (handempty robot2)
    )
    (:goal
        (and
            (on green red)
            (on white yellow)
        )
    )
)