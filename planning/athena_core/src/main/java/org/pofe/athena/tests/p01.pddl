(define (problem block_world)
    (:domain block_world)
    (:objects
        green red - block
        robot1 - robot
    )
    (:init
        (ontable green)
        (ontable red)
        (clear green)
        (clear red)
        (handempty robot1)
    )
    (:goal
        (and
            (on green red)
        )
    )
)