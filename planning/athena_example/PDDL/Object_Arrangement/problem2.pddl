(define (problem table_arrangement)
(:domain table_arrangement)
(:objects
red_plate - plate
pear - food
basket - container
knife - object
spoon - object
fork - object
wp1s wp2s wp3s wp4s wp5s wp6s wp7s wp8s wp9s - location
wp1f wp2f wp3f wp4f wp5f wp6f - location
out_location - location
)
(:init
(at red_plate wp1s)
(at pear wp2s)
(at basket wp5s)
(at knife wp7s)
(at spoon wp8s)
(at fork wp9s)
)
(:goal
(and
(at red_plate wp1f)
(at basket wp2f)
(at knife wp3f)
(at spoon wp4f)
(at fork wp5f)
(on pear red_plate)
)
)
)