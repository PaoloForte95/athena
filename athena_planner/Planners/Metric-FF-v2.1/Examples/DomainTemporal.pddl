;;Sokoban pddl
;;https://popf-cloud-solver.herokuapp.com
(define (domain sokoban)
(:requirements :strips :typing :fluents :equality :action-costs :negative-preconditions :durative-actions)

;;define the types of element defined into the problem
(:types 
        location locatable  - object
        robot static_obstacle quantity  - locatable
        loadable_materials pushable_material - materials
    )
    
;;The properties that are used to describe the state of those things
(:predicates
        (forklift ?truck - locatable)
        (bulldozer ?truck - locatable)
        (pallet ?materialType - materials)
        (block ?materialType - materials)
        (in ?obj1 - materials ?obj2 - locatable)
        (connected ?from ?to - location)
        (at ?o - object ?l - location)
        (clear ?loc - location)
        (noRobotAtLocation ?loc - location)
        (idle ?rb - robot)
        (IsNotStacked ?b - materials ?loc - location)
        (on ?b1 - materials ?b2 - materials ?loc - location)
        (onGround ?mat  - materials ?loc - location)
        (armlow ?rb - robot)
        (loadingMaterial ?robot - robot)
        
        
        
    )
    
(:functions
    (operationCost ?rb - robot)
    (machine_velocity ?rb - robot )
    (machine_capacity ?rb - robot )
    (material_amount_at_location ?mat - materials ?loc - location)
    (distance ?a ?b - location)
    (total_cost)
    

)

    
;;MACHINE ACTIONS
(:durative-action move
		:parameters (?rb - robot ?from ?to - location)
		:duration (= ?duration (/ (distance ?from ?to) (machine_velocity ?rb)))
		:precondition (and 	
		(over all (connected ?from ?to))
		(at start (not (loadingMaterial ?rb)))
		(at start(at ?rb ?from))
		(at start (clear ?to))
		(at start (idle ?rb))
		(at end (noRobotAtLocation ?to))
		)
		:effect (and 
				(at start(not (at ?rb ?from))) 
				(at start (noRobotAtLocation ?from))
				(at end (at ?rb ?to))
				(at end (not (noRobotAtLocation ?to)))
				(at end (increase (total_cost) (/(distance ?from ?to)(machine_velocity ?rb))))
	)
	)

(:durative-action move_with_material
      :parameters (?rb - robot ?from ?to - location ?mat - material)
      :duration (= ?duration (/ (distance ?from ?to) (machine_velocity ?rb)))
      :condition (and
        (over all (connected ?from ?to))
        (at start  (at ?rb ?from) )
        (at start  (in ?mat ?rb) )
        (at start  (armlow ?rb ))
        (at end  (noRobotAtLocation ?to))
        (at end  (not (loadingMaterial ?rb)))
      )
      :effect (and
        (at start(not (at ?rb ?from)))
        (at end (at ?rb ?to) )
        (at start(noRobotAtLocation ?from))
        (at end (not (noRobotAtLocation ?to)))
        (at end (increase (total_cost) (/(distance ?from ?to)(machine_velocity ?rb))))
      )
)


(:durative-action move_to_take_material
      :parameters (?rb - robot ?from ?to - location ?mat - material)
      :duration (= ?duration (/ (distance ?from ?to) (machine_velocity ?rb)))
      :condition (and
        (over all (connected ?from ?to))
        (at start  (at ?rb ?from) )
        (at start  (not (loadingMaterial ?rb)))
        (at start  (idle ?rb))
        (at start  (>= (material_amount_at_location ?mat ?to) 1))
        (at end  (noRobotAtLocation ?to))
      )
      :effect (and
        (at start(not (at ?rb ?from)))
        (at end (at ?rb ?to))
        (at start(noRobotAtLocation ?from))
        (at end (not (noRobotAtLocation ?to)))
        (at end (increase (total_cost) (/(distance ?from ?to)(machine_velocity ?rb))))
        (at end (loadingMaterial ?rb))
      )
)

(:durative-action start_push_material
      :parameters (?rb - robot ?mat - pushable_material ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start  (at ?rb ?loc) )
        (at start  (bulldozer ?rb))
        (at start  (idle ?rb))
        (at start  (onGround ?mat ?loc))
        (at start  (IsNotStacked ?mat ?loc))
        (at start   (>= (material_amount_at_location ?mat ?loc) 1))
      )
      :effect (and
        (at start (not (idle ?rb)))
        (at end (in ?mat ?rb))
        (at end (decrease (material_amount_at_location  ?mat ?loc) (machine_capacity ?rb)))
        (at start (increase (total_cost) (operationCost ?rb)))
		(at start (not (loadingMaterial ?rb)))
      )
)


(:durative-action load_material
      :parameters (?rb - robot ?mat - loadable_materials ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start  (at ?rb ?loc))
        (at start  (forklift ?rb))
        (at start  (idle ?rb))
        (at start   (onGround ?mat ?loc ))
        (at start  (IsNotStacked ?mat ?loc))
        (at start (>= (material_amount_at_location ?mat ?loc) 1))
      )
      :effect (and
        (at end (in ?mat ?rb))
        (at start (not (idle ?rb)))
        (at end (decrease (material_amount_at_location  ?mat ?loc) (machine_capacity ?rb)))
        (at end (increase (total_cost) (operationCost ?rb)))
        (at start (not (loadingMaterial ?rb)))
      )
)


(:durative-action dump_material
      :parameters (?rb - robot ?mat - materials ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start  (at ?rb ?loc))
        (at start  (in ?mat ?rb))
        (at start (clear ?loc))
      )
      :effect (and
        (at end (not (clear ?loc)))
        (at start (not (in ?mat ?rb)))
        (at end (idle ?rb))
        (at end (onGround ?mat ?loc ))
        (at end (increase (material_amount_at_location ?mat  ?loc ) (machine_capacity ?rb)))
        (at end (increase (total_cost) (operationCost ?rb)))
      )
)


(:action stack_same_material
        :parameters (?rb - robot ?mat - materials ?underox - materials ?loc - location)
        :precondition (and
            (at ?rb ?loc)
	        (in ?mat ?rb)
            ;;(isopen ?underox)
            (>= (material_amount_at_location ?mat ?loc) 1)
            (>= (material_amount_at_location ?underox ?loc) 1)
        )
        :effect (and
            (idle ?rb)
            (not (in ?mat ?rb))
            (increase (material_amount_at_location ?mat ?loc) (machine_capacity ?rb))
            (increase (total_cost) (operationCost ?rb))
        )
    )



(:durative-action stack_same_material
       :parameters (?rb - robot ?mat - materials ?underox - materials ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start  (at ?rb ?loc))
        (at start  (in ?mat ?rb))
        (at start  (>= (material_amount_at_location ?mat ?loc) 1))
        (at start (>= (material_amount_at_location ?underox ?loc) 1))
      )
      :effect (and
        (at end (idle ?rb))
        (at end (not (in ?mat ?rb)))
        (at end (increase (material_amount_at_location ?mat ?loc) (machine_capacity ?rb)))
        (at end (increase (total_cost) (operationCost ?rb)))
      )
)

(:durative-action stack_material
      :parameters (?rb - robot ?mat - materials ?underox - materials ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start  (at ?rb ?loc))
        (at start  (in ?mat ?rb))
        (at start  (forklift ?rb))
        (at start  (<= (material_amount_at_location ?mat ?loc) 1))
        (at start  (>= (material_amount_at_location ?underox ?loc) 1))
      )
      :effect (and
        (at end (idle ?rb))
        (at end (not (in ?mat ?rb)))
        (at end (on ?mat ?underox ?loc))
        (at end (not (IsNotStacked ?underox ?loc)))
        (at end (increase (material_amount_at_location ?mat ?loc) (machine_capacity ?rb)))
        (at end  (increase (total_cost) (operationCost ?rb)))
      )
)

(:durative-action unstack_material
      :parameters (?rb - robot ?mat ?underox - loadable_materials ?loc - location)
      :duration (= ?duration 1)
      :condition (and
        (at start   (at ?rb ?loc))
        (at start  (idle ?rb))
        (at start  (forklift ?rb))
        (at start   (>= (material_amount_at_location ?mat ?loc) 1))
        (at start (>= (material_amount_at_location ?underox ?loc) 1))
        (at start (on ?mat ?underox ?loc))
      )
      :effect (and
        (at end (not (idle ?rb)))
        (at end (in ?mat ?rb))
        (at end (decrease (material_amount_at_location  ?mat ?loc) (machine_capacity ?rb)))
        ;;(at end (material_start_position ?mat ?loc))
        (at start (increase (total_cost) (operationCost ?rb)))
        (at end (not (armlow ?rb )))
        (at end  (not (loadingMaterial ?rb)))

      )
)




(:durative-action adjust_arm
        :parameters (?rb - robot ?mat - materials ?loc - location)
        :duration (= ?duration 1)
		:precondition (and 	
		(at start (at ?rb ?loc))
		(at start (in ?mat ?rb))
		)
		:effect (and 
	    (at end (armlow ?rb))
	)
	)
   
    
    ;;MACHINE ACTIONS
(:durative-action all_material_is_unstacked
		:parameters (?loc - location ?mat ?underox - materials)
		:duration (= ?duration 1)
		:precondition (and 	
	    (at start (<= (material_amount_at_location ?mat ?loc) 1))
        (at start (on ?mat ?underox ?loc))
		)
		:effect (and 
		(at end (not (on ?mat ?underox ?loc)))
        (at end (IsNotStacked ?underox ?loc))
	)
	)

(:durative-action freePosition
        :parameters (?rb - robot ?loc - location ?mat - materials)
        :duration (= ?duration 1)
		:precondition (and 	
	    (at start (<=  (material_amount_at_location ?mat ?loc) 1))
	    (at start (in ?mat ?rb))
		(at start (at ?rb ?loc))
		)
		:effect (and 
		(at end (clear ?loc))
	    )
    )
    








)
