;;Contruction Site PDDL
;;https://popf-cloud-solver.herokuapp.com
(define (domain contruction_site)
(:requirements :strips :typing :fluents :equality :action-costs :negative-preconditions)

;;define the types of element defined into the problem
(:types 
        machine material location - locatable
        pushable_material loadable_material - material 
        waypoint parkingArea - location
    )
    
;;The properties that are used to describe the state of those things
(:predicates
        (forklift ?fk - machine)
        (bulldozer ?bl - machine)
        (wheelloader ?wl - machine)
        (excavator ?ex - machine)
        (can_load ?machine - machine)
        (can_push ?machine - machine)
        (pallet ?mat - material)
        (block ?mat - material)
        (pile ?mat - material)
        (in ?mat - material ?mach - machine)
        (connected ?from ?to - location)
        (at ?obj - locatable ?loc - location)
        (clear ?loc - location)
        (clear_path ?startLoc - location ?goalLoc - location)
        (noMachineAtLocation ?loc - location)
        (idle ?mach - machine)
        (IsStacked ?mat - material ?loc - location)
        (on ?mat1 - material ?mat2 - material ?loc - location)
        (armlow ?mach - machine)
        (loadingMaterial ?mach - machine)
        (dumpingMaterial ?mach - machine)
     
    )
    
(:functions
    (total_cost)
)


;;MACHINE ACTIONS
(:action move
		:parameters (?mc - machine ?from ?to - location)
		:precondition (and 	
		(not (loadingMaterial ?mc))
		(at ?mc ?from)
		(clear ?to)
		(idle ?mc)
		(noMachineAtLocation ?to)
		(clear_path ?from ?to)
		)
		:effect (and 
				(not (at ?mc ?from)) 
				(at ?mc ?to) 
				(noMachineAtLocation ?from)
				(not (noMachineAtLocation ?to))
				(increase (total_cost) 1)
	)
	)


(:action clear_Area
		:parameters (?from ?to - location)
		:precondition (and 	
		(not (clear_path ?from ?to))
		)
		:effect (and 
				(clear_path ?from ?to)
	))




(:action move_with_material
		:parameters (?mc - machine ?from ?to - location ?mat - material)
		:precondition (and 	
		;;(connected ?from ?to)
		(at ?mc ?from)
		(in ?mat ?mc)
		(armlow ?mc )
		(noMachineAtLocation ?to)
		(clear_path ?from ?to)
		(not (loadingMaterial ?mc))
		)
		:effect (and 
				(not (at ?mc ?from)) 
				(at ?mc ?to) 
				(noMachineAtLocation ?from)
				(not (noMachineAtLocation ?to))
				(increase (total_cost) 3)
				(dumpingMaterial ?mc)
	))

		
(:action move_to_take_material
		:parameters (?mc - machine ?from ?to - location ?mat - material)
		:precondition (and 	
		;;(connected ?from ?to)
		(at ?mc ?from)
		(not (loadingMaterial ?mc))
		;;(at ?mat ?to)
		(idle ?mc)
        (at ?mat ?to)
        (noMachineAtLocation ?to)
        (clear_path ?from ?to)
		)
		:effect (and 
				(not (at ?mc ?from)) 
				(at ?mc ?to) 
				(noMachineAtLocation ?from)
				(not (noMachineAtLocation ?to))
				(increase (total_cost) 2)
				(loadingMaterial ?mc)
	))	

	
(:action start_push_material
		:parameters (?mc - machine ?mat - pushable_material ?loc - location)
		:precondition (and 	
        (at ?mc ?loc)
	    (can_push ?mc)
	    (idle ?mc)
	    (not (IsStacked ?mat ?loc))
	    (at ?mat ?loc)
		)
		:effect (and 
				(not (idle ?mc))
				(in ?mat ?mc)
				(increase (total_cost) 10)
				(not (loadingMaterial ?mc))
				(clear ?loc)
				(not (at ?mat ?loc))
	)
	)
	
	
(:action load_material
        :parameters (?mc - machine ?mat - loadable_material ?loc - location)
        :precondition (and
            (at ?mc ?loc)
            (can_load ?mc)
            (idle ?mc)
            (not (IsStacked ?mat ?loc))
            (at ?mat ?loc)
        )
        :effect (and
            (in ?mat ?mc)
            (not (idle ?mc))
            (increase (total_cost) 10)
            (not (loadingMaterial ?mc))
            (clear ?loc)
            (not (at ?mat ?loc))
        )
    )	
	
	
(:action dump_material
        :parameters (?mc - machine ?mat - material ?loc - location)
        :precondition (and
            (at ?mc ?loc)
	        (in ?mat ?mc)
            (clear ?loc)
        )
        :effect (and
            (not (clear ?loc))
            (not (in ?mat ?mc))
            (idle ?mc)
            (increase (total_cost) 10)
            (at ?mat ?loc)
        )
    ) 	


(:action stack_material
        :parameters (?mc - machine ?mat - material ?undermat - material ?loc - location)
        :precondition (and
            (at ?mc ?loc)
	        (in ?mat ?mc)
            (can_load ?mc)
            (at ?undermat ?loc)
            (not (IsStacked ?undermat ?loc))
            (armlow ?mc )
        )
        :effect (and
            (idle ?mc)
            (not (in ?mat ?mc))
            (on ?mat ?undermat ?loc)
            (IsStacked ?undermat ?loc)
            (increase (total_cost) 10)
            (not (armlow ?mc ))
            (at ?mat ?loc)
  
        )
    )

    
    (:action unstack_material
        :parameters (?mc - machine ?mat ?undermat - loadable_material ?loc - location)
        :precondition (and
            (at ?mc ?loc)
	        (idle ?mc)
	        (can_load ?mc)
	        (at ?mat ?loc)
	        (at ?undermat ?loc)
	        (not (IsStacked ?mat ?loc))
            (on ?mat ?undermat ?loc)
            
        )
        :effect (and
        (not (idle ?mc))
        (in ?mat ?mc)
        (increase (total_cost) 10)
        (not (armlow ?mc ))
        (not (loadingMaterial ?mc))
        (not (on ?mat ?undermat ?loc))
        (not (IsStacked ?undermat ?loc))
        (not (at ?mat ?loc))
        )
    )
    
(:action adjust_arm
         :parameters (?mc - machine ?mat - material ?loc - location)
		:precondition (and 	
		(at ?mc ?loc)
		(in ?mat ?mc)
		(not (armlow ?mc))
		)
		:effect (and 
	    (armlow ?mc)
	)
	)
  

)	
