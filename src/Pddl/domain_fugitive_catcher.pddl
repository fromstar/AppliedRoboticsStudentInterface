(define (domain fugitive_catcher)
	(:requirements	:strips :typing :negative-preconditions
					:disjunctive-preconditions
	)

	(:types
		fugitive catcher - robot
		cell gate - location
	)

	(:predicates
		(is_in ?r - robot ?loc - location)
		(connected ?loc_start - location ?loc_end - location)
		(captured ?r - fugitive)
		(no_moves ?r - fugitive)
	)

	(:action move
		:parameters (?r - robot ?loc_start - location ?loc_end - location)
		:precondition
			( and 
				   ( is_in ?r ?loc_start )
			  	   (or  
				   		( connected ?loc_start ?loc_end )
				   		( connected ?loc_end ?loc_start )
					  
				   )
			  	 
			)	
		:effect
			( and 
					( not( is_in ?r ?loc_start ) )
					( is_in ?r ?loc_end )
				 
			)	
	)
	
	; (:action capture
	; 	:parameters (?r_catcher - catcher ?r_fugitive - fugitive ?loc - cell)
	;	:precondition
	;		( and
	;				( is_in ?r_catcher ?loc )
	;			   	( is_in ?r_fugitive ?loc )
	;			   	( no_moves ?r_fugitive )
	;			 
	;		)
	;	
	;	:effect ( captured ?r_fugitive )
	; )

)
