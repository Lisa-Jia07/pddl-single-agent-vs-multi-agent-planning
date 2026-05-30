(define (domain SingleRobotRunner)
(:requirements
    :typing
    :negative-preconditions
)

(:types
    robot gridsquare direction
)

(:predicates
    ;Planning Robot location
    (agent-at ?loc - gridsquare)
    
    ;Agent Robot is rotated a particular way
    (agent-rotated ?d - direction)
    
    ;Other immovable robots occupy squares
    (robot-at ?loc - gridsquare)
    
    ;Obstacle on a particular gridsquare
    (obstacle-at ?loc - gridsquare)
    
    ;Grid square adjacent in particular direction
    (gridsquare-connected ?from ?to - gridsquare ?d - direction)
    
    ;Rotation adjacency
    (rotation-adjacent ?from ?to - direction)
)

;The Agent Robot can move if
;  - The Agent Robot is at the current location,
;  - The Agent Robot is rotated towards the square,
;  - There is no Robot at the destination,
;  - The Agent Robot is not already at the destination location.
;Effects: Move the Agent Robot to the destination.
(:action moveforward
    :parameters (?from - gridsquare ?to - gridsquare ?d - direction)
    :precondition (and 
        (agent-at ?from)
        (agent-rotated ?d)
        (gridsquare-connected ?from ?to ?d)
        (not (robot-at ?to))
        (not (obstacle-at ?to)) 
    )
    :effect (and 
        (not (agent-at ?from))
        (agent-at ?to)
    )
)

;The Agent Robot can rotate the direction it is currently facing from its old 
; direction to a new direction if
;  - The Agent Robot is at the Current Rotation,
;  - The Destination Rotation is an adjacent rotation to the Current Rotation,
;Effects: Switch the rotation of the agent to the new rotation
(:action rotate
    :parameters (?fromdir - direction ?todir - direction)
    :precondition (and 
        (agent-rotated ?fromdir)
        (rotation-adjacent ?fromdir ?todir)
    )
    :effect (and 
        (not (agent-rotated ?fromdir))
        (agent-rotated ?todir)
    )
)

)
